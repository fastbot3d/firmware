/*
 * Unicorn 3D Printer Firmware
 * heater.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include "common.h"
#include "unicorn.h"
#include "pwm.h"
#include "temp.h"
#include "heater.h"

typedef struct {
    channel_tag  id;
    channel_tag  input;
    channel_tag  output;
    double       setpoint;
    pid_settings pid;
    double       pid_integral;
    double       celsius_history[8];
    unsigned int history_idx;
    int          log_fd;
} heater_t;

static heater_t *heaters = NULL;
static unsigned int nr_heaters = 0;

static pthread_t heater_thread;
static bool thread_quit = false;
static pthread_rwlock_t lock;

static double clip(double min, double val, double max)
{
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    }
    
    return val;
}

static int heater_index_lookup(channel_tag heater_channel)
{
    int idx;

    for (idx = 0; idx < nr_heaters; idx++) {
        if (heaters[idx].id == heater_channel) {
            return idx;
        }
    }

    if DBG(D_HEATER) {
        fprintf(stderr, "heater_index_lookup failed for '%s'\n",
                tag_name(heater_channel));
    }

    return -1;
}

channel_tag heater_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_heaters; idx++) {
        channel_tag tag = heaters[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

channel_tag heater_lookup_by_index(int idx) 
{
    if (idx >= 0 || idx < nr_heaters) {
        return heaters[idx].id; 
    } else {
        return NULL;
    }
}

static int log_file_open(const char* fname)
{
    char s[270];
    snprintf(s, sizeof(s), "./pid-%s.log", fname);

    if (DBG(D_HEATER)) {
        printf("log_file_open - looking for existing logfile named '%s'\n", s);
    }

    int fd = open(s, O_WRONLY | O_APPEND);
    if (fd < 0) {
        perror("Failed to open logile for append, logging disabled");
        return -1;
    }

    if (DBG(D_HEATER)) {
        printf("log_file_open - appending to file '%s'\n", s);
    }

    snprintf(s, sizeof(s), 
            "--------------------------------------------------------------------------------------\n"
            "   time   channel         setpoint   temp       ff       p        i        d     out%%\n"
            "--------------------------------------------------------------------------------------\n");
    write(fd, s, strlen(s));
    return fd;
}

static void log_entry(const char* name, int fd, time_t time, 
                      double setpoint, double celsius, double error,
                      double out_ff, double out_p, double out_i, double out_d, 
                      int duty_cycle)
{
    char s[120];
    snprintf(s, sizeof(s), "%7ld   %-14s   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %3d\n",
                            time, name,   setpoint, celsius, out_ff,  out_p,   out_i,   out_d,  duty_cycle);
    write(fd, s, strlen(s));

    if (DBG(D_HEATER)) {
        printf(s);
    }
}

#define PID_LOOP_FREQ 2                     /* HZ */
#define TIMER_CLOCK   CLOCK_MONOTONIC
#define NS_PER_SEC    (1000 * 1000 * 1000)

static void ns_sleep(struct timespec *ts, unsigned int ns)
{
    struct timespec tso;

    ts->tv_nsec += ns;
    if (ts->tv_nsec >= NS_PER_SEC) {
        ts->tv_nsec -= NS_PER_SEC;
        ts->tv_sec  += 1;
    }

    for (;;) {
        if (clock_nanosleep( TIMER_CLOCK, TIMER_ABSTIME, ts, &tso) == -1) {
            if (errno == EINTR) {
                printf( "  sleep interrupted, tso = %ld.%09ld\n", tso.tv_sec, tso.tv_nsec);
                *ts = tso;
            } else {
                perror( "clock_nanoslaap failed");
                break;
            }
        } else {
            break;
        }
    }
}

void *heater_thread_worker(void *arg)
{
    int i;
    struct timespec ts;
    unsigned int timer_period;

    int log_scaler;

    if (nr_heaters < 1) {
        goto out;
    }
    
    fprintf(stderr, "heater_thread: started\n");

    timer_period = NS_PER_SEC / (PID_LOOP_FREQ * nr_heaters);
    clock_getres(TIMER_CLOCK, &ts);
    clock_gettime(TIMER_CLOCK, &ts);

    while (!thread_quit) 
    {
        for (i = 0; i < nr_heaters; i++) { 
            
            if (thread_quit) {
                goto out;
            }
            
            heater_t *p   = &heaters[i];
            channel_tag input  = p->input; 
            channel_tag output = p->output;
            double celsius;

            if (temp_get_celsius(input, &celsius) < 0) {
                fprintf(stderr, "heater_thread: failed to read temp '%s'\n", tag_name(input));
            } else {
                if (p->setpoint == 0.0) { 
                    if (pwm_get_state(output) == PWM_STATE_ON) {
                        pwm_set_output(output, 0);
                    }
                } else {
                    double t_error = p->setpoint - celsius;
                    const double dt = 1.0 / PID_LOOP_FREQ;

                    /* proportional part */
                    double heater_p = t_error;

                    /* integral part, prevent integrator wind-up */
                    double heater_i = clip(-p->pid.I_limit, 
                                            p->pid_integral + t_error * dt,
                                            p->pid.I_limit);
                    p->pid_integral = heater_i;

                    /* derivative */
                    p->celsius_history[p->history_idx] = celsius;
                    if (++(p->history_idx) >= NR_ITEMS(p->celsius_history)) {
                        p->history_idx = 0;
                    }
                    
                    double old_celsius = p->celsius_history[p->history_idx];
                    if (old_celsius == 0.0) {
                        old_celsius = celsius;
                    }

                    double heater_d = (celsius - old_celsius) 
                                      / ((NR_ITEMS(p->celsius_history) - 1) * dt);

                    /* combine factors */
                    double out_p = heater_p * p->pid.P;
                    double out_i = heater_i * p->pid.I;
                    double out_d = heater_d * p->pid.D;
                    double out_ff= (p->setpoint - p->pid.FF_offset) * p->pid.FF_factor;
                    double out   = out_p + out_i + out_d + out_ff;

                    int duty = (int) clip(0.0, out, 100.0);
                    
                    if (strcmp(tag_name(p->id), "heater_ext") == 0) {
                        if (duty > 99) {
                            duty = 99;
                        }
                    }
                    if (strcmp(tag_name(p->id), "heater_bed") == 0) {
                        if (duty > 40) {
                            duty = 40;
                        }
                    }

                    if (log_scaler == 0) {
                        if (DBG(D_HEATER)) {
                            log_entry(tag_name(input), p->log_fd, ts.tv_sec, p->setpoint,
                                    celsius, t_error, out_ff, out_p, out_i, out_d, duty);
                        }
                    }

                    /* set pwm output duty */
                    pwm_set_output(output, duty);
                }
            }

            ns_sleep(&ts, timer_period);
        }

        if (++log_scaler >= 1 * PID_LOOP_FREQ) {
            log_scaler = 0;
        }
    }

out:
    thread_quit = false;
    pthread_exit(NULL);
}

int heater_config(heater_config_t *pcfgs, int nr_cfgs)
{
    int i;

    if DBG(D_HEATER) {
        printf("heater_config called with %d records\n", nr_cfgs);
    }
    
    if (!pcfgs || nr_cfgs <= 0) {
        return -1; 
    }
    
    heaters = calloc(nr_cfgs, sizeof(heater_t));
    if (!heaters) {
        return -1;
    }
    nr_heaters = 0;

    for (i = 0; i < nr_cfgs; i++) {
        heater_t *pd = &heaters[i];
        heater_config_t *ps = &pcfgs[i];

        pd->id = ps->tag;

        pd->input  = ps->temp;
        pd->output = ps->pwm;

        pd->pid.P  = ps->pid.P;
        pd->pid.I  = ps->pid.I;
        pd->pid.D  = ps->pid.D;

        pd->pid.FF_factor = ps->pid.FF_factor;
        pd->pid.FF_offset = ps->pid.FF_offset;
        pd->pid.I_limit   = ps->pid.I_limit;

        pd->setpoint     = 0.0;
        pd->history_idx  = 0;
        pd->pid_integral = 0.0;
        
        pd->log_fd = -1;
        nr_heaters++;
    }

    return 0;
}

int heater_init(void)
{
    int ret = 0;

    if DBG(D_HEATER) {
        printf("heater_init called.\n");
    }
    
    if (!heaters || nr_heaters <= 0) {
        return -1;
    }

    /* Start heater work thread */
    thread_quit = false;
    ret = sub_sys_thread_create("heater", &heater_thread, NULL, &heater_thread_worker, NULL);
    if (ret) {
        return -1;
    } 

    return ret;
}

void heater_exit(void)
{
    int i;

    if (DBG(D_HEATER)) {
        printf("heater_exit called.\n");
    }
    
    thread_quit = true;
    pthread_join(heater_thread, NULL); 

    for (i = 0; i < nr_heaters; i++) {
        heater_t *pd = &heaters[i];
        if (pwm_get_state(pd->output) == PWM_STATE_ON) {
            pwm_set_output(pd->output, 0);
            pwm_disable(pd->output);
        }
    }
}

void heater_stop(void)
{
    if (!thread_quit) {
        thread_quit = true;
    }
}
/*
 *  get current temperature for a heater
 */
int heater_get_celsius(channel_tag heater, double *pcelsius)
{
    if (pcelsius) {
        int idx = heater_index_lookup(heater);
        if (idx >= 0) {
            return temp_get_celsius(heaters[idx].input, pcelsius);
        }
    }
    return -1;
}

int heater_temp_reached(channel_tag heater)
{
    if (heater) {
        int idx = heater_index_lookup( heater);
        if (idx >= 0) {
            return temp_achieved(heaters[idx].input);
        }
        return 0;
    }

    return 1;
}
/*
 *  turn heater on or off
 */
int heater_enable(channel_tag heater, int state)
{
    int idx = heater_index_lookup(heater);
    if (state) {
        pwm_enable(heaters[idx].output);
    } else {
        pwm_disable(heaters[idx].output);
    }

    return 0;
}
/*
 *  set heater pwm output for given channel to value
 */
int heater_set_raw_pwm(channel_tag heater, double percentage)
{
    return 0;
}
/*
 *  set setpoint for a heater
 */
int heater_set_setpoint(channel_tag heater, double setpoint)
{
    int idx = heater_index_lookup(heater);
    if (idx < 0) {
        return -1;
    }
    
    printf("heater_set_setpoint %f\n", setpoint);
    heater_t *p = &heaters[idx]; 
    
    pthread_rwlock_wrlock(&lock);
    p->setpoint = setpoint;
    pthread_rwlock_unlock(&lock);
    
    if (setpoint == 0.0) {
        if (p->log_fd >= 0) {
            close(p->log_fd);
            if (DBG(D_HEATER)) {
                printf("heater_set_point - logfile closed.\n");
            }
        }
        p->log_fd = -1;
    } else {
        if (p->log_fd == -1) {
            p->log_fd = log_file_open(tag_name(p->id));
        }
    }

    temp_set_setpoint(p->input, setpoint, -4.5, 2.5);
    return 0;
}
/*
 *  get setpoint for a heater
 */
int heater_get_setpoint(channel_tag heater, double *setpoint)
{
    int idx;
    if (!setpoint) {
        return -1;
    }

    idx = heater_index_lookup(heater); 
    if (idx < 0) {
        return -1;
    }

    pthread_rwlock_rdlock(&lock);
    *setpoint = heaters[idx].setpoint;
    pthread_rwlock_unlock(&lock);
    return 0;
}
/*
 * set PID values for a heater
 */
int heater_set_pid_values(channel_tag heater, const pid_settings *pid)
{
    int idx;
    if (!pid) { 
        return -1;
    }
    
    idx = heater_index_lookup(heater);
    if (idx < 0) {
        return -1;
    }
    
    pthread_rwlock_wrlock(&lock);
    heaters[idx].pid = *pid;
    pthread_rwlock_unlock(&lock);

    return 0;
}
/*
 * get PID values for a heater
 */
int heater_get_pid_values(channel_tag heater, pid_settings *pid)
{
    int idx;
    if (!pid) { 
        return -1;
    }
    
    idx = heater_index_lookup(heater);
    if (idx < 0) {
        return -1;
    }
    
    pthread_rwlock_rdlock(&lock);
    *pid = heaters[idx].pid;
    pthread_rwlock_unlock(&lock);

    return 0;
}

const char* fname = "./heater-pid-factors";
/*
 * save pid factors to persistent storage
 */
int heater_save_settings(void)
{
    int fd;
    int ret;
  
    fd = open(fname, O_WRONLY);
    if (fd < 0) {
        perror("heater: opening of file '%s' failed");
        return -1;
    }

    pthread_rwlock_rdlock(&lock);
    ret = write(fd, heaters, sizeof(heaters));
    pthread_rwlock_unlock(&lock);

    if (ret < 0) {
        perror("heater: writing file '%s' failed");
        return ret;
    } else if (ret != sizeof(heaters)) {
        return -1;
    }

    return 0;
}
/*
 * load pid factors from persistent storage
 */
int heater_load_settings(void)
{
    int fd;
    int ret;
  
    fd = open(fname, O_WRONLY);
    if (fd < 0) {
        perror( "heater: opening of file '%s' failed");
        return -1;
    }

    pthread_rwlock_wrlock(&lock);
    ret = read(fd, heaters, sizeof(heaters));
    pthread_rwlock_unlock(&lock);
    if (ret < 0) {
        perror("heater: reading file '%s' failed");
        return ret;
    } else if (ret != sizeof(heaters)) {
        return -1;
    }

    return 0;
}

