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
#include <math.h>
#include <sys/time.h>

#include "common.h"
#include "unicorn.h"
#include "pwm.h"
#include "temp.h"
#include "gcode.h"
#include "heater.h"
#include "parameter.h"

typedef struct {
    channel_tag  id;
    channel_tag  input;
    channel_tag  output;
    double       setpoint;
    pid_settings pid;
    double       pid_integral;
    double       celsius_history[2];
    //lkj double       celsius_history[8];
    unsigned int history_idx;
    int          log_fd;
} heater_t;

static heater_t *heaters = NULL;
static heater_t *heaters_bak = NULL;  //lkj 

static unsigned int nr_heaters = 0;

static pthread_t heater_thread;
static bool thread_quit = false;
//static pthread_rwlock_t lock; for android
static pthread_mutex_t lock;

#define MAX_DANGEROUS_CELSIUS  280
#define MAX_THERMOCOUPLE_DANGEROUS_CELSIUS  1100

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

    HEATER_DBG("heater_index_lookup failed for '%s'\n",
                tag_name(heater_channel));

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

    PID_DBG("log_file_open - looking for existing logfile named '%s'\n", s);

    int fd = open(s, O_WRONLY | O_APPEND);
    if (fd < 0) {
        perror("Failed to open logile for append, logging disabled");
        return -1;
    }

    PID_DBG("log_file_open - appending to file '%s'\n", s);

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
    char s[120] = {0};
    snprintf(s, sizeof(s), "%7ld   %-14s   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %3d\n",
                            time, name,   setpoint, celsius, out_ff,  out_p,   out_i,   out_d,  duty_cycle);

    if (DBG(D_PID)) {
        write(fd, s, strlen(s));
    }

    if (DBG(D_HEATER)) {
    	printf("%7ld   %-14s   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %6.2lf   %3d\n",
                            time, name,   setpoint, celsius, out_ff,  out_p,   out_i,   out_d,  duty_cycle);
    }
}

#define PID_LOOP_FREQ 2                 /* HZ */
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

static unsigned long millis()
{
    unsigned long ms;
    struct timespec ts;
    clock_gettime(TIMER_CLOCK, &ts);
    ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    return ms;
}

#define constrain(val,low,high) ((val)<(low)?(low):((val)>(high)?(high):(val)))


static bool auto_tune_pid = false;
void pid_autotune(float target_temp, int extruder, int ncycles, int w, int (*gcode_send_response_remote)(char *))
{
    char *all_heater_name[] = {"heater_ext", "heater_ext2", "heater_ext3", "heater_bed"};
    char *all_pwm_name[] =    {"pwm_ext", "pwm_ext2", "pwm_ext3",  "pwm_bed"};
    unsigned char heater_id = 0;
    int cycles = 0;
    unsigned char soft_pwm_bed;
    unsigned char soft_pwm[MAX_EXTRUDER];
    long bias, d;  
    bool heating = true;

    float Kp, Ki, Kd;
    float Ku, Tu;

    unsigned long temp_millis;
    unsigned long t1;
    unsigned long t2;
    long t_high = 0;
    long t_low = 0;
    
    double current_temp= 0.0;
    float max_temp = 0.0;
    float min_temp = 1000.0;

	int max_heat_pwm_bed;
    int max_heat_pwm_hotend;
	bool is_thermocouple = false;

    channel_tag pwm;
    channel_tag heater;

    printf("--------------------------------------------------\n");
    printf("PID autotune start\n");
    printf("M303 E%d, S%f, C%d\n", extruder, target_temp, ncycles);
    char send_buf[200] = {0};

    memset(send_buf, 0, sizeof(send_buf));
    sprintf(send_buf, "M303 E%d, S%f, C%d\n", extruder, target_temp, ncycles);
    gcode_send_response_remote(send_buf);

	if (w != 0) {
		max_heat_pwm_bed = pa.max_heat_pwm_bed;
		max_heat_pwm_hotend = pa.max_heat_pwm_hotend;
	} else {
		max_heat_pwm_bed = pa.max_heat_pwm_bed/2;
		max_heat_pwm_hotend = pa.max_heat_pwm_hotend/2;
	}
    /* Disable all heaters */
    pwm = pwm_lookup_by_name("pwm_ext");
    pwm_set_output(pwm, 0);

    pwm = pwm_lookup_by_name("pwm_ext2");
    pwm_set_output(pwm, 0);

	pwm = pwm_lookup_by_name("pwm_ext3");
	pwm_set_output(pwm, 0);

    pwm = pwm_lookup_by_name("pwm_bed");
    pwm_set_output(pwm, 0);

    /* Get current time: ms */
    temp_millis = millis();
    t1 = temp_millis; 
    t2 = temp_millis;

    if (extruder < 0) {
        heater_id = 3; 
    } else {
        heater_id = extruder; 
    }

	auto_tune_pid = true;

    pwm = pwm_lookup_by_name(all_pwm_name[heater_id]);
	if (pwm_get_state(pwm) == PWM_STATE_OFF) {
		printf("enable %s\n", all_pwm_name[heater_id]);
		pwm_enable(pwm);
	}

	if (extruder < 0) {
        soft_pwm_bed = max_heat_pwm_bed;
        bias = d = max_heat_pwm_bed;
        pwm_set_output(pwm, soft_pwm_bed);
		printf("output %s, %d\n", all_pwm_name[heater_id], soft_pwm_bed);
    } else {
        soft_pwm[extruder] = max_heat_pwm_hotend;
        bias = d = max_heat_pwm_hotend;
        pwm_set_output(pwm, soft_pwm[extruder]);
    }

    heater = heater_lookup_by_name(all_heater_name[heater_id]);
    if (heater) {
        heater_get_celsius(heater, &current_temp);
		int idx = heater_index_lookup(heater);
		channel_tag input = heaters[idx].input;
		if ((!strcmp(input, "temp_ext3"))
              || (!strcmp(input, "temp_ext4"))
              || (!strcmp(input, "temp_ext5"))){
			is_thermocouple = true;
		}
        printf("heater:%s, heater id:%d, cur_temp:%f, is thermocouple:%d\n", heater, heater_id, current_temp, is_thermocouple);
	}

    while (1) {
        heater_get_celsius(heater, &current_temp);
    
        //printf("curr %f\n", current_temp);
		if (current_temp <=0) {
			memset(send_buf, 0, sizeof(send_buf));
			sprintf(send_buf, "%s", "PID autotune failed! Temperature problem\n");
			gcode_send_response_remote(send_buf);
            pwm_set_output(pwm, 0);
			auto_tune_pid = false;
			return;
		}

        max_temp = max(max_temp, current_temp);    
        min_temp = min(min_temp, current_temp);

        if (heating == true && current_temp > target_temp) {
            if (millis() - t2 > 5000) {
                heating = false;
                
                if (extruder<0) {
                    soft_pwm_bed = (bias - d) >> 1;
                    pwm_set_output(pwm, soft_pwm_bed);
                } else {
                    soft_pwm[extruder] = (bias - d) >> 1;
                    pwm_set_output(pwm, soft_pwm[extruder]);
                }


                t1 = millis();
                t_high = t1 - t2;
                max_temp = target_temp;
            }
        }

        if (heating == false && current_temp < target_temp) { 
            if (millis()  - t1 > 5000) {
                heating = true;
                t2 = millis();
                t_low = t2 - t1;

                if (cycles > 0) {
                    /* calculate pid params */     
                    bias += (d * (t_high - t_low)) / (t_low + t_high);
                    bias = constrain(bias, 20, (extruder<0?(max_heat_pwm_bed):(max_heat_pwm_hotend)) - 20);

                    if (bias >  (extruder<0?(max_heat_pwm_bed):(max_heat_pwm_hotend))) {
                        d = (extruder<0?(pa.max_heat_pwm_bed):(pa.max_heat_pwm_hotend)) - 1 - bias;
                    } else {
                        d = bias;
                    }

                    printf("bias : %ld\n", bias);
                    printf("d    : %ld\n", d);
                    printf("min  : %f\n", min_temp);
                    printf("max  : %f\n", max_temp);

                    if (cycles > 2) {
                        Ku = (4.0 * d) / (3.14159 * (max_temp - min_temp) / 2.0);
                        Tu = ((float)(t_low + t_high) / 1000.0);
                        printf("Ku   : %f\n", Ku);
                        printf("Tu   : %f\n", Tu);
						#if 1
						Kp = 0.6*Ku;
						Ki = 2*Kp/Tu;
						Kd = Kp*Tu/8;
						printf(" Classic PID ");
						printf(" Kp:%f", Kp);
						printf(" Ki:%f", Ki);
						printf(" Kd:%f\n", Kd);
						#endif
						#if 0
						   Kp = 0.33*Ku;
						   Ki = Kp/Tu;
						   Kd = Kp*Tu/3;
						   printf(" Some overshoot ");
						   printf(" Kp:%f ", Kp);
						   printf(" Ki:%f ", Ki);
						   printf(" Kd:%f \n", Kd);
						#endif
						#if 0
						   Kp = 0.2*Ku;
						   Ki = 2*Kp/Tu;
						   Kd = Kp*Tu/3;
						   printf(" No overshoot ");
						   printf(" Kp:%f ", Kp); 
						   printf(" Ki:%f ", Ki); 
						   printf(" Kd:%f ", Kd); 
					   #endif
				   	   memset(send_buf, 0, sizeof(send_buf));
					   sprintf(send_buf, "Class PID \nKp   : %f\nKi   : %f\nKd   : %f\n", Kp, Ki, Kd);
					   gcode_send_response_remote(send_buf);
					}
				}
                if (extruder<0) {
                    soft_pwm_bed = (bias + d) >> 1;
                    pwm_set_output(pwm, soft_pwm_bed);
                } else {
                    soft_pwm[extruder] = (bias + d) >> 1;
                    pwm_set_output(pwm, soft_pwm[extruder]);
                }

                cycles++;
                min_temp = target_temp;
            }
        }

		if (is_thermocouple){
       		usleep(500000); 
		} else {
       		usleep(10000); 
		}

        if (current_temp > target_temp + 20) {
            printf("PID autotune failed! Temperature too high\n");
            pwm_set_output(pwm, 0);

            memset(send_buf, 0, sizeof(send_buf));
            sprintf(send_buf, "%s", "PID autotune failed! Temperature too high\n");
            gcode_send_response_remote(send_buf);
			auto_tune_pid = false;
            return;
        }

        if(millis() - temp_millis > 2000) {
            int p;
            if (extruder<0) {
                p=soft_pwm_bed; 
                printf("ok B:");
            } else {
                p=soft_pwm[extruder];       
                printf("ok T:");
            }
            printf("temp: %f@%d\n", current_temp, p);
            temp_millis = millis();

            memset(send_buf, 0, sizeof(send_buf));
            sprintf(send_buf, "Current temperature:%f, pwm:%d\n", current_temp, p);
            gcode_send_response_remote(send_buf);
        }

        //if (((millis() - t1) + (millis() - t2)) > (10L * 60L * 1000L * 2L)) { //20min
        if (((millis() - t1) + (millis() - t2)) > (10L * 60L * 1000L * 2L)) { //10min
            printf("PID Autotune failed! timeout\n");
            pwm_set_output(pwm, 0);

            memset(send_buf, 0, sizeof(send_buf));
            sprintf(send_buf, "%s", "PID Autotune failed! timeout\n");
            gcode_send_response_remote(send_buf);
			auto_tune_pid = false;
            return;
        }
        
        if (cycles > ncycles) {
            printf("PID Autotune finished!\n");
            printf("Kp   : %f\n", Kp);
            printf("Ki   : %f\n", Ki);
            printf("Kd   : %f\n", Kd);
            pwm_set_output(pwm, 0);

            memset(send_buf, 0, sizeof(send_buf));
            sprintf(send_buf, "PID Autotune finished!\nKp   : %f\nKi   : %f\nKd   : %f\n", Kp, Ki, Kd);
            gcode_send_response_remote(send_buf);
			auto_tune_pid = false;
            return;
        }
    }
	auto_tune_pid = false;
    printf("PID autotune end\n");
    printf("--------------------------------------------------\n");
}


void *heater_thread_worker(void *arg)
{
    int i;
    struct timespec ts;
    unsigned int timer_period;

    int log_scaler;

    int temp_ext_respond = 0;
    int temp_bed_respond = 0;            
    double temp_ext[MAX_EXTRUDER] = {0.0};
    double temp_bed = 0.0;            
    double setpoint_ext[MAX_EXTRUDER] = {0.0};
    double setpoint_bed = 0.0;            
    char buf[150];
	struct timeval start;
	struct timeval end;
	unsigned long timeuse;

    if (nr_heaters < 1) {
        goto out;
    }
    
	HEATER_DBG("heater_thread: started\n");

    timer_period = NS_PER_SEC / (PID_LOOP_FREQ * nr_heaters);
    clock_getres(TIMER_CLOCK, &ts);
    clock_gettime(TIMER_CLOCK, &ts);

    while (!thread_quit) 
    {
        for (i = 0; i < nr_heaters; i++) { 
            
            if (thread_quit) {
                goto out;
            }

			gettimeofday(&start, NULL);
            heater_t *p   = &heaters[i];
            channel_tag input  = p->input; 
            channel_tag output = p->output;
            double celsius;

            if ((i>=pa.ext_count) && strncmp(input, "temp_bed", 8)) { //at most 3
                continue;
            }
//printf("lkj i=%d, input=%s\n", i, input);
            if (temp_get_celsius(input, &celsius) < 0) {
				if (pwm_get_state(output) == PWM_STATE_ON && auto_tune_pid == false) {
					pwm_set_output(output, 0);
				}
				if (strncmp(tag_name(p->id), "heater_ext", 10) == 0) {
					temp_ext[i] = -1000;
				}
				if (strncmp(tag_name(p->id), "heater_bed", 10) == 0) {
					temp_bed = -1000;
				}
                TEMP_DBG("heater_thread: failed to read temp '%s'\n", tag_name(input));
			} else {
                if (p->setpoint == 0.0) { 
                    if (pwm_get_state(output) == PWM_STATE_ON && auto_tune_pid == false) {
                        pwm_set_output(output, 0);
            			pwm_disable(output);
                    }

                    if (strncmp(tag_name(p->id), "heater_ext", 10) == 0) {
                        temp_ext[i] = celsius;
                        setpoint_ext[i] = 0.0;
                        temp_ext_respond &= ~(1<<i);

                        if (unicorn_get_mode() == FW_MODE_REMOTE) { //measure many thermistor for a extruder 
							int j = 0;
							for (j = 0; (j < pa.ext_count) && (i != j); j++) {
								heater_t *heat   = &heaters[j];
								if (!strcmp(heat->input, input) && heat->setpoint > 0){								
									if (temp_achieved(heat->input)) {
										TEMP_DBG("heater %d found frend %d reach\n", i, j);
										temp_ext_respond |= 1<<i;
									} else {
										temp_ext_respond &= ~(1<<i);
									}
								}
							}
                        }
                    }
                    if (strncmp(tag_name(p->id), "heater_bed", 10) == 0) {
                        temp_bed = celsius;
                        setpoint_bed = 0.0;
                        temp_bed_respond = 0;
                    }
                } else {
					/* if pwm stil not enabled, enable it */
                    if (pwm_get_state(output) == PWM_STATE_OFF) {
                        pwm_enable(output);
                    }

                    double t_error = p->setpoint - celsius;
                    const double dt = 1.0 / PID_LOOP_FREQ * nr_heaters;

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
                    double out_ff = 0 ;	//(p->setpoint - p->pid.FF_offset) * p->pid.FF_factor;
                    double out   = out_p + out_i - out_d + out_ff;

                    int duty = (int) clip(0.0, out, 100.0);
                    
                    if (strncmp(tag_name(p->id), "heater_ext", 10) == 0) {
                        if (duty > pa.max_heat_pwm_hotend) {
                            duty = pa.max_heat_pwm_hotend;
                        }

                        if (unicorn_get_mode() == FW_MODE_REMOTE) {
                            temp_ext[i] = celsius;
                            setpoint_ext[i] = p->setpoint;
							if (temp_achieved(p->input)) {
								temp_ext_respond |= 1<<i;
                            } else {
								temp_ext_respond &= ~(1<<i);
                            }
                        }
                    }

                    if (strncmp(tag_name(p->id), "heater_bed", 10) == 0) {
                        if (duty > pa.max_heat_pwm_bed) {
                            duty = pa.max_heat_pwm_bed;
                        }

                        if (unicorn_get_mode() == FW_MODE_REMOTE) {
                            temp_bed = celsius;
                            setpoint_bed = p->setpoint;
                            if (temp_achieved(p->input)) {
                                temp_bed_respond = 1;
                            } else {
                                temp_bed_respond = 0;
                            }
                        }
                    }

                    if (log_scaler == 0) {
                        if (DBG(D_HEATER)) {
                            log_entry(tag_name(input), p->log_fd, ts.tv_sec, p->setpoint,
                                    celsius, t_error, out_ff, out_p, out_i, out_d, duty);
                        }
                    }

					if ((pa.thermocouple_max6675_cnnection == 1 && strncmp(tag_name(p->id), "heater_ext1", 11) == 0)
						   || (pa.thermocouple_max6675_cnnection == 2 && strncmp(tag_name(p->id), "heater_ext2", 11) == 0)
						   || (pa.thermocouple_max6675_cnnection == 3 && strncmp(tag_name(p->id), "heater_ext3", 11) == 0) 
					       || (pa.thermocouple_ad597_cnnection == 1 && strncmp(tag_name(p->id), "heater_ext1", 11) == 0)
						   || (pa.thermocouple_ad597_cnnection == 2 && strncmp(tag_name(p->id), "heater_ext2", 11) == 0)
						   || (!strcmp(input, "temp_ext3"))
						   || (!strcmp(input, "temp_ext4"))
						   || (!strcmp(input, "temp_ext5"))
						   || (!strcmp(input, "temp_ext6"))
						   || (pa.thermocouple_ad597_cnnection == 3 && strncmp(tag_name(p->id), "heater_ext3", 11) == 0) ) {
						if ( celsius >= pa.dangerousThermocouple) {
							printf("thermocouple temp >=%d !\n", pa.dangerousThermocouple);
							pwm_set_output(output, 0);
						} else { 
							pwm_set_output(output, duty);
						}
					} else {
						/* set pwm output duty */
						if ( celsius >= pa.dangerousThermistor) {
							printf("temp >=%d !\n", pa.dangerousThermistor);
							pwm_set_output(output, 0);
						} else { 
							pwm_set_output(output, duty);
						}
					}
                }
            }

            ns_sleep(&ts, timer_period);

			gettimeofday(&end, NULL);
			timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
			//#define HEATE_100_MS (100000)	
			#define HEATE_100_MS (200000)	
			if ( timeuse < HEATE_100_MS) { //us
				//printf("usleep: %ld us\n", (HEATE_100_MS - timeuse));
            	usleep(HEATE_100_MS - timeuse);
			}
        }

        if (!thread_quit) {
            if (++log_scaler >= 3 * PID_LOOP_FREQ) {
                log_scaler = 0;

                if (unicorn_get_mode() == FW_MODE_REMOTE) {
                    /* Send respond to remote when temp not rearch */
                    memset(buf, 0, sizeof(buf));
					int buf_pos = 0;

					for (i = 0; i < pa.ext_count; i++) {
						sprintf(buf + buf_pos, "   T%d:%.2f %.2f ",  i, temp_ext[i], setpoint_ext[i]);
						buf_pos += 30;
					}

					bool share_heater = false;
					for (i = 0; i < pa.ext_count; i++) {
						heater_t *heat   = &heaters[i];
						channel_tag input  = heat->input;
							int j = 0; //check not share a heater 
							for (j = 0; (j < pa.ext_count) && (i != j); j++) {
								heater_t *heat2   = &heaters[j];
								if (!strcmp(heat2->input, input) && heat2->setpoint > 0) {								
									share_heater = true;
									break;
								}
							}
							if (share_heater) {
								break;
							}
					}

					int temp_ext_ok = pow(2.0, pa.ext_count) -1;
					if (!share_heater) {
						for (i = 0; i < pa.ext_count; i++) {
							heater_t *heat   = &heaters[i];
							//printf("i=%d, heat->setpoint:%f\n", i, heat->setpoint);
							if (heat->setpoint == 0.0) {
								temp_ext_ok &= ~(1<<i);
							}
						}
					}

                    bool bed_has_setpoint = false;
					if (pa.has_bed) {
						sprintf(buf + buf_pos, "B:%.2f %.2f ", temp_bed, setpoint_bed);
						buf_pos += 30;

                        channel_tag heater = heater_lookup_by_name("heater_bed");
                        if (heater) {
		                    int idx = heater_index_lookup(heater);
		                    heater_t *input = &heaters[idx];
							//printf("lkj bed setpoint:%f \n", input->setpoint);
                            if (input && (input->setpoint == 0.0)) {
                                bed_has_setpoint = false;
                            } else {
                                bed_has_setpoint = true;
							}
                        }
					}

					//if ((temp_ext_respond != temp_ext_ok) || ( pa.has_bed && temp_bed_respond == 0)) {
					if ((temp_ext_respond == temp_ext_ok && (temp_ext_ok != 0)) && ( ( pa.has_bed && bed_has_setpoint && (temp_bed_respond == 1)) ||
					          										  ( pa.has_bed && !bed_has_setpoint ) ||
					          										  ( !pa.has_bed )) ) {
						buf[0] = 'o';
						buf[1] = 'k';
						printf("temp_ext_respond ok\n");
					} else {
						//printf("temp_ext_respond not ready \n");
						;
					}
					/*else {
						buf[0] = 'o';
						buf[1] = 'k';
					}*/
					buf[buf_pos] = '\n';

        			if (DBG(D_HEATER)) {
						for (i = 0; i < 99; i++) {
							printf("%c ", buf[i]);	
						}
						printf("temp_ext_ok:0x%x, pa.ext_count:%d,heat:%s\n", temp_ext_ok, pa.ext_count, buf);
					}

					gcode_send_byte_response_remote((char *)&buf, buf_pos+1);
                }
            }
        }
    }

out:
    thread_quit = false;
    printf("Leaving heater thread!\n");
    pthread_exit(NULL);
}

int heater_config(heater_config_t *pcfgs, int nr_cfgs)
{
    int i;

	HEATER_DBG("heater_config called with %d records\n", nr_cfgs);

    if (!pcfgs || nr_cfgs <= 0) {
        return -1; 
    }
    
    heaters = calloc(nr_cfgs, sizeof(heater_t));
    if (!heaters) {
        return -1;
    }

    heaters_bak = calloc(nr_cfgs, sizeof(heater_t));
    if (!heaters_bak) {
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

	memcpy(heaters_bak, heaters, nr_cfgs * sizeof(heater_t));

    return 0;
}

/*"1","THERMISTOR_1",	"2","THERMISTOR_2",	"3","Thermocouple_MAX6675",	"4","Thermocouple_AD597", 5,... ????? */
static char *normal_input_measure [] = {"", "temp_ext","temp_ext2", "temp_ext3", "temp_ext4",  "temp_ext5", "temp_ext6"};
void heater_reconfig() 
{
	heaters[0].input = normal_input_measure[pa.measure_ext1];
	COMM_DBG("heaters[0].input=%s\n", heaters[0].input);

	heaters[1].input = normal_input_measure[pa.measure_ext2];
	COMM_DBG("heaters[1].input=%s\n", heaters[1].input);

	heaters[2].input = normal_input_measure[pa.measure_ext3];
	COMM_DBG("heaters[2].input=%s\n", heaters[2].input);
}

int heater_init(void)
{
    int ret = 0;
    pthread_attr_t attr;
    //struct sched_param sched;

	HEATER_DBG("heater_init called.\n");

    if (!heaters || nr_heaters <= 0) {
        ret = -1;
    }

    /* Initial the pthread attribute */
    if (pthread_attr_init(&attr)) {
        return -1;
    }

	#if 0 //for android
    /* Force the thread to use custom scheduling attributes  */
    if (pthread_attr_setschedpolicy(&attr, PTHREAD_EXPLICIT_SCHED)) {
        return -1;
    }
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("sched_priority max: %d, min %d\n", 
            sched.sched_priority,
            sched_get_priority_min(SCHED_FIFO));
    if (pthread_attr_setschedparam(&attr, &sched)) {
        return -1;
    }
	#endif

    printf("--- Creating heater_thread..."); 
    ret = pthread_create(&heater_thread, &attr, heater_thread_worker, NULL);
    if (ret) {
        printf("create heater thread failed with ret %d\n", ret);
        return -1;
    } else {
        printf("done ---\n");
    }

    return ret;
}

void heater_exit(void)
{
	HEATER_DBG("heater_exit called.\n");
    if (heaters) {
        free(heaters);
    }

    if (!thread_quit) {
        thread_quit = true;
    }

    printf("waiting heater to quit\n");
    pthread_join(heater_thread, NULL); 
    printf("ok\n");
}

int heater_start(void)
{
    if (thread_quit) {
        thread_quit = false;
    }

    return 0;
}

void heater_stop(void)
{
    int i;

    for (i = 0; i < nr_heaters; i++) {
        heater_t *pd = &heaters[i];
		pd->setpoint = 0.0;
        pd->history_idx  = 0;
        pd->pid_integral = 0.0;
        pd->log_fd = -1;

        if (pwm_get_state(pd->output) == PWM_STATE_ON) {
            pwm_set_output(pd->output, 0);
            pwm_disable(pd->output);
        }
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
 *  turn on heater
 */
void heater_enable(channel_tag heater)
{
    int idx = heater_index_lookup(heater);
    if (idx >= 0) {
        pwm_enable(heaters[idx].output);
    }
}
/*
 *  turn off heater
 */
void heater_disable(channel_tag heater)
{
    int idx = heater_index_lookup(heater);
    if (idx >= 0) {
        pwm_disable(heaters[idx].output);
    }
}
/*
 *  set heater pwm output for given channel to value
 */
int heater_set_raw_pwm(channel_tag heater, int percentage)
{
    int ret = -1;
    if (heater) {
        int idx = heater_index_lookup(heater);
        if (percentage >= 0 && percentage <= 99) {
            pwm_set_output(heaters[idx].output, percentage);
            ret = 0;
        }
    }
    return ret;
}
/*
 *  set setpoint for a heater
 */
int heater_set_setpoint(channel_tag heater, double setpoint)
{
    int idx = heater_index_lookup(heater);
    if (idx < 0) {
        printf("[heater]: look up %s err\n", heater);
        return -1;
    }
    
	HEATER_DBG("heater_set_setpoint %f\n", setpoint);

    heater_t *p = &heaters[idx]; 
    
    pthread_mutex_lock(&lock);
    p->setpoint = setpoint;
    pthread_mutex_unlock(&lock);

    if (setpoint == 0.0) {
        if (p->log_fd >= 0) {
            PID_DBG("heater_set_point - logfile closed.\n");
            close(p->log_fd);
        }
        p->log_fd = -1;
    } else {
        if (DBG(D_PID)) {
            if (p->log_fd == -1) {
                p->log_fd = log_file_open(tag_name(p->id));
            }
        }
    }

    temp_set_setpoint(p->input, setpoint, -4.5, 4.5);
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

    pthread_mutex_lock(&lock);
    *setpoint = heaters[idx].setpoint;
    pthread_mutex_unlock(&lock);
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
    
    pthread_mutex_lock(&lock);
    heaters[idx].pid = *pid;
    pthread_mutex_unlock(&lock);

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
    
    pthread_mutex_lock(&lock);
    *pid = heaters[idx].pid;
    pthread_mutex_unlock(&lock);

    return 0;
}
