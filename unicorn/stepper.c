/*
 * Unicorn 3D Printer Firmware
 * stepper.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include <stepper_spi.h>

#include "analog.h"
#include "parameter.h"
#include "planner.h"
#include "unicorn.h"
#include "stepper.h"
#include "stepper_pruss.h"

#include "lmsw.h"
#include "common.h"


typedef struct {
    channel_tag   id;
    channel_tag   vref;

    stepper_cmd_t cmd;
    uint32_t      current;    /* mA */
} stepper_t;

typedef struct {
    int  (*init)(void);
    void (*exit)(void);

    int  (*queue_move)(block_t *block);
    int  (*queue_is_full)(void);
    int  (*queue_wait)(void);

    int  (*queue_get_len)(void);
    int  (*queue_get_max_rate)(void);

    int  (*send_cmd)(st_cmd_t *cmd);
} stepper_ops_t;

static stepper_t *steppers = NULL;
static int nr_steppers = 0;

static stepper_ops_t *stepper_ops = NULL;

static int st_dev_fd;

static pthread_t stepper_thread;
static bool thread_quit = false;
static bool paused = false;
static bool exit_waiting = false;

static inline bool check_step_valid(uint8_t steps)
{
    int i;
    uint8_t valid_steps[] = {1, 2, 4, 8, 16, 32};
    
    for (i = 0; i < sizeof(valid_steps); i++) {
        if (steps == valid_steps[i]) {
            return true;
        }
    }

    return false;
}
/*
 * Set microstep mode
 */
int stepper_set_microstep(uint8_t axis, uint8_t steps)
{
    uint8_t mode; 
    stepper_t *st = NULL;
    
    if (!check_axis_valid(axis)) {
        return -1;
    }
    
    if (!check_step_valid(steps)) {
        return -1;
    }

    switch (steps) {
    case 1:  
        mode = STEP_MODE_1;  
        break;
    case 2:  
        mode = STEP_MODE_2;  
        break;
    case 4:  
        mode = STEP_MODE_4;  
        break;
    case 8:  
        mode = STEP_MODE_8;  
        break;
    case 16: 
        mode = STEP_MODE_16; 
        break;
    case 32: 
        mode = STEP_MODE_32; 
        break;
    default: 
        break;
    }

    st = &steppers[axis];
    st->cmd.mode = mode; 

    return 0;
}
/*
 * Get microstep mode
 */
int stepper_get_microstep(uint8_t axis, uint8_t *steps)
{
    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }
    
    st = &steppers[axis];
    
    switch (st->cmd.mode) {
        case STEP_MODE_1:  
            *steps = 1;  
            break;
        case STEP_MODE_2:  
            *steps = 2;  
            break;
        case STEP_MODE_4:  
            *steps = 4;  
            break;
        case STEP_MODE_8:  
            *steps = 8;  
            break;
        case STEP_MODE_16: 
            *steps = 16; 
            break;
        case STEP_MODE_32: 
            *steps = 32; 
            break;
        default: 
            break;
    }
    return 0;
}
/*
 * Set stepper current
 */
int stepper_set_current(uint8_t axis, uint32_t current_mA)
{
#define ISENSE_RESISTOR (0.1)
    float vol;

    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->current = current_mA;

    /* change current_mA to dac value mV */
    vol = current_mA * 1000 * 5 * ISENSE_RESISTOR;  
    
    //fprintf(stderr, "set to %duV\n", (int)vol);

    /* set analog out channel */
    analog_set_output(st->vref, (int)vol);

    return 0;
}
/*
 * Enable stepper
 */
int stepper_enable(uint8_t axis)
{
    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->cmd.enable = 0;

    return 0;
}
/*
 * Disable stepper
 */
int stepper_disable(uint8_t axis)
{
    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }
    
    st = &steppers[axis];
    st->cmd.enable = 1;
    
    return 0;
}
/*
 * Reset Stepper
 */
int stepper_reset(uint8_t axis)
{
    stepper_t *st = NULL;
    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->cmd.reset = 0;
    return 0;
}

int stepper_disable_reset(uint8_t axis)
{
    stepper_t *st = NULL;
    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->cmd.reset = 1;
    return 0;
}
/*
 * The stepper subsystem goes to sleep when it runs out of things to execute.
 * Call this to notify the subsystem that it is time to go to work.
 */
int stepper_wake_up(uint8_t axis)
{
    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->cmd.sleep = 1;
    return 0;
}
/*
 * Set stepper motor to sleep
 */
int stepper_sleep(uint8_t axis)
{
    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->cmd.sleep = 0;
    return 0;
}
/*
 * update stepper spi command
 */
int stepper_update(void)
{
    int i; 
    stepper_t *st = NULL;

    for (i = 0; i < nr_steppers; i++) {
        st = &steppers[nr_steppers - i - 1];    

#if 0
        fprintf(stderr, "cmd->mode   %#x\n", st->cmd.mode);
        fprintf(stderr, "cmd->enable %#x\n", st->cmd.enable);
        fprintf(stderr, "cmd->reset  %#x\n", st->cmd.reset);
        fprintf(stderr, "cmd->sleep  %#x\n", st->cmd.sleep);
        fprintf(stderr, "cmd->decay  %#x\n", st->cmd.decay);
#endif

#ifndef HOST
        if (st_dev_fd < 0) {
            return -1;
        }
        
        if (ioctl(st_dev_fd, STEPPER_SET_CMD, &st->cmd) < 0) {
            perror("ioctl failed");
            return -1;
        }
#endif
    }
    return 0;
}
/*
 * Start stepper to print
 */
int stepper_start(void)
{
    st_cmd_t cmd = {
        .ctrl.cmd = ST_CMD_START,
    };
    printf("Truby: stepper_start start send cmd\n");
    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }
    printf("Truby: stepper_start start send cmd ok\n");

    return 0;
}
/*
 * Stop stepper from printing
 */
int stepper_stop(void)
{
    st_cmd_t cmd = {
        .ctrl.cmd = ST_CMD_STOP,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }

    return 0;
}
/*
 * Pause stepper
 */
int stepper_pause(void)
{
    st_cmd_t cmd = {
        .ctrl.cmd = ST_CMD_PAUSE,
    };

    if (!paused) { 
        paused = true;
    }

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }

    return 0;
}
/*
 * Resume stepper
 */
int stepper_resume(void)
{
    st_cmd_t cmd = {
        .ctrl.cmd = ST_CMD_RESUME,
    };
    
    if (paused) {
        paused = false;
    }

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }

    return 0;
}
/*
 * homing one axis to lmsw
 */
int stepper_homing_axis(uint8_t axis, uint8_t reverse)
{
    int i;
    uint8_t limited = 0;
    float speed[3];
    uint32_t dir = 0;

    if (!check_axis_valid(axis)) {
        return -1;
    }
    
    /* Check lmsw first */
    switch (axis) {
    case X_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_X, NULL);
        break;

    case Y_AXIS: 
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_Y, NULL);
        break;

    case Z_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_Z, NULL);
        break;

    default:
        break;
    }

    /* if lmsw not hited, send homing cmd */ 
    if (!limited) {
        if (pa.invert_x_dir) {
            dir &= ~(1 << X_AXIS);
        } else {
            dir |=  (1 << X_AXIS);
        }

        if (pa.invert_y_dir) {
            dir &= ~(1 << Y_AXIS);
        } else {
            dir |=  (1 << Y_AXIS);
        }

        if (pa.invert_z_dir) {
            dir &= ~(1 << Z_AXIS);
        } else {
            dir |=  (1 << Z_AXIS);
        }
        
        for (i = 0; i < 3; i++) {
            speed[i] = pa.homing_feedrate[i] * pa.axis_steps_per_unit[i];
        }
        
        st_cmd_t cmd = {
            .homing.cmd   = ST_CMD_AXIS_HOMING,
            .homing.axis  = axis,
            .homing.dir   = dir,
            .homing.speed = fmax(speed[0], fmax(speed[1], speed[2])) / 60.0,
        };

        if (stepper_ops->send_cmd) {
            stepper_ops->send_cmd(&cmd);
        } else {
            return -1;
        }
    }

    return 0;
}
/*
 * Waiting for lmsw
 */
int stepper_wait_for_lmsw(uint8_t axis)
{
    uint8_t limited = 0;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    /* Check lmsw first */
    switch (axis) {
    case X_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_X, NULL);
        break;

    case Y_AXIS: 
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_Y, NULL);
        break;

    case Z_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_Z, NULL);
        break;

    default:
        break;
    }

    if (!limited) {
        wait_lmsw_min(axis);
    }

    return 0;
}
/*
 * Config lmsw: 
 * gpio number and polarity
 */
int stepper_config_lmsw(uint8_t axis)
{
    if (!check_axis_valid(axis)) {
        return -1;
    }

    st_cmd_t cmd = {
        .lmsw.cmd = ST_CMD_LMSW_CONFIG,
        .lmsw.axis = axis,
        .lmsw.min_gpio = 100,
        .lmsw.min_invert = 1,
        .lmsw.max_gpio = 120,
        .lmsw.max_invert = 1,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }

    return 0;
}
/* 
 * Set current position in steps
 */
void stepper_set_position(const long x, const long y, const long z, const long e)
{
    //TODO: Check pos valid according to param

    st_cmd_t cmd = {
        .pos.cmd = ST_CMD_SET_POS,
        .pos.x = x,
        .pos.y = y,
        .pos.z = z,
        .pos.e = e,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    }
}

void stepper_set_e_position(const long e)
{

}

/*
 * Get current position in steps
 */
uint32_t stepper_get_position(uint8_t axis)
{
    uint32_t pos = 0;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    st_cmd_t cmd = {
        .pos.cmd = ST_CMD_GET_POS,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    }

    switch (axis) { 
    case X_AXIS:
        pos = cmd.pos.x;
        break;
    case Y_AXIS:
        pos = cmd.pos.y;
        break;
    case Z_AXIS:
        pos = cmd.pos.z;
        break;
    case E_AXIS:
        pos = cmd.pos.e;
        break;
    default:
        break;
    }

    return pos;
}

/*
 * Get current position in mm
 */
float stepper_get_position_mm(uint8_t axis)
{
    float pos = 0.0;

    pos = stepper_get_position(axis);
    //TODO:
    //step per unit
    
    return pos;
}
/*
 * Block until all buffered steps are executed
 */
void stepper_sync(void)
{
    /* wait all queue movement processed */
    stepper_ops->queue_wait();
}
/*         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 * 
 *  The trapezoid is the shape of the speed curve over time. 
 *  It starts at block->initial_rate, accelerates first block->accelerate_until step_events_completed,
 *  then keeps going at constant speed until step_events_completed reaches block->decelerate_after 
 *  after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated with the leib ramp alghorithm.
 */
/* 
 * Stepper thread
 * It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
 */
void *stepper_thread_worker(void *arg)
{
    int i;
#ifdef SLOWDOWN
    int len;
#endif
    int max_rate;
    int current = 450;
    int last_current = 400;
    bool started = false;
    int block_size = 0;

    /* A pointer to the block currently being traced */
    block_t *current_block = NULL; 

    /* Setup stepper motor driver */
    stepper_set_microstep(X_AXIS, 16);
    stepper_set_microstep(Y_AXIS, 16);
    stepper_set_microstep(Z_AXIS, 16);
    stepper_set_microstep(E_AXIS, 16);
    stepper_set_microstep(E2_AXIS, 16);

    stepper_wake_up(X_AXIS);
    stepper_wake_up(Y_AXIS);
    stepper_wake_up(Z_AXIS);
    stepper_wake_up(E_AXIS);
    stepper_wake_up(E2_AXIS);
    
    stepper_disable_reset(X_AXIS);
    stepper_disable_reset(Y_AXIS);
    stepper_disable_reset(Z_AXIS);
    stepper_disable_reset(E_AXIS);
    stepper_disable_reset(E2_AXIS);

    stepper_enable(X_AXIS);
    stepper_enable(Y_AXIS);
    stepper_enable(Z_AXIS);
    stepper_enable(E_AXIS);
    stepper_enable(E2_AXIS);
    
    /* Send spi command */
    stepper_update(); 

    while (!thread_quit || exit_waiting) 
    {
        if (!paused) {
            if (exit_waiting) {
                block_size = plan_get_block_size();
                if (block_size > 0) {
                    printf("remain block size %d\n", block_size);
                    /* Queue all filled block */
                    for (i = 0; i < block_size; i++) {
                        current_block = plan_get_current_block();
                        if (current_block != NULL) {
                            if (stepper_ops->queue_move) {
                                stepper_ops->queue_move(current_block);
                            }

                            current_block = NULL;
                            plan_discard_current_block();
                        }
                    }

                    break;
                }
            }
    
            current_block = plan_get_current_block();
            if (current_block != NULL) {

                if (!started) {
                    stepper_start(); 
                    started = true;
                }
                
#ifdef SLOWDOWN
                /* Get queue filled len */
                if (stepper_ops->queue_get_len) {
                    len = stepper_ops->queue_get_len();
                }

                /* Slow down when de buffer starts to empty, 
                 * rather than wait at the corner for a buffer refill 
                 */
                if ((len > 1) && len < QUEUE_LEN / 2) {
                    printf("Slow down...\n");
                    current_block->initial_rate *= 0.5;
                    current_block->nominal_rate *= 0.5;
                    current_block->final_rate *= 0.5;
                }
                if (DBG(D_STEPPER)) {
                    block_size = plan_get_block_size();
                    printf("block %d, queue len %d\n", block_size, len);
                }
                
#endif

                /* get max speed rate */
                if (stepper_ops->queue_get_max_rate) {
                    max_rate = stepper_ops->queue_get_max_rate();
                    //printf("max_rate %d\n", max_rate);
                }
#ifdef COREXY 
                /* Adjust current */
                if (max_rate < 1024) {
                    current = 400; 
                }
                if (max_rate >= 1024 && max_rate < 4096) {
                    current = 500;
                }
                if (max_rate >= 4096 && max_rate < 8192) {
                    current = 600;
                }
                if (max_rate >= 8192 && max_rate < 10240) {
                    current = 800;
                }
                if (max_rate >= 10240 && max_rate < 14336) {
                    current = 900;
                }
                if (max_rate >= 14336 && max_rate < 20480 ) {
                    current = 1000;
                }
                if (max_rate >= 20480) { 
                    current = 1200;
                }
#else
                /* Adjust current */
                if (max_rate < 1024) {
                    current = 300; 
                }
                if (max_rate >= 1024 && max_rate < 4096) {
                    current = 400;
                }
                if (max_rate >= 4096 && max_rate < 8192) {
                    current = 500;
                }
                if (max_rate >= 8192 && max_rate < 10240) {
                    current = 700;
                }
                if (max_rate >= 10240 && max_rate < 14336) {
                    current = 900;
                }
                if (max_rate >= 14336 && max_rate < 20480 ) {
                    current = 1000;
                }
                if (max_rate >= 20480) { 
                    current = 1200;
                }
#endif
                if (current != last_current) {
                    stepper_set_current(X_AXIS, current); 
                    stepper_set_current(Y_AXIS, current); 
                    last_current = current;
                }

                if (stepper_ops->queue_move) {
                    stepper_ops->queue_move(current_block);
                }

                current_block = NULL;
                plan_discard_current_block();

            } else {
                usleep(1000);
            }

        } else {
            usleep(1000);
        }
    }
    
    if (exit_waiting) {
        stepper_sync();
    }

    fprintf(stderr, "stepper thread quit!\n");
    pthread_exit(NULL);
}
/*
 * stepper subsystem config
 */
int stepper_config(stepper_config_t *pcfgs, int nr_cfgs)
{
    int i;

    fprintf(stderr, "stepper_config calstepper with %d records\n", nr_cfgs);
    steppers = calloc(nr_cfgs, sizeof(stepper_t));
    if (!steppers) {
        return -1;
    }
    nr_steppers = 0;

    stepper_ops = calloc(1, sizeof(stepper_ops_t));
    if (!stepper_ops) {
        return -1;
    }

    for (i = 0; i < nr_cfgs; i++) {
        stepper_t *pd        = &steppers[i];
        stepper_config_t *ps = &pcfgs[i];
        
        pd->id   = ps->tag;  
        pd->vref = analog_lookup_by_name(ps->vref);
        if (!pd->vref) {
            printf("vref not found %s\n", ps->vref);
            return -1;
        }
        nr_steppers++;
    }

    /* Register queue ops */
    stepper_ops->init          = pruss_stepper_init,
    stepper_ops->exit          = pruss_stepper_exit,
    stepper_ops->queue_move    = pruss_queue_move;
    stepper_ops->queue_is_full = pruss_queue_is_full;
    stepper_ops->queue_wait    = pruss_queue_wait;
    stepper_ops->queue_get_len = pruss_queue_get_len;
    stepper_ops->send_cmd      = pruss_send_cmd;
    stepper_ops->queue_get_max_rate = pruss_queue_get_max_rate;

    return 0;
}
/*
 *  Initialize and start the stepper motor subsystem
 */
int stepper_init(void)
{
    int ret;
    pthread_attr_t attr;
    struct sched_param sched;

    if (DBG(D_STEPPER)) {
        fprintf(stderr, "stepper_init called.\n");
    }
    
    if (!steppers || nr_steppers <= 0) {
        return -1;
    }
    
    /* check queue ops */
    if (!stepper_ops 
        || !stepper_ops->init
        || !stepper_ops->exit
        || !stepper_ops->queue_move 
        || !stepper_ops->queue_is_full 
        || !stepper_ops->queue_wait
        || !stepper_ops->send_cmd
        || !stepper_ops->queue_get_len
        || !stepper_ops->queue_get_max_rate) {
        return -1;
    }

    fprintf(stderr, "[Truby]: init pruss\n");
    stepper_ops->init();
    fprintf(stderr, "[Truby]: init ok\n");

#ifndef HOST
    /* open stepper_spi device */
    st_dev_fd = open(STEPPER_SPI_DEV, O_RDONLY);;    
    if (st_dev_fd < 0) {
        perror("Can not open stepper_spi device");
        return -1;
    }
#endif
    
    fprintf(stderr, "[Truby]: set current\n");

    stepper_set_current(X_AXIS, 450);  //450
    stepper_set_current(Y_AXIS, 450);  //450
    stepper_set_current(Z_AXIS, 450);  //450
    stepper_set_current(E_AXIS, 450);  //450
    stepper_set_current(E2_AXIS, 450);  //450
   
    fprintf(stderr, "[Truby]: set microstep\n");
    stepper_set_microstep(X_AXIS, 16);
    stepper_set_microstep(Y_AXIS, 16);
    stepper_set_microstep(Z_AXIS, 16);
    stepper_set_microstep(E_AXIS, 16);
    stepper_set_microstep(E2_AXIS, 16);
    
    fprintf(stderr, "[Truby]: stepper update\n");
    stepper_update();

    /* Start heater work thread */
    /* Initial the pthread attribute */
    thread_quit = false;
    if (pthread_attr_init(&attr)) {
        return -1;
    }

    /* Force the thread to use custom scheduling attributes  */
    if (pthread_attr_setschedpolicy(&attr, PTHREAD_EXPLICIT_SCHED)) {
        return -1;
    }
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
    printf("[Truby]: sched_priority max: %d, min %d\n", 
            sched.sched_priority,
            sched_get_priority_min(SCHED_FIFO));
    if (pthread_attr_setschedparam(&attr, &sched)) {
        return -1;
    }

    fprintf(stderr, "--- Creating stepper_thread..."); 
    ret = pthread_create(&stepper_thread, &attr, stepper_thread_worker, NULL);
    if (ret) {
        printf("create stepper thread : failed with ret %d\n", ret);
        return -1;
    } else {
        printf("done ---\n");
    }
    return 0;    
}
/*
 *  Exit and stop the stepper motor subsystem
 */
void stepper_exit(int blocking)
{
    if (DBG(D_STEPPER)) {
        fprintf(stderr, "stepper_exit called.\n");
    }

    if (blocking) {
        exit_waiting = true; 
    } else {
        exit_waiting = false;
    }

    thread_quit = true;
    pthread_join(stepper_thread, NULL);

    if (stepper_ops) {
        stepper_stop();

        /* waiting for lmsw */
        stepper_wait_for_lmsw(Y_AXIS);
        stepper_wait_for_lmsw(X_AXIS);

        stepper_ops->exit();

        free(stepper_ops);
    }

    stepper_disable(X_AXIS);
    stepper_disable(Y_AXIS);
    stepper_disable(Z_AXIS);
    stepper_disable(E_AXIS);
    stepper_disable(E2_AXIS);
    
    stepper_sleep(X_AXIS);
    stepper_sleep(Y_AXIS);
    stepper_sleep(Z_AXIS);
    stepper_sleep(E_AXIS);
    stepper_sleep(E2_AXIS);

    stepper_update(); 

    /* close stepper_spi device */
    if (st_dev_fd) {
        close(st_dev_fd);
    }

    if (steppers) {
        free(steppers);
    }
}

