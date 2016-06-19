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
#include <syslog.h>


#include <stepper_spi.h>

#include "analog.h"
#include "parameter.h"
#include "planner.h"
#include "unicorn.h"
#include "stepper.h"
#include "stepper_pruss.h"

#include "lmsw.h"
#include "common.h"

#include "util/Fifo.h"
#include "util/Pause.h"

typedef struct {
    channel_tag   id;
    channel_tag   vref;

    stepper_cmd_t cmd;
    uint32_t      current;    /* mA */
} stepper_t;

typedef struct {
    int  (*init)(void);
    void (*exit)(void);

    int  (*start)(void);
    void (*stop)(void);

    int  (*queue_move)(block_t *block);
    int  (*queue_is_full)(void);
    int  (*queue_wait)(void);
    void  (*queue_terminate_wait)(int);

    void (*queue_parameter_update)(void);

    int  (*queue_get_len)(void);
    int  (*queue_get_max_rate)(void);

    int  (*send_cmd)(st_cmd_t *cmd);
} stepper_ops_t;

static stepper_t *steppers = NULL;
static int nr_steppers = 0;

static stepper_ops_t *stepper_ops = NULL;

static int st_dev_fd;

static pthread_t stepper_thread;
static bool stop = false;
static bool exit_waiting = false;
static bool paused = false;
static bool started = false;
static bool motor_enable = false;

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

	if(axis >= nr_steppers){
		printf("error, axis is out of range\n");
		return -1;
	}

    st = &steppers[axis];
    st->current = current_mA;

    /* change current_mA to dac value mV */
    vol = current_mA * 1000 * 5 * ISENSE_RESISTOR;  
    
    STEPPER_DBG("set vref to %duV\n", (int)vol);

    usleep(10000); //for spi delay 
    /* set analog out channel */
    analog_set_output(st->vref, (int)vol);

    return 0;
}
/*
 * Set stepper decay mode
 */
#define FAST_DECAY    1
#define SLOW_DECAY    0
int stepper_set_decay(uint8_t axis, uint8_t mode)
{
    stepper_t *st = NULL;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    st = &steppers[axis];
    st->cmd.decay = mode;

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

        if (st_dev_fd < 0) {
            return -1;
        }
        
        if (ioctl(st_dev_fd, STEPPER_SET_CMD, &st->cmd) < 0) {
            perror("ioctl failed");
            return -1;
        }
    }
    return 0;
}
/* 
 * Enable stepper drivers
 */
void stepper_enable_drivers(void)
{
    /* Setup stepper motor driver */
    stepper_wake_up(X_AXIS);
    stepper_wake_up(Y_AXIS);
    stepper_wake_up(Z_AXIS);
    stepper_wake_up(E_AXIS);
    stepper_wake_up(E2_AXIS);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_wake_up(E3_AXIS);
        stepper_wake_up(U_AXIS);
        stepper_wake_up(U2_AXIS);
    } 

    stepper_disable_reset(X_AXIS);
    stepper_disable_reset(Y_AXIS);
    stepper_disable_reset(Z_AXIS);
    stepper_disable_reset(E_AXIS);
    stepper_disable_reset(E2_AXIS);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_disable_reset(E3_AXIS);
        stepper_disable_reset(U_AXIS);
        stepper_disable_reset(U2_AXIS);
    } 

    stepper_enable(X_AXIS);
    stepper_enable(Y_AXIS);
    stepper_enable(Z_AXIS);
    stepper_enable(E_AXIS);
    stepper_enable(E2_AXIS);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_enable(E3_AXIS);
        stepper_enable(U_AXIS);
        stepper_enable(U2_AXIS);
    } 

    /* Send spi command */
    stepper_update(); 
	motor_enable = true;
}
/* 
 * Disable stepper drivers
 */
void stepper_disable_drivers(void)
{
    /* Disable Steppers */
    stepper_disable(X_AXIS);
    stepper_disable(Y_AXIS);
    stepper_disable(Z_AXIS);
    stepper_disable(E_AXIS);
    stepper_disable(E2_AXIS);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_disable(E3_AXIS);
        stepper_disable(U_AXIS);
        stepper_disable(U2_AXIS);
    }

    stepper_reset(X_AXIS);
    stepper_reset(Y_AXIS);
    stepper_reset(Z_AXIS);
    stepper_reset(E_AXIS);
    stepper_reset(E2_AXIS);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_reset(E3_AXIS);
        stepper_reset(U_AXIS);
        stepper_reset(U2_AXIS);
    } 

    stepper_sleep(X_AXIS);
    stepper_sleep(Y_AXIS);
    stepper_sleep(Z_AXIS);
    stepper_sleep(E_AXIS);
    stepper_sleep(E2_AXIS);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_sleep(E3_AXIS);
        stepper_sleep(U_AXIS);
        stepper_sleep(U2_AXIS);
    }

    stepper_update(); 
	motor_enable = false;
}
/*
 * Homing one axis to lmsw
 */
int stepper_homing_axis(uint8_t axis)
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

    /* If lmsw not hited, send homing cmd */ 
    if (!limited) {
		dir = get_homing_dir();
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

int get_homing_dir()
{
    uint32_t dir = 0;
	if (pa.machine_type == MACHINE_COREXY) {
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
		if (pa.invert_e_dir) {
			dir &= ~(1 << E_AXIS);
		} else {
			dir |=  (1 << E_AXIS);
		}
	} else if (pa.machine_type == MACHINE_DELTA) {
		/* delta */
		if (pa.invert_x_dir) {
			dir |=  (1 << X_AXIS);
		} else {
			dir &= ~(1 << X_AXIS);
		}

		if (pa.invert_y_dir) {
			dir |=  (1 << Y_AXIS);
		} else {
			dir &= ~(1 << Y_AXIS);
		}

		if (pa.invert_z_dir) {
			dir |=  (1 << Z_AXIS);
		} else {
			dir &= ~(1 << Z_AXIS);
		}
		if (pa.invert_e_dir) {
			dir &= ~(1 << E_AXIS);
		} else {
			dir |=  (1 << E_AXIS);
		}
	} else {
		/* xyz */
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
		if (pa.invert_e_dir) {
			dir &= ~(1 << E_AXIS);
		} else {
			dir |=  (1 << E_AXIS);
		}
	}
	return dir;
}

/*
 * Check limit switch state
 */
int stepper_check_lmsw(uint8_t axis)
{
    int limited = 0;

    if (!check_axis_valid(axis)) {
        return -1;
    }

    /* Check lmsw first */
    switch (axis) {
    case X_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_X, NULL);
        limited = pa.x_endstop_invert ? !limited : limited;
        break;

    case Y_AXIS: 
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_Y, NULL);
        limited = pa.y_endstop_invert ? !limited : limited;
        break;

    case Z_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MIN_Z, NULL);
        limited = pa.z_endstop_invert ? !limited : limited;
        break;

    case MAX_X_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MAX_X, NULL);
        break;

    case MAX_Y_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MAX_Y, NULL);
        break;

    case MAX_Z_AXIS:
        limited = ioctl(st_dev_fd, STEPPER_CH_MAX_Z, NULL);
        break;

    case AUTOLEVEL_Z_AXIS:
		if (bbp_board_type == BOARD_BBP1S) {
        	limited = ioctl(st_dev_fd, AUTOLEVEL_Z, NULL);
            limited = pa.autolevel_endstop_invert ? !limited : limited;
		} else 
	       	limited = 0;
        break;

    default:
        break;
    }

    return limited;
}

/*
 *  1 -> wait ok
 *  0 -> wait failed
 * -1 -> fault
 */
int stepper_wait_for_autoLevel()
{
    int limited = 0;
	int count = 40;

	limited = stepper_check_lmsw(AUTOLEVEL_Z_AXIS);
	if (limited) {
		STEPPER_DBG("stepper_wait_for_autoLevel, limited=%d\n", limited);
    	return limited;
	}

	while ( count && !limited ){
		STEPPER_DBG("stepper_wait_for_autoLevel, count=%d\n",  count);
		limited = stepper_check_lmsw(AUTOLEVEL_Z_AXIS);
		count--;
		sleep(1);
	}

	STEPPER_DBG("stepper_wait_for_autoLevel, limited=%d\n", limited);
    return limited;
}

int stepper_autoLevel_gpio_turn(bool on)
{
	int ret = 0;
	if (bbp_board_type == BOARD_BBP1S) {
		if (on) {
			COMM_DBG("gpio turn on\n");
            stepper_config_lmsw(AUTOLEVEL_Z_AXIS, pa.autolevel_endstop_invert);
			ret = ioctl(st_dev_fd, AUTOLEVEL_Z_GPIO_INPUT, NULL);
		} else {
			COMM_DBG("gpio turn off\n");
            if (pa.autolevel_endstop_invert) {
                set_gpio(14, 1, 0); //14 for autolevel gpio, output low level
            } else {
                set_gpio(14, 1, 1);
            }
			//ret = ioctl(st_dev_fd, AUTOLEVEL_Z_GPIO_OUTPUT, NULL);
		}
	} else 
		ret  = 0;

	return ret;
}

/*
 * Waiting for lmsw
 *  1 -> wait ok
 *  0 -> wait failed
 * -1 -> fault
 */
int stepper_wait_for_lmsw(uint8_t axis)
{
    int limited = 0;
	int count = 10;

	if (!check_axis_valid(axis)) {
		return -1;
	}

	limited = stepper_check_lmsw(axis);
	if (limited) {
		STEPPER_DBG("stepper_wait_for_lmsw axis=%d, limited=%d\n", axis, limited);
    	return limited;
	}

	while ( count && !limited ){
		STEPPER_DBG("stepper_wait_for_lmsw axis=%d, count=%d\n", axis, count);
		wait_lmsw_min(axis);
		limited = stepper_check_lmsw(axis);
		if(limited == -1)
			break;
		count--;
	}

	STEPPER_DBG("stepper_wait_for_lmsw axis=%d, limited=%d\n", axis, limited);
    return limited;
}
/*
 * Config lmsw: 
 * gpio number and polarity
 */
int stepper_config_lmsw(uint8_t axis, bool high) //default active low 
{
    if (!check_axis_valid(axis)) {
        return -1;
    }

    stepper_cmd_lmsw_dir_t cmd_driver;
    st_cmd_t cmd = {
        .lmsw.cmd = ST_CMD_LMSW_CONFIG,
        .lmsw.axis = axis,
        .lmsw.min_gpio = 100,
        .lmsw.min_invert = high,
        .lmsw.max_gpio = 120,
        .lmsw.max_invert = 1,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }

    cmd_driver.axis= axis;
    cmd_driver.high = high;
    if (ioctl(st_dev_fd, STEPPER_SET_LMSW_DIRECTION, &cmd_driver) < 0) {
        perror("ioctl lmsw_direction failed");
        return -1;
    }

    return 0;
}

/* 
 * Set current position in steps
 */
void stepper_set_position(const long x, const long y, const long z, const long e)
{
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
 * Set pause position in steps
 * because delta homing position is different from others
 */
void stepper_set_pause_position(const long x, const long y, const long z)
{
    st_cmd_t cmd = {
        .pos.cmd = ST_CMD_SET_PAUSE_POS,
        .pos.x = x,
        .pos.y = y,
        .pos.z = z,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    }
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
    int32_t pos = 0;
    float pos_mm = 0.0;

    pos = stepper_get_position(axis);
    if (pos < 0) {
       // printf("error, pos:%d <0 \n", pos);
       // return -1;
    }
    if (pos == 0) {
        return 0;
    }

    //if (pa.machine_type == MACHINE_DELTA) {
    {
        //TODO:
//    } else {
        switch (axis) { 
        case X_AXIS:
            pos_mm = pos * 100.0f / pa.axis_steps_per_unit[X_AXIS] / 100.0f;
            break;
        case Y_AXIS:
            pos_mm = pos  * 100.0f/ pa.axis_steps_per_unit[Y_AXIS] / 100.0f;
            break;
        case Z_AXIS:
            pos_mm = pos  * 100.0f/ pa.axis_steps_per_unit[Z_AXIS] / 100.0f;
            break;
        case E_AXIS:
            pos_mm = pos * 100.0f / pa.axis_steps_per_unit[E_AXIS] / 100.0f;
            break;
        default:
            break;
        }
    }
    return pos_mm;
}

static unsigned int last_pru_state = ST_CMD_IDLE;
void stepper_load_filament(int filament_cmd)
{
    st_cmd_t cmd;
    st_cmd_t cmd_get_state;

    cmd_get_state.state.cmd = ST_CMD_GET_PRU_STATE;
    cmd_get_state.state.state = 0;
    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd_get_state);
    }

    int state = cmd_get_state.state.state;
    if((state != STATE_LOAD_FILAMENT) && (state != STATE_UNLOAD_FILAMENT)){
        last_pru_state = state;
        COMM_DBG("set last pru state=%d\n", last_pru_state);
    }

    cmd.state.cmd = ST_CMD_SET_PRU_STATE;
    switch (filament_cmd){
        case 1:
            cmd.state.state = STATE_LOAD_FILAMENT;
            break;
        case 2:
            cmd.state.state = STATE_UNLOAD_FILAMENT;
            break;
        case 3:
            cmd.state.state = last_pru_state;
            break;
        case 4:
          	COMM_DBG("print last pru state:%d, cur pru state:%d\n", last_pru_state, state);
            break;
        default:
            printf("error: unknown filament cmd %d \n",filament_cmd);
            return ;
    }
    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    }
}


/*
 * Get queue filled length
 */
int stepper_get_queue_len(void)
{
    int len = 0;
    if (stepper_ops->queue_get_len) {
        len = stepper_ops->queue_get_len();
    }

    return len;
}
/*
 * Block until all buffered steps are executed
 */
void stepper_sync(void)
{
	//sleep(1); // wait plan to execute for pru .
    usleep(200000);
    STEPPER_DBG("stepper_sync...\n");
    /* wait all queue movement processed */
    stepper_ops->queue_terminate_wait(0);
    stepper_ops->queue_wait();
    STEPPER_DBG("stepper_sync ok!\n");
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

//#define STEPPER_BLOCK_SIZE  (128)
//#define STEPPER_BLOCK_SIZE  (3072)
#define STEPPER_BLOCK_SIZE  (8192)
block_t stepper_blocks[STEPPER_BLOCK_SIZE];

extern Fifo_Handle  hFifo_st2plan;
extern Fifo_Handle  hFifo_plan2st;
extern Pause_Handle hPause_printing;
/*
 * Reset fifo_st2plan
 * Fill all buffer to st2plan fifo
 */
int stepper_reset_fifo(void)
{
    int i;
    block_t *block = NULL;

    /* Put all block buffer available to planner */ 
    for (i = 0; i < STEPPER_BLOCK_SIZE; i++) {
        block = &stepper_blocks[i];
        if (block) {
            if (hFifo_st2plan) {
                Fifo_put(hFifo_st2plan, block);
            }
        }
    }
    return 0;
}
/* 
 * Clear up Fifo 
 */
int stepper_clean_up_fifo(void)
{
    int i = 0;
    int fifo_num = 0;

    block_t *block = NULL;

    fifo_num = Fifo_getNumEntries(hFifo_plan2st);
    STEPPER_DBG("Start to clean up hFifo_plan2st: %d\n", fifo_num);
    if (fifo_num > 0) {
        for (i = 0; i < fifo_num; i++) {
            Fifo_get(hFifo_plan2st, (void **)&block);
        }
    }

    fifo_num = Fifo_getNumEntries(hFifo_st2plan);
    STEPPER_DBG("Start to clean up hFifo_st2plan: %d\n", fifo_num);
    if (fifo_num > 0) {
        for (i = 0; i < fifo_num; i++) {
            Fifo_get(hFifo_st2plan, (void **)&block);
        }
    }

    STEPPER_DBG("Still %d plan2st fifo left\n", Fifo_getNumEntries(hFifo_plan2st));
    STEPPER_DBG("Still %d st2plan fifo left\n", Fifo_getNumEntries(hFifo_st2plan));
    return 0;
}

#ifdef SLOWDOWN
int first_run = 0;
#define SLOWDOWN_LEN 1024
#endif

void *stepper_thread_worker(void *arg)
{
    int i = 0;
    int ret = 0;
    int fifo_num = 0;

    block_t *block = NULL;
    
    float scale = 1.0;
    int len = 0;
    //int block_size;

    //int max_rate;
    //int current_x = 450, current_y = 450,  current_z = 450;
    //int last_current_x = 400, last_current_y = 400, last_current_z = 400;

    /* Thread start up sync */
    if (hPause_printing) {
        Pause_test(hPause_printing);
    }

    STEPPER_DBG("start up stepper thread\n");

    while (!stop || exit_waiting) 
    {
        //TODO: Blocking exit, Get all block from buffer
        
        fifo_num = Fifo_getNumEntries(hFifo_plan2st);
        if (fifo_num > 0) {
            //STEPPER_DBG("stepper fifo_num %d\n", fifo_num);
            for (i = 0; i < fifo_num; i++) {
                /* Get block from planner thread */
                if (hPause_printing) {
                    Pause_test(hPause_printing);
                }
                ret = Fifo_get(hFifo_plan2st, (void **)&block);
                if (ret == 0) { 

                    if (pa.slow_down) {
                        /* Do not slow down at the begin of printing */
                        if (first_run < QUEUE_LEN) {
                            first_run++;
                        }

                        if (first_run >= QUEUE_LEN) {
                            /* Get queue filled len */
                            #if 1
                            if (stepper_ops->queue_get_len) {
                                len = stepper_ops->queue_get_len();
                            }
                            #endif

                            /* Slow down when de buffer starts to empty, 
                             * rather than wait at the corner for a buffer refill 
                             */
							int slowdown_len = pa.slowdown_percent / 100.0f * SLOWDOWN_LEN;
                            if (len <= slowdown_len) {
                                scale = (float)len / ((float)slowdown_len);
                                if (scale < 0.1) {
                                    scale = 0.1;
                                }
                                block->initial_rate *= scale;
                                block->nominal_rate *= scale;
                                block->final_rate *= scale;
                                //STEPPER_DBG("Slow down...%f\n", scale);
                                printf("Slow down...%f\n", scale);
     							syslog(LOG_DEBUG, "[STEPPER]:Slow down...%f\n", scale);   
							#if 0  //lkj for debug slowdown
                            int block_size = plan_get_block_size();
                            STEPPER_DBG("block %d, fifo %d, queue len %d\n", 
                                         block_size, Fifo_getNumEntries(hFifo_plan2st), len);
							#endif
                            }
                        } 
                    }

                    /* queue move to pru */
                    if (!stop) {
                        if (stepper_ops->queue_move) {
                            stepper_ops->queue_move(block);
                        }
                    }

                    /* Put current block to planner */
                    Fifo_put(hFifo_st2plan, block);
                } else {
                    printf("fifo get err\n");
                }

				//lkj slow down if auto_current is true, because queue_get_max_rate waste too much cpu.
		#if 0
                if (pa.auto_current) {
                    /* get max speed rate */
                    if (stepper_ops->queue_get_max_rate) {
                        max_rate = stepper_ops->queue_get_max_rate();
                        if (DBG(D_STEPPER)) {
                            //printf("max_rate %d\n", max_rate);
                        }
                    }
				#if 0
                    /* Adjust current */
                    if (max_rate < 1024) {
                        current = 500; 
                    }
                    if (max_rate >= 1024 && max_rate < 4096) {
                        current = 700;
                    }
                    if (max_rate >= 4096 && max_rate < 8192) {
                        current = 800;
                    }
                    if (max_rate >= 8192 && max_rate < 10240) {
                        current = 900;
                    }
                    if (max_rate >= 10240 && max_rate < 14336) {
                        current = 1100;
                    }
                    if (max_rate >= 14336 && max_rate < 20480 ) {
                        current = 1200;
                    }
                    if (max_rate >= 20480) { 
                        current = 1300;
                    }
			 #else
					if (max_rate < 4096) {
						current_x = pa.axis_current[X_AXIS];
						current_y = pa.axis_current[Y_AXIS];
						current_z = pa.axis_current[Z_AXIS];
					} else if (max_rate >= 4096 && max_rate < 8192) {
						current_x = pa.axis_current[X_AXIS] + 100;
						current_y = pa.axis_current[Y_AXIS] + 100;
						current_z = pa.axis_current[Z_AXIS] + 100;
			        } else if (max_rate >= 8192 && max_rate < 10240) {
						current_x = pa.axis_current[X_AXIS] + 200;
						current_y = pa.axis_current[Y_AXIS] + 200;
						current_z = pa.axis_current[Z_AXIS] + 200;
				    } else if (max_rate >= 10240 && max_rate < 14336) {
						current_x = pa.axis_current[X_AXIS] + 400;
						current_y = pa.axis_current[Y_AXIS] + 400;
						current_z = pa.axis_current[Z_AXIS] + 400;
				    } else if (max_rate >= 14336 && max_rate < 20480) {
						current_x = pa.axis_current[X_AXIS] + 500;
						current_y = pa.axis_current[Y_AXIS] + 500;
						current_z = pa.axis_current[Z_AXIS] + 500;
			        } else {
						current_x = pa.axis_current[X_AXIS] + 600;
						current_y = pa.axis_current[Y_AXIS] + 600;
						current_z = pa.axis_current[Z_AXIS] + 600;
					}
				
					if (current_x > 1200) {
						current_x = 1200;
					}
					if (current_y > 1200) {
						current_y = 1200;
					}
					if (current_z > 1200) {
						current_z = 1200;
					}
				#endif

                    if (current_x != last_current_x) {
                        stepper_set_current(X_AXIS, current_x); 
                        last_current_x = current_x;
					}
                    if (current_y != last_current_y) {
                        stepper_set_current(Y_AXIS, current_y); 
                        last_current_y = current_y;
					}
                    if (current_z != last_current_z) {
						if (pa.machine_type == MACHINE_DELTA) {
                        	stepper_set_current(Z_AXIS, current_z); 
							if (bbp_board_type == BOARD_BBP1S) {
								stepper_set_current(U_AXIS, current_z);
							}
                        	last_current_z = current_z;
						}
					}
                }
	#endif
            }
        } else {
            usleep(10000);
		}
    }

    STEPPER_DBG("Leaving stepper thread!\n");
    //pthread_exit(NULL);
	return NULL;
}
/*
 * stepper subsystem config
 */
int stepper_config(stepper_config_t *pcfgs, int nr_cfgs)
{
    int i;

	STEPPER_DBG("stepper_config calstepper with %d records\n", nr_cfgs);
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

    stepper_ops->start         = pruss_stepper_start,
    stepper_ops->stop          = pruss_stepper_stop,

    stepper_ops->queue_move    = pruss_queue_move;
    stepper_ops->queue_is_full = pruss_queue_is_full;
    stepper_ops->queue_wait    = pruss_queue_wait;
    stepper_ops->queue_terminate_wait = pruss_queue_terminate_wait;
    stepper_ops->queue_get_len = pruss_queue_get_len;
    stepper_ops->queue_get_max_rate = pruss_queue_get_max_rate;
    stepper_ops->queue_parameter_update = pruss_stepper_parameter_update;

    stepper_ops->send_cmd      = pruss_send_cmd;

    return 0;
}
/*
 *  Initialize and start the stepper motor subsystem
 */
int stepper_init(void)
{
    int ret;
    pthread_attr_t attr;
    //struct sched_param sched;

	STEPPER_DBG("stepper_init called.\n");

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

	STEPPER_DBG("[stepper]: init pruss\n");

    if (stepper_ops->init) {
        stepper_ops->init();
    }

	STEPPER_DBG("[stepper]: init ok\n");

    /* open stepper_spi device */
    st_dev_fd = open(STEPPER_SPI_DEV, O_RDONLY);    
    if (st_dev_fd < 0) {
        perror("Can not open stepper_spi device");
        return -1;
    }
    
    //FIXME: The stepper current and micromode should set to
    //       the value in parameter.
	STEPPER_DBG("[stepper]: set current\n");

    stepper_set_current(X_AXIS, pa.axis_current[X_AXIS]);
    stepper_set_current(Y_AXIS, pa.axis_current[Y_AXIS]);
    stepper_set_current(Z_AXIS, pa.axis_current[Z_AXIS]);
    stepper_set_current(E_AXIS, pa.axis_current[E_AXIS]);
    stepper_set_current(E2_AXIS, pa.axis_current[E2_AXIS]);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_set_current(E3_AXIS, pa.axis_current[E3_AXIS]);
        stepper_set_current(U_AXIS,  pa.axis_current[U_AXIS]);
        stepper_set_current(U2_AXIS, pa.axis_current[E2_AXIS]); //FIXME
    }

	STEPPER_DBG("[stepper]: set microstep\n");
    stepper_set_microstep(X_AXIS, pa.axis_ustep[X_AXIS]);
    stepper_set_microstep(Y_AXIS, pa.axis_ustep[Y_AXIS]);
    stepper_set_microstep(Z_AXIS, pa.axis_ustep[Z_AXIS]);
    stepper_set_microstep(E_AXIS, pa.axis_ustep[E_AXIS]);
    stepper_set_microstep(E2_AXIS, pa.axis_ustep[E2_AXIS]);
    if (bbp_board_type == BOARD_BBP1S) {
        stepper_set_microstep(E3_AXIS, pa.axis_ustep[E3_AXIS]);
        stepper_set_microstep(U_AXIS,  pa.axis_ustep[U_AXIS]);
        stepper_set_microstep(U2_AXIS, pa.axis_ustep[E2_AXIS]); //FIXME
    }
    
	STEPPER_DBG("[stepper]: set decay mode\n");

	if (unicorn_get_mode() == FW_MODE_TEST) {
        stepper_set_decay(X_AXIS,  FAST_DECAY);
        stepper_set_decay(Y_AXIS,  FAST_DECAY);
        stepper_set_decay(Z_AXIS,  FAST_DECAY);
        stepper_set_decay(E_AXIS,  FAST_DECAY);
        stepper_set_decay(E2_AXIS, FAST_DECAY);
		if (bbp_board_type == BOARD_BBP1S) {
            stepper_set_decay(E3_AXIS, FAST_DECAY);
            stepper_set_decay(U_AXIS,  FAST_DECAY);
            stepper_set_decay(U2_AXIS, FAST_DECAY);
        }
    } else {
        stepper_set_decay(X_AXIS,  FAST_DECAY);
        stepper_set_decay(Y_AXIS,  FAST_DECAY);
#ifdef DELTA
        stepper_set_decay(Z_AXIS,  FAST_DECAY);
#else
        stepper_set_decay(Z_AXIS,  SLOW_DECAY);
#endif
		if (bbp_board_type == BOARD_BBP1S) {
            stepper_set_decay(E3_AXIS,  SLOW_DECAY);
            stepper_set_decay(U_AXIS,  SLOW_DECAY);
        }
        stepper_set_decay(E_AXIS,  SLOW_DECAY);
        stepper_set_decay(E2_AXIS, SLOW_DECAY);
    }

    stepper_update();

    stop = false;

    /* Initial the pthread attribute */
    if (pthread_attr_init(&attr)) {
        return -1;
    }

    /* Force the thread to use custom scheduling attributes  */
	#if 0 //for android
    if (pthread_attr_setschedpolicy(&attr, PTHREAD_EXPLICIT_SCHED)) {
        return -1;
    }
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);

    if (pthread_attr_setschedparam(&attr, &sched)) {
        return -1;
    }
	#endif
    
    STEPPER_DBG("--- Creating stepper_thread..."); 
    ret = pthread_create(&stepper_thread, &attr, stepper_thread_worker, NULL);
    if (ret) {
        printf("create stepper thread failed with ret %d\n", ret);
        return -1;
    } else {
        STEPPER_DBG("done ---\n");
    }

	if (pa.autoLeveling && (pa.probeDeviceType == PROBE_DEVICE_SERVO || pa.probeDeviceType == PROBE_DEVICE_Z_PIN
						|| pa.probeDeviceType == PROBE_DEVICE_FSR)) {
		stepper_autoLevel_gpio_turn(true);
	} else {
		stepper_autoLevel_gpio_turn(false);
	}

    return 0;    
}

bool stepper_is_stop()
{
	return !started;
}

bool motor_is_disable()
{
	return (motor_enable == false);
}


/*
 *  Exit and stop the stepper motor subsystem
 */
void stepper_exit(int blocking)
{
	STEPPER_DBG("stepper_exit called.\n");

    if (blocking) {
        exit_waiting = true; 
    } else {
        exit_waiting = false;
    }

    if (!stop) {
        stop = true;
    } 
    
    STEPPER_DBG("waiting stepper to quit\n");
    pthread_join(stepper_thread, NULL);
    STEPPER_DBG("ok\n");

	st_cmd_t cmd = {
		.ctrl.cmd = ST_CMD_IDLE,
	};

	if (stepper_ops->send_cmd) {
		stepper_ops->send_cmd(&cmd);
	} 
	/* FIXME: should wait the last queue move to finished */
	usleep(20000);

	if (stepper_ops->stop) {
		stepper_ops->stop();
	}

    if (stepper_ops) {
        stepper_ops->exit();
        free(stepper_ops);
    }

    /* Turn off stepper drivers */
    stepper_disable_drivers();

    /* close stepper_spi device */
    if (st_dev_fd) {
        close(st_dev_fd);
    }

    if (steppers) {
        free(steppers);
    }
}
/*
 * Pause stepper
 */
int stepper_pause(void)
{
    int pos_x;
    int pos_y;
    int pos_z;

    if (!paused) { 
        paused = true;
        
        st_cmd_t cmd = {
            .ctrl.cmd = ST_CMD_PAUSE,
        };

        if (stepper_ops->send_cmd) {
            stepper_ops->send_cmd(&cmd);

        	stepper_wait_for_lmsw(X_AXIS);
        	stepper_wait_for_lmsw(Y_AXIS);

            /* Update pause position */ 
            if (pa.machine_type == MACHINE_DELTA) {

        	    stepper_wait_for_lmsw(Z_AXIS);

                float delta_tower1_x = -SIN_60 * (pa.radius + pa.radius_adj[0]);
                float delta_tower1_y = -COS_60 * (pa.radius + pa.radius_adj[0]);
                float delta_tower2_x =  SIN_60 * (pa.radius + pa.radius_adj[1]);
                float delta_tower2_y =  COS_60 * (pa.radius + pa.radius_adj[1]);
                float delta_tower3_x = 0.0;
                float delta_tower3_y = (pa.radius + pa.radius_adj[2]);

                float delta_diagonal_rod_2_tower1 = pow(pa.diagonal_rod + pa.diagonal_rod_adj[0], 2);
                float delta_diagonal_rod_2_tower2 = pow(pa.diagonal_rod + pa.diagonal_rod_adj[1], 2);
                float delta_diagonal_rod_2_tower3 = pow(pa.diagonal_rod + pa.diagonal_rod_adj[2], 2);

                float delta_x = sqrt(delta_diagonal_rod_2_tower1 
                                    - powf(delta_tower1_x, 2)
                                    - powf(delta_tower1_y, 2)
                                    ) + pa.z_home_pos;

                float delta_y = sqrt(delta_diagonal_rod_2_tower2 
                                    - powf(delta_tower2_x, 2)
                                    - powf(delta_tower2_y, 2)
                                    ) + pa.z_home_pos;

                float delta_z = sqrt(delta_diagonal_rod_2_tower3 
                                    - powf(delta_tower3_x, 2)
                                    - powf(delta_tower3_y, 2)
                                    ) + pa.z_home_pos;


                pos_x = lround(delta_x * pa.axis_steps_per_unit[X_AXIS]);
                pos_y = lround(delta_y * pa.axis_steps_per_unit[X_AXIS]);
                pos_z = lround(delta_z * pa.axis_steps_per_unit[X_AXIS]);
                
                pos_x -= stepper_get_position(X_AXIS);
                pos_y -= stepper_get_position(Y_AXIS);
                pos_z -= stepper_get_position(Z_AXIS);
            } else {
                pos_x = stepper_get_position(X_AXIS);
                pos_y = stepper_get_position(Y_AXIS);
                pos_z = stepper_get_position(Z_AXIS);
            }

            /* Set the pause position, so the PRU know where to resume 
             * COREXY type, calculate resume x,y,z by itself in pru code.
             */
            if (pa.machine_type == MACHINE_XYZ || pa.machine_type == MACHINE_DELTA) {
                stepper_set_pause_position(pos_x, pos_y, pos_z);
            }
        } else {
            return -1;
        }
    } else {
        STEPPER_DBG("Already pasued!\n");
    }
    return 0;
}
/*
 * Resume stepper
 */
int stepper_resume(void)
{
    if (paused) {
        paused = false;

        st_cmd_t cmd = {
            .ctrl.cmd = ST_CMD_RESUME,
        };
        
        if (stepper_ops->send_cmd) {
            stepper_ops->send_cmd(&cmd);
        } else {
            return -1;
        }

    } else {
        STEPPER_DBG("Already resumed!\n");
    }

    return 0;
}

void stepper_parameter_update(void)
{
    if (stepper_ops->queue_parameter_update) {
        stepper_ops->queue_parameter_update();
    }
}

/*
 * Start stepper to print
 */
int stepper_start(void)
{
    if (!started) {
        started = true;
        
        if (stop) {
            stop = false;
        }

#ifdef SLOWDOWN
        first_run = 0;
#endif
        
        STEPPER_DBG("stepper_start...........\n");

        //STEPPER_DBG("Before %d plan2st fifo left\n", Fifo_getNumEntries(hFifo_plan2st));
        //STEPPER_DBG("Before %d st2plan fifo left\n", Fifo_getNumEntries(hFifo_st2plan));
        //STEPPER_DBG("reset_fifo...........\n");

        stepper_reset_fifo();

        //STEPPER_DBG("After %d plan2st fifo left\n", Fifo_getNumEntries(hFifo_plan2st));
        //STEPPER_DBG("After %d st2plan fifo left\n", Fifo_getNumEntries(hFifo_st2plan));

        if (stepper_ops->start) {
            stepper_ops->start();
        }

        st_cmd_t cmd = {
            .ctrl.cmd = ST_CMD_START,
        };

		STEPPER_DBG("stepper_start start send cmd\n");

        if (stepper_ops->send_cmd) {
            stepper_ops->send_cmd(&cmd);
        } else {
            return -1;
        }

		STEPPER_DBG("stepper_start start send cmd ok\n");

        /* Turn on stepper drivers */
        stepper_enable_drivers();

        if (paused) {
            paused = false;
        }
    }

    return 0;
}
/*
 * Stop stepper from printing
 */
int stepper_stop(bool blocking)
{
    if (started) {
        started = false;

        if (blocking) { 
            stepper_sync();
        }

        st_cmd_t cmd = {
            .ctrl.cmd = ST_CMD_STOP,
        };

        if (stepper_ops->send_cmd) {
            stepper_ops->send_cmd(&cmd);
        } else {
            return -1;
        }

        STEPPER_DBG("stepper_stop: waiting for lmsw\n");
        stepper_wait_for_lmsw(X_AXIS);
        stepper_wait_for_lmsw(Y_AXIS);
        if (pa.machine_type == MACHINE_DELTA) {
            stepper_wait_for_lmsw(Z_AXIS);
        }
        STEPPER_DBG("stepper_stop: waiting for lmsw: ok\n");
        
        /* FIXME: should wait the last queue move to finished */
        usleep(20000);

        if (stepper_ops->stop) {
            stepper_ops->stop();
        }

        /* Turn off stepper drivers */
        stepper_disable_drivers();
    }

	if ((!blocking) && stepper_ops->queue_terminate_wait) {
    	stepper_ops->queue_terminate_wait(1);
	}

    return 0;
}

int stepper_idle(void)
{
    if (started) {
        started = false;

        st_cmd_t cmd = {
            .ctrl.cmd = ST_CMD_IDLE,
        };

        if (stepper_ops->send_cmd) {
            stepper_ops->send_cmd(&cmd);
        } else {
            return -1;
        }
        
        /* FIXME: should wait the last queue move to finished */
        usleep(20000);

        if (stepper_ops->stop) {
            stepper_ops->stop();
        }

        /* Turn off stepper drivers */
        stepper_disable_drivers();

        if (stepper_ops->queue_terminate_wait) {
            stepper_ops->queue_terminate_wait(1);
        }
    }
    return 0;
}

/*
 * Test stepper
 */
int stepper_test(void)
{
    int i;
    float speed[3];
    uint32_t dir = 0;

    stepper_wake_up(X_AXIS);
    stepper_wake_up(Y_AXIS);
    stepper_wake_up(Z_AXIS);
    stepper_wake_up(E_AXIS);
    stepper_wake_up(E2_AXIS);
	if (bbp_board_type == BOARD_BBP1S) {
        stepper_wake_up(E3_AXIS);
        stepper_wake_up(U_AXIS);
        stepper_wake_up(U2_AXIS);
    }

    stepper_disable_reset(X_AXIS);
    stepper_disable_reset(Y_AXIS);
    stepper_disable_reset(Z_AXIS);
    stepper_disable_reset(E_AXIS);
    stepper_disable_reset(E2_AXIS);
	if (bbp_board_type == BOARD_BBP1S) {
        stepper_disable_reset(E3_AXIS);
        stepper_disable_reset(U_AXIS);
        stepper_disable_reset(U2_AXIS);
    }

    stepper_enable(X_AXIS);
    stepper_enable(Y_AXIS);
    stepper_enable(Z_AXIS);
    stepper_enable(E_AXIS);
    stepper_enable(E2_AXIS);
	if (bbp_board_type == BOARD_BBP1S) {
        stepper_enable(E3_AXIS);
        stepper_enable(U_AXIS);
        stepper_enable(U2_AXIS);
    }

    stepper_update();

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
        .test.cmd   = ST_CMD_TEST,
        //.test.axis  = axis,
        .test.dir   = dir,
        .test.speed = fmax(speed[0], fmax(speed[1], speed[2])) / 60.0,
    };

    if (stepper_ops->send_cmd) {
        stepper_ops->send_cmd(&cmd);
    } else {
        return -1;
    }
    
    return 0;
}
