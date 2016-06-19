/*
 * Unicorn 3D Printer Firmware
 * stepper_pruss.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <dirent.h>
#include <string.h> 
#include <time.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include <stepper_spi.h>

#include "common.h"
#include "parameter.h"
#include "planner.h"
#include "pruss.h"
#include "stepper.h"
#include "stepper_pruss.h"

#if 0
static struct active_extruder_gpio g_active_ext_gpio[MAX_EXTRUDER] = {
       {
        .ext_step_ctl_gpio = 0x4804C13C,
        .ext_step_ctl_offset = 28,
        .ext_step_dir_gpio = 0x481AC13C,     
        .ext_step_dir_offset = 2
        },
       {
        .ext_step_ctl_gpio = 0x4804C13C,
        .ext_step_ctl_offset = 24,
        .ext_step_dir_gpio = 0x481AC13C,     
        .ext_step_dir_offset = 5
        },
       {
        .ext_step_ctl_gpio = 0x4804C13C,
        .ext_step_ctl_offset = 25,
        .ext_step_dir_gpio = 0x481AE13C,     
        .ext_step_dir_offset = 7
        }
};
#endif

struct lmsw {
    uint32_t gpio;
    uint32_t level;
} __attribute__((packed));

struct homing {
    uint32_t dir;
    uint32_t time;
    uint32_t axis;
} __attribute__((packed));

struct position {
    uint32_t x; 
    uint32_t y; 
    uint32_t z; 
    uint32_t e; 
} __attribute__((packed));

/* The communication with the PRU. 
 * We memory map the static RAM in the PRU and write stuff into it from here. 
 */
//static volatile struct queue *pru_queue = NULL;
volatile struct queue *pru_queue = NULL;
static volatile unsigned int queue_pos = 0;

static int mem_fd;
static void *ddr_mem = NULL;


/*
 * pruss queue operation
 */
static volatile struct queue *queue_mmap(void)
{
    int i;

#if defined(SHARE_PRU_MEM)
    void *dram = NULL;
    prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &dram);
    bzero(dram, sizeof(*pru_queue));
    pru_queue = (struct queue *)dram;
#elif defined(SHARE_DATARAM)
    void *shared_mem = NULL;

    ret = prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &shared_mem);
    if (ret < 0) {
        printf("mem map pru shared mem err\n");
        return NULL;
    } else {
        printf("map pru shared mem to %#x\n", (unsigned int)shared_mem);
    }

    bzero(shared_mem, sizeof(*pru_queue));
    pru_queue = (struct queue *)shared_mem;

#else  

#define DDR_BASEADDR     0x80e00000
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return NULL;
    }

    /* map the DDR memory */
    ddr_mem = mmap(0, 0x100000, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DDR_BASEADDR);
    if (ddr_mem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return NULL;
    }

    bzero(ddr_mem, sizeof(*pru_queue));
    pru_queue = (struct queue *)(ddr_mem);
#endif

    for (i = 0; i < QUEUE_LEN; i++) {
        pru_queue->ring_buf[i].state = STATE_EMPTY;
    }
    queue_pos = 0;

    return pru_queue;
}

static volatile struct queue_element *queue_get_next_element(void)
{
    queue_pos %= QUEUE_LEN;

    /* Update queue pos */
    pru_queue->write_pos = queue_pos * sizeof(struct queue_element);

    /* Wait for an available queue element */
    while (pru_queue->ring_buf[queue_pos].state != STATE_EMPTY) {
        prussdrv_pru_wait_event(PRU_EVTOUT_0);
        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
    }

    return &pru_queue->ring_buf[queue_pos++];
}

//test
#if 0
static long counter = 0;
static void queue_dump_element(volatile struct queue_element *e)
{
    if (e->state == STATE_EXIT) {
        fprintf(stderr, "\nqueue[%02td]: EXIT\n", e - pru_queue->ring_buf);
    } else {
        struct queue_element element = *e;
        fprintf(stderr, "queue[%5ld : %3d]: dir:0x%02x->0x%02x s:(%5d + %5d + %5d) = %5d ",
                counter++,
                e - pru_queue->ring_buf, 
                element.direction, 
                element.direction_bits, 
                element.loops_accel,
                element.loops_travel,
                element.loops_decel,
                element.loops_accel + element.loops_travel + element.loops_decel);
#if 1
        if (element.steps_x) {
            printf( "x:%d ", element.steps_x);
        }
        if (element.steps_y) {
            printf("y:%d ", element.steps_y);
        }
        if (element.steps_z) {
            printf("z:%d ", element.steps_z);
        }
        if (element.steps_e) {
            printf("e:%d ", element.steps_e);
        }
#endif
        printf("\n");
#if 0
        fprintf(stderr, "                    ac: %d, dc: %d, ic: %d, tc %d",
                element.accel_cycles,
                element.decel_cycles,
                element.init_cycles,
                element.travel_cycles);

        fprintf(stderr, "\n");
#endif
    }
}
#endif

static void queue_put_element(struct queue_element *element)
{
    uint8_t state_to_send = element->state;
    if (state_to_send == STATE_EMPTY) {
        printf("queue an empty element? %#x\n", element->state);
        return;
    }

    /* Initially, we copy everything with 'STATE_EMPTY', then flip the state
     * to avoid a race condition while copying.
     */
    element->state = STATE_EMPTY;
    volatile struct queue_element *qe = queue_get_next_element();
    *qe = *element;

    /* Fully inited. Tell busy-waiting PRU by flipping the state */
    qe->state = state_to_send;

#if 0
    if (DBG(D_STEPPER)) {
        queue_dump_element(qe);
    }
#endif
}
/*
 * pruss queue movement
 */
#define DELAY_PER_STEP   (2)

int pruss_queue_move(block_t *block)
{
    uint8_t dir = 0;
    struct queue_element qe;
    bzero(&qe, sizeof(struct queue_element)); 
    uint32_t final_cycles;

    if (!block) {
        return -1;
    }

    /* queue type */
    qe.state = STATE_FILLED; 

    /* Direction Setting */ 
    dir = block->direction_bits; 
    qe.direction = block->direction_bits;

    qe.type = block->type;
	if (qe.type == BLOCK_M_CMD) {
    	queue_put_element(&qe);
		return 0;
	}

    if (pa.invert_x_dir) {
        if ((dir & (1 << X_AXIS)) != 0) {
            dir &= ~(1 << X_AXIS);  // X_AXIS -direction
        } else {
            dir |=  (1 << X_AXIS);  // X_AXIS +direction
        }
    }
    if (pa.invert_y_dir) {
        if ((dir & (1 << Y_AXIS)) != 0) {
            dir &= ~(1 << Y_AXIS);  // Y_AXIS -direction
        } else {
            dir |=  (1 << Y_AXIS);  // Y_AXIS +direction
        }
    }
    if (pa.invert_z_dir) {
        if ((dir & (1 << Z_AXIS)) != 0) {
            dir &= ~(1 << Z_AXIS);  // Z_AXIS -direction
        } else {
            dir |=  (1 << Z_AXIS);  // Z_AXIS +direction
        }
    }
    if (pa.invert_e_dir) {
        if ((dir & (1 << E_AXIS)) != 0) {
            dir &= ~(1 << E_AXIS);  // E_AXIS -direction
        } else {
            dir |=  (1 << E_AXIS);  // E_AXIS +direction
        }
    }
    qe.direction_bits = dir;
    
    qe.steps_count = block->step_event_count;
    qe.steps_x = block->steps_x;
    qe.steps_y = block->steps_y;
    qe.steps_z = block->steps_z;
    qe.steps_e = block->steps_e;

    /* Phase Setting */
    qe.loops_accel  = block->accelerate_until;

    if (block->nominal_rate == block->final_rate) {
        qe.loops_travel = block->step_event_count - block->accelerate_until;
        qe.loops_decel  = 0;
    } else {
        qe.loops_travel = block->decelerate_after - block->accelerate_until;
        qe.loops_decel  = block->step_event_count - block->decelerate_after;
    }
 	if (block->step_event_count - block->decelerate_after < 0) {
        qe.loops_travel = block->step_event_count - block->accelerate_until;
       	qe.loops_decel = 0;
	}

    /* Calculate delay */
    qe.init_cycles   = NSEC_PER_SEC / block->initial_rate / DELAY_PER_STEP;
    //qe.final_cycles  = NSEC_PER_SEC / block->final_rate / DELAY_PER_STEP; 
    final_cycles  = NSEC_PER_SEC / block->final_rate / DELAY_PER_STEP; 
    qe.travel_cycles = NSEC_PER_SEC / block->nominal_rate / DELAY_PER_STEP;


    if (qe.loops_accel != 0) {
        qe.accel_cycles = (qe.init_cycles - qe.travel_cycles) / qe.loops_accel;
    } else {
        qe.accel_cycles = 0;
    }
    if (qe.loops_decel != 0) {
        qe.decel_cycles = (final_cycles - qe.travel_cycles) / qe.loops_decel;
    } else {
        qe.decel_cycles = 0;
    }
    
    //fix me add E1 E2 direction when E0 E1 E2 move together.
    //fix me add E1 E2 steps     when E0 E1 E2 move together.
    //ext_step_bit       // bit 0-2 active extruder E0, E1, E2,  3-5 dir e0, e1, e2   
    qe.ext_step_bit = 0;
    if ((block->active_extruder == 0) || (block->active_extruder == 1) || (block->active_extruder == 2)) {
		if (pa.bbp1s_dual_xy_mode == MOTOR56_MODE_EXTRUDER) {
        	qe.ext_step_bit |= 1 << (block->active_extruder); //fix me, more than one active extruder
		} else if (pa.bbp1s_dual_xy_mode == MOTOR56_MODE_DUAL_X_Y) {
        	qe.ext_step_bit |= 1 << (0); 
		}
    }
    qe.ext_step_bit |= ((qe.direction_bits >> E_AXIS) & 0x1) << 3; // dir e0
    qe.ext_step_bit |= ((qe.direction_bits >> E_AXIS) & 0x1) << 4; // dir e1 fix me
    qe.ext_step_bit |= ((qe.direction_bits >> E_AXIS) & 0x1) << 5; // dir e2 fix me

    //qe.ext_step_ctl_gpio   = g_active_ext_gpio[block->active_extruder].ext_step_ctl_gpio;
    //qe.ext_step_dir_gpio   = g_active_ext_gpio[block->active_extruder].ext_step_dir_gpio;
    //qe.ext_step_ctl_offset = g_active_ext_gpio[block->active_extruder].ext_step_ctl_offset;
    //qe.ext_step_dir_offset = g_active_ext_gpio[block->active_extruder].ext_step_dir_offset;
    queue_put_element(&qe);

#if 0    
    if (DBG(D_STEPPER)) {
        printf("steps_count %ld\n", block->step_event_count);
        printf("steps_x %ld\n", block->steps_x);
        printf("steps_y %ld\n", block->steps_y);
        printf("steps_z %ld\n", block->steps_z);
        printf("steps_e %ld\n", block->steps_e);
    }
#endif

#if 0
    STEPPER_DBG("                    entry %ld, nominal %ld, final %ld, acc %ld\n\n", 
                block->initial_rate, block->nominal_rate, block->final_rate, 
                block->acceleration_st);
        //printf("accelerate_until %ld, decelerate_after %ld, step_event_count %ld\n",
        //        block->accelerate_until, block->decelerate_after, block->step_event_count);
#endif
    return 0;
}

int pruss_queue_is_full(void)
{
    queue_pos %= QUEUE_LEN;
    if (pru_queue->ring_buf[queue_pos].state == STATE_EMPTY) {
        return 0;
    } else {
        return -1;
    }
}

static int pruss_queue_is_empty(void)
{
    int i;
    int ret = 1;

    for (i = 0; i < QUEUE_LEN; i++) {
        if (pru_queue->ring_buf[i].state != STATE_EMPTY) {
            ret = 0;
            break;
        }
    }

    return ret;
}

static int exit_queue_wait = 0;
void pruss_queue_terminate_wait(int exit)
{
	exit_queue_wait = exit;
}

int pruss_queue_wait(void)
{
#if 0
    const unsigned int last = (queue_pos - 1) % QUEUE_LEN;

    while (pru_queue->ring_buf[last].state != STATE_EMPTY) {
        usleep(10000);
    }
#else
    while ( !pruss_queue_is_empty() && (exit_queue_wait == 0) ) {
        usleep(100000);
    }
#endif

    return 0;
}

int pruss_queue_get_max_rate(void)
{
    int i;
    int cycles = 0;
    int min_cycles = 40000000;
    int max_rate = 0; 

    for (i = 0; i < QUEUE_LEN; i++) {
        if (pru_queue->ring_buf[i].state != STATE_EMPTY) {
            cycles = pru_queue->ring_buf[i].travel_cycles;
            if (cycles < min_cycles && cycles != 0) {
                min_cycles = cycles;
            }
        }
    }
    max_rate = NSEC_PER_SEC / (min_cycles * DELAY_PER_STEP);

    return max_rate;
}

int pruss_queue_get_len(void)
{
#if 0
    int i; 
    int len = 0; 
    for (i = 0; i < QUEUE_LEN; i++) {
        if (pru_queue->ring_buf[i].state != STATE_EMPTY) {
            len++; 
        }
    }
#endif
    int len = ((pru_queue->write_pos - pru_queue->read_pos_pru) / sizeof(struct queue_element) + QUEUE_LEN) & (QUEUE_LEN - 1);

    return len;
}

int pruss_stepper_start(void)
{
    int i;
    float speed[3];
    float max_speed;

    uint32_t dir = 0;

	dir = get_homing_dir();
    
    for (i = 0; i < 3; i++) {
        speed[i] = pa.homing_feedrate[i] * pa.axis_steps_per_unit[i];
    }
    
    //max_speed = fmax(speed[0], fmax(speed[1], speed[2])) / 60.0;
    max_speed = fmin(speed[0], fmax(speed[1], speed[2])) / 60.0;

    pru_queue->homing_dir  = dir; 
    pru_queue->homing_time = NSEC_PER_SEC / max_speed / DELAY_PER_STEP;
    pru_queue->homing_axis = 0; 

    pru_queue->state = STATE_IDLE;

    pru_queue->pos_x = 0;
    pru_queue->pos_y = 0;
    pru_queue->pos_z = 0;
    pru_queue->pos_e = 0;

    pru_queue->pause_x = 0;
    pru_queue->pause_y = 0;
    pru_queue->pause_z = 0;

    pru_queue->write_pos = 0;
    pru_queue->read_pos_pru = 0;

    /* Clear up queue buffer */
    for (i = 0; i < QUEUE_LEN; i++) {
        pru_queue->ring_buf[i].state = STATE_EMPTY;
    }
    queue_pos = 0;
    
    pru_queue->machine_type = pa.machine_type;
	pru_queue->bbp1_extend_func = pa.bbp1_extend_func;
	pru_queue->motor56_mode = pa.bbp1s_dual_xy_mode;

    pru_queue->invert_endstop = 0;
    pru_queue->invert_endstop = (pa.x_endstop_invert << 0) | (pa.y_endstop_invert << 1) | 
                                (pa.z_endstop_invert << 2) | (pa.autolevel_endstop_invert << 3);
    pru_queue->mcode_count = 0;

    pruss_enable();
    return 0;
}

void pruss_stepper_parameter_update(void)
{
    int i;
    float speed[3];
    float max_speed;

    uint32_t dir = 0;

	dir = get_homing_dir();
    
    for (i = 0; i < 3; i++) {
        speed[i] = pa.homing_feedrate[i] * pa.axis_steps_per_unit[i];
    }
    
    max_speed = fmax(speed[0], fmax(speed[1], speed[2])) / 60.0;

    pru_queue->homing_dir  = dir; 
    pru_queue->homing_time = NSEC_PER_SEC / max_speed / DELAY_PER_STEP;

    pru_queue->machine_type = pa.machine_type;
	pru_queue->bbp1_extend_func = pa.bbp1_extend_func;
	pru_queue->motor56_mode = pa.bbp1s_dual_xy_mode;

    pru_queue->invert_endstop = 0;
    pru_queue->invert_endstop = (pa.x_endstop_invert << 0) | (pa.y_endstop_invert << 1) | 
                                (pa.z_endstop_invert << 2) | (pa.autolevel_endstop_invert << 3);
}

void pruss_stepper_stop(void)
{
    int i;

    pru_queue->pos_x = 0;
    pru_queue->pos_y = 0;
    pru_queue->pos_z = 0;
    pru_queue->pos_e = 0;

    pru_queue->write_pos = 0; 
    pru_queue->read_pos_pru = 0;

    //pru_queue->state = STATE_IDLE;  //FIXME: Bug here!!!
    for (i = 0; i < QUEUE_LEN; i++) {
        pru_queue->ring_buf[i].state = STATE_EMPTY;
    }
    queue_pos = 0;
}

int pruss_send_cmd(st_cmd_t *cmd)
{
    int ret = 0; 
    int tmp = 0; 

    switch (cmd->gen[0]) {
    case ST_CMD_AXIS_HOMING:
		if (pa.machine_type == MACHINE_DELTA) {
        	pru_queue->homing_axis = (1 << X_AXIS) | (1 << Y_AXIS) | (1 << Z_AXIS);
		} else {
        	pru_queue->homing_axis |= 1 << cmd->homing.axis;
		}
        pru_queue->homing_dir  = cmd->homing.dir;
        pru_queue->homing_time = NSEC_PER_SEC / cmd->homing.speed / DELAY_PER_STEP;
        pru_queue->state = STATE_HOME;
        break;

    case ST_CMD_START:
        pru_queue->state = STATE_PRINT;
        break;

    case ST_CMD_STOP:
    	//pru_queue->pause_z_distance_steps = 20 * pa.axis_steps_per_unit[Z_AXIS]; 
		tmp = pa.z_max_length * pa.axis_steps_per_unit[Z_AXIS] - abs(pru_queue->pos_z);
		if (tmp > 10 * pa.axis_steps_per_unit[Z_AXIS]){
    		pru_queue->pause_z_distance_steps = 10.0 * pa.axis_steps_per_unit[Z_AXIS];
		} else {
    		pru_queue->pause_z_distance_steps = tmp/2 * pa.axis_steps_per_unit[Z_AXIS];
		}
        pru_queue->state = STATE_STOP;
        break;

    case ST_CMD_PAUSE:
    	//pru_queue->pause_z_distance_steps = (pa.z_max_length - 50.0) * pa.axis_steps_per_unit[Z_AXIS] - abs(pru_queue->pos_z);
		tmp = pa.z_max_length * pa.axis_steps_per_unit[Z_AXIS] - abs(pru_queue->pos_z);
		if (tmp > 10 * pa.axis_steps_per_unit[Z_AXIS]){
    		pru_queue->pause_z_distance_steps = 10.0 * pa.axis_steps_per_unit[Z_AXIS];
		} else {
    		pru_queue->pause_z_distance_steps = tmp/2 * pa.axis_steps_per_unit[Z_AXIS];
		}
		COMM_DBG("pause z distance:%d\n", pru_queue->pause_z_distance_steps);
        pru_queue->state = STATE_PAUSE; 
        break;

    case ST_CMD_RESUME:
        pru_queue->state = STATE_RESUME;
        break;

    case ST_CMD_IDLE:
        pru_queue->state = STATE_IDLE;
        break;

    case ST_CMD_TEST:
        pru_queue->homing_dir  = cmd->test.dir;
        pru_queue->homing_time = NSEC_PER_SEC / cmd->test.speed / DELAY_PER_STEP;
        pru_queue->state = STATE_TEST;
        break;

    case ST_CMD_LMSW_CONFIG:
        switch (cmd->lmsw.axis) {
            case X_AXIS:
            case Y_AXIS:
            case Z_AXIS:
                   pru_queue->invert_endstop &= ~(1<< (cmd->lmsw.axis));
                   pru_queue->invert_endstop |= (cmd->lmsw.min_invert) << (cmd->lmsw.axis);
                   break;

            case AUTOLEVEL_Z_AXIS:
                   pru_queue->invert_endstop &= ~(1<< 3);
                   pru_queue->invert_endstop |= (cmd->lmsw.min_invert) << (3);
                   break;
        }
       printf("lkj invert_endstop,axis 0x%x, invert:%x\n", cmd->lmsw.axis, cmd->lmsw.min_invert);
       printf("lkj invert_endstop, 0x%x\n", pru_queue->invert_endstop);
        break;

    case ST_CMD_SET_POS:
        pru_queue->pos_x = cmd->pos.x; 
        pru_queue->pos_y = cmd->pos.y; 
        pru_queue->pos_z = cmd->pos.z; 
        pru_queue->pos_e = cmd->pos.e; 
        break;

    case ST_CMD_GET_POS:
		printf("ST_CMD_GET_POS pru_queue->pos_z:%d\n", pru_queue->pos_z);
       // cmd->pos.x = abs(pru_queue->pos_x);
      //  cmd->pos.y = abs(pru_queue->pos_y);
      //  cmd->pos.z = abs(pru_queue->pos_z);
       // cmd->pos.e = abs(pru_queue->pos_e);
        cmd->pos.x = pru_queue->pos_x;
        cmd->pos.y = pru_queue->pos_y;
        cmd->pos.z = pru_queue->pos_z;
        cmd->pos.e = pru_queue->pos_e;
        break;
    
    case ST_CMD_SET_PAUSE_POS:
        pru_queue->pause_x = cmd->pos.x; 
        pru_queue->pause_y = cmd->pos.y; 
        pru_queue->pause_z = cmd->pos.z; 
        break;

    case ST_CMD_GET_PRU_STATE:
        cmd->state.state = pru_queue->state;
        break;

    case ST_CMD_SET_PRU_STATE:
        pru_queue->state = cmd->state.state;
        break;

    case ST_CMD_SET_CANCEL_Z_UP_POS:
        pru_queue->cancel_z_up_steps= cmd->pos.z; 
        break;

    case ST_CMD_GET_CANCEL_Z_UP_POS:
        cmd->pos.z = pru_queue->cancel_z_up_steps; 
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}
/*
 * pruss stepper ops
 */
int pruss_stepper_init(void)
{
    int i = 0;

    STEPPER_DBG("pruss_stepper_init\n");
    pruss_init();

    if (queue_mmap() == NULL) {
        printf("Couldn't map PRU memory for queue.\n");
        return -1;
    }
    
    /* Clear up queue buffer */
    pru_queue->state = STATE_IDLE;

    pru_queue->pos_x = 0;
    pru_queue->pos_y = 0;
    pru_queue->pos_z = 0;
    pru_queue->pos_e = 0;

    pru_queue->pause_x = 0;
    pru_queue->pause_y = 0;
    pru_queue->pause_z = 0;

    pru_queue->write_pos = 0; 
    pru_queue->mcode_count = 0;

    for (i = 0; i < QUEUE_LEN; i++) {
        pru_queue->ring_buf[i].state = STATE_EMPTY;
    }
    queue_pos = 0;


    pru_queue->pause_z_distance_steps = 0;

    pruss_enable();
    return 0;
}

void pruss_stepper_exit(void)
{
	STEPPER_DBG("pruss exit\n");
  	prussdrv_pru_disable(0);
  	prussdrv_pru_disable(1);
	pruss_exit();
	if (ddr_mem) {
		munmap(ddr_mem, 0x100000);
	}
	STEPPER_DBG("pruss exit end\n");
}

