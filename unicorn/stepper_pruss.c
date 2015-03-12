/*
 * Unicorn 3D Printer Firmware
 * stepper_pruss.c
 * Stepper motor control module using pruss
 * Copyright (c) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * Unicorn is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Unicorn is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, see <http://www.gnu.org/licenses/>.
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

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include <stepper_spi.h>

#include "common.h"
#include "parameter.h"
#include "planner.h"
#include "pruss.h"
#include "stepper.h"
#include "stepper_pruss.h"

struct queue_element {
    /* queue header */
    uint8_t  state;
    uint8_t  direction_bits;
    uint8_t  direction;
    uint8_t  reserved;

    /* Travel Parameters */
    uint16_t loops_accel;    /* Phase 1: loops spent in acceleration */
    uint16_t loops_travel;   /* Phase 2: loops spent in travel */
    uint16_t loops_decel;    /* Phase 3: loops spent in deceleration */
    uint16_t steps_count;    /* Max step event count */

    uint32_t accel_cycles;   /* acceleration delay cycles */ 
    uint32_t travel_cycles;  /* travel delay cycles */
    uint32_t decel_cycles;   /* decelerate delay cycles */
    
    uint32_t init_cycles;    /* init enter entry delay cycles */
    uint32_t final_cycles;   /* final exit entry delay cycles */

    uint16_t steps_x;        /* fraction of steps count of axis x */
    uint16_t steps_y;        /* fraction of steps count of axis y */
    uint16_t steps_z;        /* fraction of steps count of axis z */
    uint16_t steps_e;        /* fraction steps count of axis e */
} __attribute__((packed));

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
struct queue {
    volatile struct queue_element ring_buf[QUEUE_LEN];
    volatile uint32_t state;

    volatile uint32_t homing_axis;
    volatile uint32_t homing_dir;
    volatile uint32_t homing_time;

    volatile uint32_t pos_x;
    volatile uint32_t pos_y; 
    volatile uint32_t pos_z;
    volatile uint32_t pos_e;

    volatile uint32_t origin_x;
    volatile uint32_t origin_y;
    volatile uint32_t origin_z;
    
    volatile uint32_t write_pos;

    volatile struct lmsw lmsw_x;
    volatile struct lmsw lmsw_y;
    volatile struct lmsw lmsw_z;
};
static volatile struct queue *pru_queue = NULL;
static unsigned int queue_pos = 0;

/*
 * pruss queue operation
 */
static volatile struct queue *queue_mmap(void)
{
    int i;
    void *dram = NULL;
    prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &dram);
    bzero(dram, sizeof(*pru_queue));

    pru_queue = (struct queue *)dram;

    for (i = 0; i < QUEUE_LEN; i++) {
        pru_queue->ring_buf[i].state = STATE_EMPTY;
    }
    queue_pos = 0;

    return pru_queue;
}

static volatile struct queue_element *queue_get_next_element(void)
{
    uint32_t next = queue_pos + 1;
    queue_pos %= QUEUE_LEN;
    next %= QUEUE_LEN;

    /* Update queue pos */
    pru_queue->write_pos = next * sizeof(struct queue_element);

    /* wait for an available queue element */
    while (pru_queue->ring_buf[next].state != STATE_EMPTY) {
        prussdrv_pru_wait_event(PRU_EVTOUT_0);
        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
    }

    return &pru_queue->ring_buf[queue_pos++];
}

//test
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

        if (element.steps_x) {
            fprintf(stderr, "x:%d ", element.steps_x);
        }
        if (element.steps_y) {
            fprintf(stderr, "y:%d ", element.steps_y);
        }
        if (element.steps_z) {
            fprintf(stderr, "z:%d ", element.steps_z);
        }
        if (element.steps_e) {
            fprintf(stderr, "e:%d ", element.steps_e);
        }
        fprintf(stderr, "\n");

        fprintf(stderr, "                    ac: %d, dc: %d, ic: %d, tc %d, fc: %d ",
                element.accel_cycles,
                element.decel_cycles,
                element.init_cycles,
                element.travel_cycles,
                element.final_cycles);

        fprintf(stderr, "\n");
    }
}

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

    if (DBG(D_STEPPER)) {
        queue_dump_element(qe);
    }
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

    if (!block) {
        return -1;
    }

    /* queue type */
    qe.state = STATE_FILLED; 

    /* Direction Setting */ 
    dir = block->direction_bits; 
    qe.direction = block->direction_bits;

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

    /* Calculate delay */
    qe.init_cycles   = NSEC_PER_SEC / block->initial_rate / DELAY_PER_STEP;
    qe.final_cycles  = NSEC_PER_SEC / block->final_rate / DELAY_PER_STEP; 
    qe.travel_cycles = NSEC_PER_SEC / block->nominal_rate / DELAY_PER_STEP;

    if (qe.loops_accel != 0) {
        qe.accel_cycles = (qe.init_cycles - qe.travel_cycles) / qe.loops_accel;
    } else {
        qe.accel_cycles = 0;
    }
    if (qe.loops_decel != 0) {
        qe.decel_cycles = (qe.final_cycles - qe.travel_cycles) / qe.loops_decel;
    } else {
        qe.decel_cycles = 0;
    }

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

#if 1
    if (DBG(D_STEPPER)) {
        printf("                    entry %ld, nominal %ld, final %ld, acc %ld\n\n", 
                block->initial_rate, block->nominal_rate, block->final_rate, 
                block->acceleration_st);
        //printf("accelerate_until %ld, decelerate_after %ld, step_event_count %ld\n",
        //        block->accelerate_until, block->decelerate_after, block->step_event_count);

    }
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

int pruss_queue_wait(void)
{
    const unsigned int last = (queue_pos - 1) % QUEUE_LEN;

    while (pru_queue->ring_buf[last].state != STATE_EMPTY) {
        //prussdrv_pru_wait_event(PRU_EVTOUT_0);
        //prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
        usleep(10000);
    }

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
            if (cycles < min_cycles) {
                min_cycles = cycles;
            }
        }
    }

    max_rate = NSEC_PER_SEC / (min_cycles * DELAY_PER_STEP);

    return max_rate;
}

int pruss_queue_get_len(void)
{
    int i; 
    int len = 0; 

    for (i = 0; i < QUEUE_LEN; i++) {
        if (pru_queue->ring_buf[i].state != STATE_EMPTY) {
            len++; 
        }
    }

    return len;
}

int pruss_send_cmd(st_cmd_t *cmd)
{
    int ret = 0; 

    switch (cmd->gen[0]) {
    case ST_CMD_AXIS_HOMING:
        pru_queue->homing_axis |= 1 << cmd->homing.axis;
        pru_queue->homing_dir  = cmd->homing.dir;
        pru_queue->homing_time = NSEC_PER_SEC / cmd->homing.speed / DELAY_PER_STEP;
        pru_queue->state = STATE_HOME;
        break;

    case ST_CMD_START:
        pru_queue->state = STATE_PRINT;
        break;

    case ST_CMD_STOP:
        pru_queue->state = STATE_STOP;
        break;

    case ST_CMD_PAUSE:
        pru_queue->state = STATE_PAUSE; 
        break;

    case ST_CMD_RESUME:
        pru_queue->state = STATE_RESUME;
        break;

    case ST_CMD_LMSW_CONFIG:
        break;

    case ST_CMD_SET_POS:
        pru_queue->origin_x = cmd->pos.x; 
        pru_queue->origin_y = cmd->pos.y; 
        pru_queue->origin_z = cmd->pos.z; 
        break;

    case ST_CMD_GET_POS:
        cmd->pos.x = pru_queue->pos_x;
        cmd->pos.y = pru_queue->pos_y;
        cmd->pos.z = pru_queue->pos_z;
        cmd->pos.e = pru_queue->pos_e;
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
    int i;
    float speed[3];
    float max_speed;

    uint32_t dir = 0;
    fprintf(stderr, "pruss_stepper_init\n");

    pruss_init();

    if (queue_mmap() == NULL) {
        fprintf(stderr, "Couldn't map PRU memory for queue.\n");
        return -1;
    }

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
    
    max_speed = fmax(speed[0], fmax(speed[1], speed[2])) / 60.0,

    pru_queue->state = STATE_IDLE;
    pru_queue->homing_dir  = dir; 
    pru_queue->homing_time = NSEC_PER_SEC / max_speed / DELAY_PER_STEP;
    pru_queue->homing_axis = 0; 

    pru_queue->pos_x = 0;
    pru_queue->pos_y = 0;
    pru_queue->pos_z = 0;
    pru_queue->pos_e = 0;

    pru_queue->origin_x = 0;
    pru_queue->origin_y = 0;
    pru_queue->origin_z = 0;
    
    pruss_enable();
    return 0;
}

void pruss_stepper_exit(void)
{
    /* wait all queue element processed by pru */
    //pruss_queue_wait(); 

    fprintf(stderr, "pruss exit\n");
    pruss_exit();
}

