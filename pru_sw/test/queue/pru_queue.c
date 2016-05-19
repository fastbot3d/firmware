/*--------------------------------------------------------------------
 * pru_queue.c
 --------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <sys/mman.h>
#include <sys/stat.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "pru_queue.h"

#define DEBUG 1

/* In calculation of delay cycles: number of bits shifted
 * for higher resolution.
 */
#define DELAY_CYCLE_SHIFT 5

/* We need two loops per motor step (edge up, edge down),
 * So we need to multiply step-counts by 2
 * This could be more, if we wanted to implement sub-step resolution with
 * more than one bit output per step (probably only with hand-built drivers).
 */
#define LOOPS_PER_STEP (1 << 1)

#define PRU_NUM       (0)

struct queue_element {
    /* queue header */
    uint8_t state;
    uint8_t direction_bits;
    uint16_t reserved;

    /* Travel Parameters */
    uint16_t loops_accel;          /* Phase 1: loops spent in acceleration */
    uint16_t loops_travel;         /* Phase 2: loops spent in travel */
    uint16_t loops_decel;          /* Phase 3: loops spent in deceleration */
    uint16_t aux;                  /* right now: only lowest 2 bits */
    uint32_t accel_series_index;   /* index in taylor */

    uint32_t hires_accel_cycles;   /* acceleration delay cycles */ 
    uint32_t travel_delay_cycles;  /* travel delay cycles */

    uint32_t fractions[NR_MOTORS]; /* fixed point fractions to add each step */
} __attribute__((packed));

/* Mask for the output map when a particular endswitch hits. 
 * The first array element is the switch in question. 
 * The second the direction we are turning.
 * This is important, because we want a switch e.g. on the left be stopping
 * the stepper if it goes left, but we want to allow it to 'escape' going
 * to the right.
 */
struct lmsw_mask {
    uint32_t mask[3][2];
};

/* The communication with the PRU. We memory map the static RAM in the PRU
 * and write stuff into it from here. Mostly this is a ring-buffer with
 * commands to execute, but also configuration data, such as what to do when
 * an endswitch fires.
 */
struct queue {
    volatile struct queue_element ring_buf[QUEUE_LEN];
    volatile struct lmsw_mask lmsw;
};

static volatile struct queue *pru_queue = NULL;
static unsigned int queue_pos = 0;

static volatile struct queue *map_pru_queue(void)
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

static volatile struct queue_element *next_queue_element(void)
{
    queue_pos %= QUEUE_LEN;
    /* wait for an available queue element */
    while (pru_queue->ring_buf[queue_pos].state != STATE_EMPTY) {
        prussdrv_pru_wait_event(PRU_EVTOUT_0);
        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
    }
    return &pru_queue->ring_buf[queue_pos++];
}

#if DEBUG
static void dump_queue_element(volatile struct queue_element *e)
{
    if (e->state == STATE_EXIT) {
        fprintf(stderr, "enqueue[%02td]: EXIT\n", e - pru_queue->ring_buf);
    } else {
        int i;
        struct queue_element element = *e;
        fprintf(stderr, "enqueue[%02td]: dir:0x%02x s:(%5d + %5d + %5d) = %5d "
                "ad: %d; tf %d ",
                e - pru_queue->ring_buf, 
                element.direction_bits, 
                element.loops_accel,
                element.loops_travel,
                element.loops_decel,
                element.loops_accel + element.loops_travel + element.loops_decel,
                element.hires_accel_cycles >> DELAY_CYCLE_SHIFT,
                element.travel_delay_cycles);

        for (i = 0; i < NR_MOTORS; i++) {
            if (element.fractions[i] == 0) {
                continue;
            }
            //fprintf(stderr, "f%d:0x%08x ", i, element.fractions[i]);
        }
        fprintf(stderr, "\n");
    }
}
#endif

static void enqueue_element(struct queue_element *element)
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
    volatile struct queue_element *qe = next_queue_element();
    *qe = *element;

    /* Fully inited. Tell busy-waiting PRU by flipping the state */
    qe->state = state_to_send;
    
#if DEBUG
    dump_queue_element(qe);
#endif
}

/* do not go over 1MHZ */
#define HW_FREQ_LIMIT  1e6
static float clip_hw_freq_limit(float v)
{
    return v < HW_FREQ_LIMIT ? v : HW_FREQ_LIMIT;
}

/* two cycles per loop */
static double cycles_per_second()
{
    return 100e6;
}

static int enqueue_internal(const struct movement *mv, int max_steps)
{
    int i;
    struct queue_element qe;
    qe.direction_bits = 0;

    /* The max_steps is the number of steps of the axis that requires
     * the most number of steps. All the others are a fraction of the steps.
     *
     * The top bits have LOOPS_PER_STEP states (2 is the minium, as we need two
     * cycles for a 0 1 transition. So in that case we have 31 bit fraction
     * and 1 bit that overflows and toggles for the steps we want to generate.
     */
    const uint64_t max_fraction = 0xFFFFFFFF / LOOPS_PER_STEP;

    for (i = 0; i < NR_MOTORS; i++) {
        if (mv->steps[i] < 0) {
            qe.direction_bits |= (1 << i);
        }
        
        const uint64_t delta = abs(mv->steps[i]);
        qe.fractions[i] = delta * max_fraction / max_steps;
    }

    const float travel_speed = clip_hw_freq_limit(mv->travel_speed);

    /* Calculate speeds
     * First step while exerpimenting: assume start/endspeed always 0.
     * TODO: take these into account (requires acceleration planning)
     */
    const int total_loops = LOOPS_PER_STEP * max_steps;
    if (mv->acceleration <= 0) {
        qe.loops_accel = qe.loops_decel = 0;  //FIXME
    } else {
        /* Steps to reach requested speed at acceleration
         * v = a*t -> t = v/a
         * s = a/2 * t^2; subsitution t from above: s = v^2/(2*a)
         */
        const int accel_loops = LOOPS_PER_STEP * (travel_speed * travel_speed 
                                                  / (2.0 * mv->acceleration));
        if ( 2 * accel_loops < total_loops) {
            qe.loops_accel = accel_loops;
            qe.loops_travel = total_loops - 2 * accel_loops;
            qe.loops_decel = accel_loops;
        } else {
            /* We don't want deceleration have more steps than acceleration (the
             * iterative approximation will not be happy), so let's make sure to have
             * accel_steps >= decel_steps by using the fact that integer div
             * essentially does floor()
             */
            qe.loops_decel = total_loops / 2;
            qe.loops_travel = 0;
            qe.loops_accel = total_loops - qe.loops_decel;
        }
    
        double accel_factor = cycles_per_second()
                              * (sqrt(LOOPS_PER_STEP * 2.0 / mv->acceleration));

        qe.accel_series_index = 0;
        qe.hires_accel_cycles = ((1 << DELAY_CYCLE_SHIFT) * accel_factor 
                                 * 0.67605 / LOOPS_PER_STEP);
    }

    qe.travel_delay_cycles = cycles_per_second() 
                             / (LOOPS_PER_STEP * travel_speed);
    qe.aux = mv->aux_bits;
    qe.state = STATE_FILLED;
    
    /* Write element into queue mem */
    enqueue_element(&qe); 

    return 0;
}

int pruss_enqueue(const struct movement *mv)
{
    int i;
    /* TODO: this function should automatically split this into multiple segments
     * each with the maximum number of steps.
     */
    int max_steps = abs(mv->steps[0]); 

    for (i = 0; i < NR_MOTORS; i++) {
        if (abs(mv->steps[i] > max_steps)) {
            max_steps = abs(mv->steps[i]);
        }
    }

    if (max_steps == 0) {
        fprintf(stderr, "zero steps. Ignoring command.\n");
        return -1;
    }
    if (max_steps > 65535 / LOOPS_PER_STEP) {
        fprintf(stderr, "At most %d steps, got %d. Ignoring command.\n",
                65535 / LOOPS_PER_STEP, max_steps);
        return -2;
    }

    enqueue_internal(mv, max_steps);
    return 0;
}

void pruss_wait_queue_empty(void)
{
    const unsigned int last = (queue_pos - 1) % QUEUE_LEN;
    while (pru_queue->ring_buf[last].state != STATE_EMPTY) {
        prussdrv_pru_wait_event(PRU_EVTOUT_0);
        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
    }
}

int pruss_init(void)
{
    int ret;
    tpruss_intc_initdata intc_initdata = PRUSS_INTC_INITDATA; 
    
    prussdrv_init();
    
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret) {
        printf("prussdrv_open failed\n");
        return ret;
    }
    prussdrv_pruintc_init(&intc_initdata);    
    
    if (map_pru_queue() == NULL) {
        fprintf(stderr, "Couldn't map PRU memory for queue.\n");
        return -1;
    }

    //prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, 0, PRUcode, sizeof(PRUcode));
    //prussdrv_pru_enable(PRU_NUM);

    printf("PRU loading code : pru_queue.bin\n");
    prussdrv_exec_program(PRU_NUM, "./pru_queue.bin");
    return 0;
}

void pruss_exit_no_wait(void)
{
    printf("Disable PRU and close memory mapping\n");
    prussdrv_pru_disable(PRU_NUM);
    prussdrv_exit();
}

void pruss_exit(void)    
{
    struct queue_element qe;
    bzero(&qe, sizeof(qe)); 

    qe.state = STATE_EXIT;

    enqueue_element(&qe);

    printf("wait queue empty\n");
    pruss_wait_queue_empty();

    printf("pruss disable\n");
    prussdrv_pru_disable(PRU_NUM);

    printf("pruss exit\n");
    prussdrv_exit();
}

int main(int argc, char **argv)
{
    int i;
    struct movement mv; 
    bzero(&mv, sizeof(struct movement));

    printf("\nStarting %s example.\r\n", "pru_gpio");
    pruss_init();
    
    mv.start_speed = 400;
    mv.travel_speed = 400;
    mv.end_speed = 400;
    
    mv.acceleration = 100;

    mv.steps[0] = -100;
    mv.steps[1] = 512;
    mv.steps[2] = -10;
    printf("pruss enqueue\n");
    for (i = 0; i < 1024; i++) {
        pruss_enqueue(&mv);        
    }
    
    pruss_exit();

    return 0;
}

