/*
 * Unicorn 3D Printer Firmware
 * planner.c
*/

/*  
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 * s == speed, a == acceleration, t == time, d == distance
 *  
 * Basic definitions:
 * Speed[s_, a_, t_] := s + (a*t) 
 * Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *  
 * Distance to reach a specific speed with a constant acceleration:
 * Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 * d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 * 
 * Speed after a given distance of travel with constant acceleration:
 * Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 * m -> Sqrt[2 a d + s^2]    
 *  
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 * 
 * When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:
 *  
 * Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 * di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 * 
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <semaphore.h>

#include "common.h"
#include "parameter.h"
#include "unicorn.h"
#include "planner.h"
#include "stepper.h"

static bool stop = false;

/*
 * Public variables
 */ 
int extrudemultiply = 100;

/* everything with less than this number of steps will be ignored 
 * as move and joined with the next movement */
const unsigned int dropsegments = 5;

unsigned long axis_steps_per_sqr_second[NUM_AXIS];

/*
 * Semi-private variables, used in inline functions
 */
#define BLOCK_BUFFER_SIZE 32
#define BLOCK_BUFFER_MASK 0x1f

/* A ring buffer for motion instructions */
block_t block_buffer[BLOCK_BUFFER_SIZE];   

/* Index of next block to be pushed */
volatile unsigned char block_buffer_head;  

/* Index of the block to process now */
volatile unsigned char block_buffer_tail;  

/* The current position of the tool in absolute steps */
/* rescaled from extern when axis_steps_per_unit are changed by gcode */
long position[4];                         

/* Speed of previous path line segment */
static float previous_speed[4];           

/* Nominal speed of previous path line segment */
static float previous_nominal_speed;      

/*
 * Returns the index of the next block in the ring buffer
 */
static int8_t next_block_index(int8_t idx) 
{
    idx++;
    if (idx == BLOCK_BUFFER_SIZE) {
        idx = 0;
    }
    return idx;
}
/*
 * Returns the index of the previous block in the ring buffer
 */
static int8_t prev_block_index(int8_t idx)
{
    if (idx == 0) {
        idx = BLOCK_BUFFER_SIZE;
    }
    idx--;
    return idx;
}
/*
 * Calculates the distance (not time) it takes to accelerate from initial_rate to
 * target_rate using the given acceleration:
 */
static inline float estimate_acceleration_distance(float initial_rate, 
                                                   float target_rate, 
                                                   float acceleration)
{
    if (acceleration != 0) {
        return ((target_rate * target_rate - initial_rate * initial_rate) 
                / (2.0 * acceleration));
    } else {
        return 0.0;
    }
}
/*
 * This function gives you the point at which you must start braking (at the rate of -acceleration)
 * if you started at speed initial_rate and accelerated untill this point and want to end at the
 * final_rate after a total travel of distance.
 * This can be used to compute the intersection point between acceleration and deceleration
 * in the case where the trapezoid has no plateau (i.e. never reaches maximum speed)
 */
static inline float intersection_distance(float initial_rate, 
                                          float final_rate, 
                                          float acceleration, 
                                          float distance)
{
    if (acceleration != 0) {
        return ((2.0 * acceleration * distance - initial_rate * initial_rate + final_rate * final_rate)
                / (4.0 * acceleration));
    } else {
        return 0.0;
    }
}
/*
 * Calculates trapezoid parameters so tha the entry-speed and exit-speed is compensated by
 * the provided factors.
 */
static void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor)
{
    unsigned long initial_rate;
    unsigned long final_rate;

    initial_rate = ceil(block->nominal_rate * entry_factor); // step/s
    final_rate   = ceil(block->nominal_rate * exit_factor);  // step/s
    
    /* Limit minimal step rate, otherwisw the timer will overflow. */
    if (initial_rate < 120) {
        initial_rate = 120;
    }
    if (final_rate < 120) {
        final_rate = 120;
    }

    long acceleration = block->acceleration_st;
    float distance;
    distance = estimate_acceleration_distance(initial_rate, 
                                              block->nominal_rate, 
                                              acceleration);
    int32_t accelerate_steps = ceil(distance);
#if 0
    if (accelerate_steps < 0) {
        accelerate_steps = 0;
    }
#endif
    distance = estimate_acceleration_distance(block->nominal_rate, 
                                              final_rate, 
                                              -acceleration);
    int32_t decelerate_steps = floor(distance);
#if 0
    if (decelerate_steps < 0) {
        decelerate_steps = 0;
    }
#endif
    /* Calculate the size of Plateau of Nominal Rate */
    int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

    /* Is the Plateau of Nominal Rate smaller than nothing?
     * That means no cruising, and we will have intersection_distance() to calculate 
     * when to abort acceleration and start braking in order to reach the final_rate 
     * exactly at the end of this block.
     */
    if (plateau_steps < 0) { 
        distance = intersection_distance(initial_rate, 
                                         final_rate, 
                                         acceleration, 
                                         block->step_event_count);
        accelerate_steps = ceil(distance);

        /* Check limit due to numerical round-off */
        accelerate_steps = max(accelerate_steps, 0);

        /* We can cast here to unsigned, because the above line ensures that we are above zero */
        accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);
        plateau_steps = 0;
    }

    /* critical section start */
    if (block->busy == false) {
        block->accelerate_until = accelerate_steps;
        block->decelerate_after = accelerate_steps + plateau_steps;
        block->initial_rate     = initial_rate;  
        block->final_rate       = final_rate;
    }
    /* critical section end */
}
/*
 * Calculates the maximum allowable speed at this point when you must be able to reach
 * target_velocity using the acceleration within the allotted distance.
 */
static inline float max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}
/*
 * The kernel called by planner_recalculate() when scanning the plan from the last to fist entry.
 */
static void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next)
{
    if (!current) { 
        return;
    }

    if (next) {
        /*
         * If entry speed is already at the maximum entry speed, no need to to recheck.
         * Block is cruising. 
         * If not, block in state of acceleration or deceleration. 
         * Reset entry speed to maximum and check for maximum allowable speed reduction
         * to ensure maximum possible planned speed.
         */
        if (current->entry_speed != current->max_entry_speed) {
            /*
             * If nominal length true, max junction speed is guaranteed to be reached.
             * Only compute for max allowable speed if block is decelerating 
             * and nominal length is false.
             */
            if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
                current->entry_speed = min(current->max_entry_speed, 
                                           max_allowable_speed(-current->acceleration, 
                                                               next->entry_speed,
                                                               current->millimeters));
            } else {
                current->entry_speed = current->max_entry_speed;
            }
            current->recalculate_flag = true;
        }
    } // Skip last block. Already initialized and set for recalculation.
}
/*
 * planner_recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the reverse pass.
 */
static void planner_reverse_pass(void) 
{
    uint8_t block_index = block_buffer_head;
    
    /* Make a local copy of block_buffer_tail, because the interrupt can alter it */
    unsigned char tail = block_buffer_tail; 

    if (((block_buffer_head - tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
        block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
        block_t *block[3] = {NULL, NULL, NULL};

        while (block_index != tail) {
            block_index = prev_block_index(block_index);
            block[2] = block[1];
            block[1] = block[0];
            block[0] = &block_buffer[block_index];
            planner_reverse_pass_kernel(block[0], block[1], block[2]);
        }
    }
}
/*
 * The kernel called by planner_recalculate() when scanning the plan from first to the last entry.
 */
static void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next)
{
    if (!previous) {
        return;
    }

    /*
     * If the previous block is an acceleration block, but it is not long enough to 
     * complete the full change within the block, we need to adjust the entry speed accordingly.
     * Entry speeds have already been reset, maximized, and reverse planned by reverse planner.
     * If nominal length is true, max junction speed is guaranteed to be reached.
     * No need to recheck.
     */
    if (!previous->nominal_length_flag) {
        if (previous->entry_speed < current->entry_speed) {
            double entry_speed = min(current->entry_speed, 
                                     max_allowable_speed(-previous->acceleration,
                                                          previous->entry_speed, 
                                                          previous->millimeters));
            /* Check for junction speed change */
            if (current->entry_speed != entry_speed) {
                current->entry_speed = entry_speed;
                current->recalculate_flag = true;
            } 
        }
    }
}
/*
 * planner_recalculate() needs to go over the current plan twice. 
 * Once in reverse and once forward. This implements the forward pass.
 */
static void planner_forward_pass(void)
{
    uint8_t block_index = block_buffer_tail; 
    block_t *block[3] = {NULL, NULL, NULL};

    while (block_index != block_buffer_head) {
        block[0] = block[1];
        block[1] = block[2];
        block[2] = &block_buffer[block_index];

        planner_forward_pass_kernel(block[0], block[1], block[2]);
        block_index = next_block_index(block_index);
    }
    
    planner_forward_pass_kernel(block[1], block[2], NULL);
}
/*
 * Recalculate the trapezoid speed profiles for all blocks in the plan according to the
 * entry_factor for each junction.
 * Must be called by planner_recalculate() after updating the blocks.
 */
static void planner_recalculate_trapezoids(void)
{
    int8_t block_index = block_buffer_tail;
    block_t *current;
    block_t *next = NULL;

    while (block_index != block_buffer_head) {
        current = next; 
        next = &block_buffer[block_index]; 

        if (current) {
            /* Recalculate if current block entry or exit junction speed has changed. */
            if (current->recalculate_flag || next->recalculate_flag) {
                /* Note: Entry and exit factors always > 0 by all previous logic operations */
                calculate_trapezoid_for_block(current, 
                                              current->entry_speed / current->nominal_speed,
                                              next->entry_speed / current->nominal_speed);
                /* Reset current only to ensure next trapezoid is computed */
                current->recalculate_flag = false;
            }
        }

        block_index = next_block_index(block_index);
    }

    /* Last/newest block in buffer.
     * Exit speed is set with MINIMUM_PLANNER_SPEED.
     * Always recalculated.
     */
    if (next != NULL) {
        calculate_trapezoid_for_block(next, 
                                      next->entry_speed / next->nominal_speed,
                                      MINIMUM_PLANNER_SPEED / next->nominal_speed);
        next->recalculate_flag = false;
    }
}
/* 
 * Recalculates the motion plan according to the following algorithm:
 * 
 * 1. Go over every block in reverse order and calculate a junction speed reduction 
 * (i.e. block_t.entry_factor), so that:
 *    a. The junction jerk is within the set limit
 *    b. No speed reduction within one block requires faster deceleration than the one, true constant 
 *       acceleration.
 *
 * 2. Go over every block in chronological order and dial down junction speed reduction values if 
 *    a. The speed increase within one block would require faster accelleration than the one, true 
 *       constant acceleration.
 *
 * When these stages are complete all blocks have an entry_factor that will allow all speed changes 
 * to be performed using only the one, true constant acceleration, and where no junction jerk 
 * is jerkier than the set limit. Finally it will:
 *
 * 3. Recalculate trapezoids for all blocks.
 */
static void planner_recalculate(void)
{
    planner_reverse_pass();
    planner_forward_pass();
    planner_recalculate_trapezoids();
}
/*
 * Init the planner sub system
 */
int plan_init(void)
{
    int i;
    
    for (i = 0; i < NUM_AXIS; i++) {
        axis_steps_per_sqr_second[i] = pa.max_acceleration_units_per_sq_second[i] 
                                       * pa.axis_steps_per_unit[i];
    }

    /* clear position */
    memset(position, 0, sizeof(position));

    block_buffer_head = 0;
    block_buffer_tail = 0;
    
    previous_speed[0] = 0.0;
    previous_speed[1] = 0.0;
    previous_speed[2] = 0.0;
    previous_speed[3] = 0.0;
    
    previous_nominal_speed = 0.0;
    return 0;
}
/*
 * Add a new linear movement to the buffer.
 * x, y and z is the signed, absolute target position in millimeters.
 * Feed rate specfies the speed of the motion.
 * Microseconds specify how many microseconds the move should take to perform.
 * To aid acceleration calculation the caller must also provide the physical 
 * length of the line in millimeters.
 */
void plan_buffer_line(float x, float y, float z, const float e, 
                      float feed_rate, const uint8_t extruder)
{
    int i;

    /* Calculate the buffer head after we push this byte */
    int next_buffer_head = next_block_index(block_buffer_head);

    while (block_buffer_tail == next_buffer_head) {
        /* waste so many cpu resource here */
        usleep(1000);
        if (stop) {
            break;
        }
    }
    
    if (stop) {
        return;
    }

    /*---------------------------------------------------------
     * 1. Calculate the target position in absolute steps 
     ---------------------------------------------------------*/
    long target[4];
    target[X_AXIS] = lround(x * pa.axis_steps_per_unit[X_AXIS]);
    target[Y_AXIS] = lround(y * pa.axis_steps_per_unit[Y_AXIS]);
    target[Z_AXIS] = lround(z * pa.axis_steps_per_unit[Z_AXIS]);
    target[E_AXIS] = lround(e * pa.axis_steps_per_unit[E_AXIS]);
    
    /* Prepare to set up new block */
    block_t *block = &block_buffer[block_buffer_head]; 

    /* Mark block as not busy, (Not executed by the stepper thread) */
    block->busy = false;

    /* Number of steps for each axis */
#ifndef COREXY
    /* default non-h-bot planning */
    block->steps_x = labs(target[X_AXIS] - position[X_AXIS]);
    block->steps_y = labs(target[Y_AXIS] - position[Y_AXIS]);
#else
    /* corexy planning 
     * these equations follow the form of the dA and dB equations on
     * http://www.corexy.com/theory.html
     */
    block->steps_x = labs((target[X_AXIS] - position[X_AXIS]) + (target[Y_AXIS] - position[Y_AXIS]));
    block->steps_y = labs((target[X_AXIS] - position[X_AXIS]) - (target[Y_AXIS] - position[Y_AXIS]));
#endif
    
    block->steps_z = labs(target[Z_AXIS] - position[Z_AXIS]);
    block->steps_e = labs(target[E_AXIS] - position[E_AXIS]);
    block->steps_e *= extrudemultiply;
    block->steps_e /= 100;

    block->step_event_count = max(block->steps_x, 
                                  max(block->steps_y, 
                                      max(block->steps_z, block->steps_e)));

    /* Bail if this is a zero-length block */
    if (block->step_event_count <= dropsegments) {
        return;
    }
    
    /*---------------------------------------------------------
     * 2. Compute direction bits for this block 
     *    1 -> - directtion
     *    0 -> + direction
     ---------------------------------------------------------*/
    block->direction_bits = 0;

#ifndef COREXY
    if (target[X_AXIS] < position[X_AXIS]) {
        block->direction_bits |= (1 << X_AXIS);
    }
    if (target[Y_AXIS] < position[Y_AXIS]) {
        block->direction_bits |= (1 << Y_AXIS);
    }
#else
    if ((target[X_AXIS] - position[X_AXIS]) + (target[Y_AXIS] - position[Y_AXIS]) < 0) { 
        block->direction_bits |= (1 << X_AXIS);
    }
    if ((target[X_AXIS] - position[X_AXIS]) - (target[Y_AXIS] - position[Y_AXIS]) < 0) {
        block->direction_bits |= (1 << Y_AXIS);
    }
#endif

    if (target[Z_AXIS] < position[Z_AXIS]) {
        block->direction_bits |= (1 << Z_AXIS);
    }
    
    if (target[E_AXIS] < position[E_AXIS]) {
        block->direction_bits |= (1 << E_AXIS);
    }

    block->active_extruder = extruder;

    /*---------------------------------------------------------
     * 3. enable active axis 
     ---------------------------------------------------------*/
#ifdef COREXY 
    if ((block->steps_x != 0) || (block->steps_y != 0)) {
        //stepper_enable(X_AXIS);
        //stepper_enable(Y_AXIS);
    }
#else
    if (block->steps_x != 0) {
        //stepper_enable(X_AXIS);
    }
    if (block->steps_y != 0) {
        //stepper_enable(Y_AXIS);
    }
#endif

    if (block->steps_z != 0) {
        //stepper_enable(Z_AXIS);
    }

    if (block->steps_e != 0) {
        //stepper_enable(E_AXIS);
    }

    //FIXME   
    //stepper_update();

    /*---------------------------------------------------------
     * 4. Calculate speed
     ---------------------------------------------------------*/
    if (block->steps_e == 0) {
        if (feed_rate < pa.mintravelfeedrate) {
            feed_rate = pa.mintravelfeedrate;
        }
    } else {
        if (feed_rate < pa.minimumfeedrate) {
            feed_rate = pa.minimumfeedrate;
        }
    } 
    
    int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE)
                       & (BLOCK_BUFFER_SIZE - 1);
    
#ifdef SLOWDOWN
    if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5))) {
        feed_rate = feed_rate * moves_queued / (BLOCK_BUFFER_SIZE * 0.5);
    }
#endif

    float delta_mm[4];
#ifndef COREXY
    delta_mm[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / pa.axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / pa.axis_steps_per_unit[Y_AXIS];
#else
    delta_mm[X_AXIS] = ((target[X_AXIS] - position[X_AXIS]) + (target[Y_AXIS] - position[Y_AXIS]))
                       / pa.axis_steps_per_unit[X_AXIS];
    delta_mm[Y_AXIS] = ((target[X_AXIS] - position[X_AXIS]) - (target[Y_AXIS] - position[Y_AXIS]))
                       / pa.axis_steps_per_unit[Y_AXIS];
#endif
    
    delta_mm[Z_AXIS] = (target[Z_AXIS] - position[Z_AXIS]) / pa.axis_steps_per_unit[Z_AXIS];
    delta_mm[E_AXIS] = ((target[E_AXIS] - position[E_AXIS]) / pa.axis_steps_per_unit[E_AXIS])
                       * extrudemultiply / 100.0;
    
    if (block->steps_x <= dropsegments && block->steps_y <= dropsegments 
            && block->steps_z <= dropsegments) {
        block->millimeters = fabs(delta_mm[E_AXIS]);
    } else {
        block->millimeters = sqrtf(powf(delta_mm[X_AXIS], 2)
                                  + powf(delta_mm[Y_AXIS], 2)
                                  + powf(delta_mm[Z_AXIS], 2));
    }
    
    /* Inverse millimeters to remove multiple divides */
    float inverse_millimeters = 1.0 / block->millimeters;
    
    /* Calculate speed in mm/second for each axis.
     * No divide by zero due to previous checks.
     */
    float inverse_second = feed_rate * inverse_millimeters;
    
    block->nominal_speed = block->millimeters * inverse_second;           /* (mm/sec) Always > 0 */
    block->nominal_rate = ceil(block->step_event_count * inverse_second); /* (step/sec) Always > 0 */

    /* Calculate and limit speed in mm/sec for each axis */
    float current_speed[4];
    float speed_factor = 1.0; //factor <=1 do decrease speed
    for (i = 0; i < 4; i++) {
        current_speed[i] = delta_mm[i] * inverse_second;
        if (fabs(current_speed[i]) > pa.max_feedrate[i]) {
            speed_factor = min(speed_factor, pa.max_feedrate[i] / fabs(current_speed[i]));
        }
    }
    
    /* Correct the speed */
    if (speed_factor < 1.0) {
        for (i = 0; i < 4; i++) {
            current_speed[i] *= speed_factor;
        }

        block->nominal_speed *= speed_factor;
        block->nominal_rate *= speed_factor;
    }

    /* Compute and limit the acceleration rate for the trapezoid generator. */
    float steps_per_mm = block->step_event_count / block->millimeters;
    if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0) {
        /* convert to: acceleration steps/sec^2 */
        block->acceleration_st = ceilf(pa.retract_acceleration * steps_per_mm); 
    } else {
        /* convert to: acceleration steps/sec^2 */
        block->acceleration_st = ceilf(pa.move_acceleration * steps_per_mm);  //FIXME

        /* Limit acceleration per axis */
        if (((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) 
                > axis_steps_per_sqr_second[X_AXIS]) {
            block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
        }

        if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) 
                > axis_steps_per_sqr_second[Y_AXIS]) {
            block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
        }

        if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) 
                > axis_steps_per_sqr_second[Z_AXIS]) {
            block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
        }

        if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) 
                > axis_steps_per_sqr_second[E_AXIS]) {
            block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
        }
    }
    
    block->acceleration = block->acceleration_st / steps_per_mm;
    
    //FIXME 
    block->acceleration_rate = (long)((float)block->acceleration_st * 8.388608);

    /* Start with a safe speed */
    float vmax_junction = pa.max_xy_jerk / 2; 
    float vmax_junction_factor = 1.0; 
    if (fabs(current_speed[Z_AXIS]) > pa.max_z_jerk / 2)  {
        vmax_junction = min(vmax_junction, pa.max_z_jerk / 2);
    }

    if (fabs(current_speed[E_AXIS]) > pa.max_e_jerk/2) {
        vmax_junction = min(vmax_junction, pa.max_e_jerk/2);
    }

    vmax_junction = min(vmax_junction, block->nominal_speed);
    float safe_speed = vmax_junction;

    if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
        float jerk = sqrt(pow((current_speed[X_AXIS] - previous_speed[X_AXIS]), 2)
                        + pow((current_speed[Y_AXIS] - previous_speed[Y_AXIS]), 2));

        vmax_junction = block->nominal_speed;
        if (jerk > pa.max_xy_jerk) {
            vmax_junction_factor = (pa.max_xy_jerk / jerk);
        } 
        
        if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > pa.max_z_jerk) {
            vmax_junction_factor= min(vmax_junction_factor, 
                                      (pa.max_z_jerk 
                                       / fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
        } 

        if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > pa.max_e_jerk) {
            vmax_junction_factor = min(vmax_junction_factor, 
                                       (pa.max_e_jerk
                                        / fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
        } 

        /* Limit speed to max previous speed */
        vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); 
    }

    block->max_entry_speed = vmax_junction;

    /* Initialize block entry speed. 
     * Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.*/
    double v_allowable = max_allowable_speed(-block->acceleration,
                                             MINIMUM_PLANNER_SPEED,
                                             block->millimeters);
    block->entry_speed = min(vmax_junction, v_allowable);
    
    /* Initialize planner efficiency flags
     * Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
     * If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
     * the current block and next block junction speeds are guaranteed to always be at their maximum
     * junction speeds in deceleration and acceleration, respectively. This is due to how the current
     * block nominal speed limits both the current and next maximum junction speeds. Hence, in both
     * the reverse and forward planners, the corresponding block junction speed will always be at the
     * the maximum junction speed and may always be ignored for any speed reduction checks.
     */
    if (block->nominal_speed <= v_allowable) { 
        block->nominal_length_flag = 1; 
    } else { 
        block->nominal_length_flag = 0; 
    }
    block->recalculate_flag = 1; // Always calculate trapezoid for new block

    /* Update previous path unit_vector and nominal speed */
    memcpy(previous_speed, current_speed, sizeof(previous_speed));
    previous_nominal_speed = block->nominal_speed;

#if 0 
    printf("[Truby]: max_entry_speed %f, entry_speed %f, nominal_speed %f, safe_speed %f\n",
            block->max_entry_speed, block->entry_speed, block->nominal_speed, safe_speed);
#endif

    calculate_trapezoid_for_block(block, 
                                  block->entry_speed / block->nominal_speed,
                                  safe_speed / block->nominal_speed);

    /* Move buffer head */
    block_buffer_head = next_buffer_head;

    /* Update position */
    memcpy(position, target, sizeof(target));

    planner_recalculate();
}
/*
 * Set position
 * Used for G92 instructions.
 */
void plan_set_position(float x, float y, float z, const float e)
{
    position[X_AXIS] = lround(x * pa.axis_steps_per_unit[X_AXIS]); 
    position[Y_AXIS] = lround(y * pa.axis_steps_per_unit[Y_AXIS]); 
    position[Z_AXIS] = lround(z * pa.axis_steps_per_unit[Z_AXIS]); 
    position[E_AXIS] = lround(e * pa.axis_steps_per_unit[E_AXIS]); 

    /* Reset planner junction speeds. Assume start from rest */ 
    previous_nominal_speed = 0.0;
    previous_speed[0] = 0.0;
    previous_speed[1] = 0.0;
    previous_speed[2] = 0.0;
    previous_speed[3] = 0.0;
}

void plan_set_e_position(const float e)
{
    position[E_AXIS] = lround(e * pa.axis_steps_per_unit[E_AXIS]); 
}
/* 
 * Discard the current block 
 */
void plan_discard_current_block(void)
{
    if (block_buffer_head != block_buffer_tail) {
        block_buffer_tail = (block_buffer_tail + 1) & BLOCK_BUFFER_MASK;
    }
}
/* 
 * Get the current block, Return NULL if buffer empty 
 */
block_t *plan_get_current_block(void)
{
    if (block_buffer_head == block_buffer_tail) {
        return NULL;
    }
    block_t *block = &block_buffer[block_buffer_tail];
    block->busy = 1;
    return block;
}
/*
 * Get the current filled block size
 */
int plan_get_block_size(void)
{
    int queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE)
                       & (BLOCK_BUFFER_SIZE - 1);
    return queued;
}
/*
 * Stop Planner
 */
void plan_stop(void)
{
    if (!stop) {
        stop = true;
    }
}
/*
 * Delete the planner sub system
 */
void plan_exit(void)
{

}
