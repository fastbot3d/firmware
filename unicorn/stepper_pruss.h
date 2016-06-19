/*
 * Unicorn 3D Printer Firmware
 * stepper_pruss.h
*/
#ifndef _STEPPER_PRUSS_H
#define _STEPPER_PRUSS_H

#include <stepper_spi.h>

#include "planner.h"
#include "common.h"
#include "stepper.h"

#define STATE_EMPTY    (0)
#define STATE_FILLED   (1)
#define STATE_EXIT     (2)

#define STATE_IDLE     (3)
#define STATE_PAUSE    (4)
#define STATE_HOME     (5)
#define STATE_PRINT    (6)
#define STATE_STOP     (7)
#define STATE_RESUME   (8)
#define STATE_TEST     (9)
#define STATE_PAUSE_FINISH     (10)
#define STATE_LOAD_FILAMENT  (11)
#define STATE_UNLOAD_FILAMENT  (12)

//#define QUEUE_LEN      (320)
//#define QUEUE_LEN      (1024)
//#define QUEUE_LEN      (3072)
//lkj #define QUEUE_LEN      (4096)
//#define QUEUE_LEN      (8192)
//#define QUEUE_LEN      (12288)
#define QUEUE_LEN      (15360)
//#define QUEUE_LEN      (16384)

#define MOTOR56_MODE_EXTRUDER   0
#define MOTOR56_MODE_DUAL_X_Y   1

struct queue_element {
    /* queue header */
    uint8_t  state;
    uint8_t  direction_bits;
    uint8_t  direction;
    uint8_t  type; 

    /* Travel Parameters */
    uint32_t loops_accel;    /* Phase 1: loops spent in acceleration */
    uint32_t loops_travel;   /* Phase 2: loops spent in travel */
    uint32_t loops_decel;    /* Phase 3: loops spent in deceleration */
    uint32_t steps_count;    /* Max step event count */

    uint32_t accel_cycles;   /* acceleration delay cycles */ 
    uint32_t travel_cycles;  /* travel delay cycles */
    uint32_t decel_cycles;   /* decelerate delay cycles */
    
    uint32_t init_cycles;    /* init enter entry delay cycles */
    //uint32_t final_cycles;   /* final exit entry delay cycles */

    uint32_t steps_x;        /* fraction of steps count of axis x */
    uint32_t steps_y;        /* fraction of steps count of axis y */
    uint32_t steps_z;        /* fraction of steps count of axis z */
    uint32_t steps_e;        /* fraction steps count of axis e */

 // uint32_t ext_step_ctl_gpio;     // Selects the active extruder step control gpio out
    uint8_t ext_step_bit;          // bit 0-2 dir e0, e1, e2, 3-5 active extruder E0, E1, E2  
    uint8_t ext_reserved_1;         
    uint8_t ext_reserved_2;              
    uint8_t ext_reserved_3;              

    //.u32 ext_step_dir_gpio;     
    uint8_t  reserved_1;         
    uint8_t  reserved_2;         
    uint8_t  reserved_3;         
    uint8_t  reserved_4;         

    uint8_t  reserved_5;         //ext_step_ctl_offset;     
    uint8_t  reserved_6;         //ext_step_dir_offset;  
	uint8_t  reserved_7;         
    uint8_t  reserved_8; 
} __attribute__((packed));

struct active_extruder_gpio {
    uint32_t ext_step_ctl_gpio;     
    uint32_t ext_step_dir_gpio;     
    uint8_t  ext_step_ctl_offset;
    uint8_t  ext_step_dir_offset;
}; 

struct queue {
    volatile struct queue_element ring_buf[QUEUE_LEN];
    volatile uint32_t state;

    volatile uint32_t homing_axis;
    volatile uint32_t homing_dir;
    volatile uint32_t homing_time;

    volatile int32_t pos_x;
    volatile int32_t pos_y; 
    volatile int32_t pos_z;
    volatile int32_t pos_e;

    volatile uint32_t pause_x;
    volatile uint32_t pause_y; 
    volatile uint32_t pause_z;
    
    volatile uint32_t write_pos;

    volatile uint8_t machine_type;
    volatile uint8_t bbp1_extend_func;  //only for bbp1 extend fifth socket. option, dual z, dual extruder
    volatile uint8_t reserved_1;
    volatile uint8_t reserved_2;

    volatile uint32_t mcode_count;
    volatile uint32_t pause_z_distance_steps; //offset 56, z move down distance
    volatile uint32_t read_pos_pru; //offset 60, pru read_pos offset, bytes like write_pos
    volatile uint32_t cancel_z_up_steps; //offset 64, z move up distance
    volatile uint32_t motor56_mode; //offset 68
    volatile uint32_t invert_endstop; //offset 72, bit0-7 invert x,y,z,autolevel;
};

/*
 * Initialize pruss motor control
 */
extern int pruss_stepper_init(void);
/*
 * Shut down pruss motor control, wait for queue to all empty.
 */
extern void pruss_stepper_exit(void);

extern int pruss_stepper_start(void);
extern void pruss_stepper_stop(void);

extern int pruss_queue_move(block_t *block);

extern int pruss_send_cmd(st_cmd_t *cmd);

extern int pruss_queue_wait(void);
extern void pruss_queue_terminate_wait(int exit);
extern int pruss_queue_is_full(void);
extern int pruss_queue_get_max_rate(void);
extern int pruss_queue_get_len(void);
extern void pruss_stepper_parameter_update(void);
#if defined (__cplusplus)
}
#endif
#endif
