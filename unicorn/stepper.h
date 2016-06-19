/*
 * Unicorn 3D Printer Firmware
 * stepper.h
*/
#ifndef _STEPPER_H
#define _STEPPER_H

#include "common.h"
#include <stepper_spi.h>

#define STEPPER_SPI_DEV "/dev/stepper_spi"

typedef const struct {
    channel_tag  tag;
    channel_tag  vref;
} stepper_config_t;

#define ST_CMD_LMSW_CONFIG   (0x00)
#define ST_CMD_AXIS_HOMING   (0x01)
#define ST_CMD_SET_POS       (0x02)
#define ST_CMD_GET_POS       (0x03)
#define ST_CMD_PAUSE         (0x04)
#define ST_CMD_RESUME        (0x05)
#define ST_CMD_START         (0x06)
#define ST_CMD_STOP          (0x07)
#define ST_CMD_TEST          (0x08)
#define ST_CMD_SET_PAUSE_POS (0x09)
#define ST_CMD_IDLE			 (0x0a)
#define ST_CMD_GET_PRU_STATE			 (13)
#define ST_CMD_SET_PRU_STATE			 (14)
#define ST_CMD_SET_CANCEL_Z_UP_POS       (15)
#define ST_CMD_GET_CANCEL_Z_UP_POS      (16)


typedef struct {
    uint32_t cmd;
    uint32_t axis;
    uint32_t min_gpio; 
    uint32_t min_invert;
    uint32_t max_gpio;
    uint32_t max_invert;
} lmsw_config_t;

typedef struct {
    uint32_t cmd;
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t e;
} pos_t;

typedef struct {
    uint32_t cmd;
} control_t;

typedef struct {
    uint32_t cmd;
    uint8_t  axis;
    uint32_t dir;
    uint32_t speed;
} homing_t;

typedef struct {
    uint32_t cmd;
    uint8_t  axis;
    uint32_t dir;
    uint32_t speed;
} test_t;

typedef struct {
    uint32_t cmd;
    uint32_t  state;
} current_state;


typedef union {
    uint32_t      gen[6];
    lmsw_config_t lmsw; 
    homing_t      homing;
    control_t     ctrl;
    pos_t         pos;
    test_t        test;
    current_state state;
} st_cmd_t;


#if defined (__cplusplus)
extern "C" {
#endif

extern int  stepper_config(stepper_config_t *pcfgs, int nr_cfgs);

extern int  stepper_init(void);
extern void stepper_exit(int blocking);

extern int stepper_set_microstep(uint8_t axis, uint8_t steps);
extern int stepper_get_microstep(uint8_t axis, uint8_t *steps);

extern int stepper_set_current(uint8_t axis, uint32_t current);

extern void stepper_enable_drivers(void);
extern void stepper_disable_drivers(void);

extern int stepper_enable(uint8_t axis);
extern int stepper_disable(uint8_t axis);

extern int stepper_reset(uint8_t axis);

extern int stepper_wake_up(uint8_t axis);
extern int stepper_sleep(uint8_t axis);

extern int stepper_update(void);

extern void stepper_sync(void);

extern int stepper_start(void);

extern bool stepper_is_stop();
extern bool motor_is_disable();
#if 1
extern int stepper_stop(bool blocking);
#else
extern int stepper_stop(void);
#endif
extern int stepper_idle(void);
extern int stepper_clean_up_fifo(void);
extern int stepper_reset_fifo(void);

extern int stepper_pause(void);
extern int stepper_resume(void);
extern int stepper_test(void);

extern int stepper_homing_axis(uint8_t axis);
extern int get_homing_dir();
extern int stepper_check_lmsw(uint8_t axis);
extern int stepper_wait_for_lmsw(uint8_t axis);
extern int stepper_wait_for_autoLevel(void);
extern int stepper_autoLevel_gpio_turn(bool on);

extern void stepper_set_position(const long x, const long y, const long z, const long e);
extern void stepper_set_e_position(const long e);
extern void stepper_set_pause_position(const long x, const long y, const long z);

extern uint32_t stepper_get_position(uint8_t axis);
extern float stepper_get_position_mm(uint8_t axis);

extern void stepper_parameter_update(void);

extern int stepper_get_queue_len(void);
extern void stepper_load_filament(int cmd); //1, upload , 2 unload , 3 pause 

extern int stepper_config_lmsw(uint8_t axis, bool high); //default active low 

#if defined (__cplusplus)
}
#endif
#endif
