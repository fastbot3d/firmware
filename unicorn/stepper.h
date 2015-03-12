/*
 * Unicorn 3D Printer Firmware
 * stepper.h
 * Stepper motor control, which pops blocks from the block_buffer and
 * execute them by push motion command approopriately.
 *
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
#ifndef _STEPPER_H
#define _STEPPER_H

#include "common.h"
#include <stepper_spi.h>

#define STEPPER_SPI_DEV "/dev/stepper_spi"

typedef const struct {
    channel_tag  tag;
    channel_tag  vref;
} stepper_config_t;

#define ST_CMD_LMSW_CONFIG  (0x00)
#define ST_CMD_AXIS_HOMING  (0x01)
#define ST_CMD_SET_POS      (0x02)
#define ST_CMD_GET_POS      (0x03)
#define ST_CMD_PAUSE        (0x04)
#define ST_CMD_RESUME       (0x05)
#define ST_CMD_START        (0x06)
#define ST_CMD_STOP         (0x07)

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
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t e;
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

typedef union {
    uint32_t      gen[6];
    lmsw_config_t lmsw; 
    homing_t      homing;
    control_t     ctrl;
    pos_t         pos;
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

extern int stepper_enable(uint8_t axis);
extern int stepper_disable(uint8_t axis);

extern int stepper_reset(uint8_t axis);

extern int stepper_wake_up(uint8_t axis);
extern int stepper_sleep(uint8_t axis);

extern int stepper_update(void);

extern void stepper_sync(void);

extern int stepper_start(void);
extern int stepper_stop(void);

extern int stepper_pause(void);
extern int stepper_resume(void);

extern int stepper_homing_axis(uint8_t axis, uint8_t reverse);
extern int stepper_wait_for_lmsw(uint8_t axis);

extern void stepper_set_position(const long x, const long y, const long z, const long e);
extern void stepper_set_e_position(const long e);

extern uint32_t stepper_get_position(uint8_t axis);
extern float stepper_get_position_mm(uint8_t axis);

#if defined (__cplusplus)
}
#endif
#endif
