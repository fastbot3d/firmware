/*
 * Unicorn 3D Printer Firmware
 * servo.h
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
#ifndef _SERVO_H
#define _SERVO_H

#include "common.h"

typedef const struct {
    channel_tag  tag;
    channel_tag  pwm;
    unsigned int angle;
} servo_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int servo_config(servo_config_t *pcfgs, int nr_cfgs);
extern int servo_init(void);
extern void servo_exit(void);

extern channel_tag servo_lookup_by_name(const char *name);
extern channel_tag servo_lookup_by_index(int idx);

extern int servo_enable(channel_tag servo);
extern int servo_disable(channel_tag servo);

extern int servo_set_angle(channel_tag servo, unsigned int angle);
extern int servo_get_angle(channel_tag servo, unsigned int *angle);

#if defined (__cplusplus)
}
#endif
#endif
