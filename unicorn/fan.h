/*
 * Unicorn 3D Printer Firmware
 * fan.h
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
#ifndef _FAN_H
#define _FAN_H

#include "common.h"

typedef const struct {
    channel_tag  tag;
    channel_tag  pwm;
    unsigned int level;
} fan_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int fan_config(fan_config_t *pcfgs, int nr_cfgs);
extern int fan_init(void);
extern void fan_exit(void);

extern channel_tag fan_lookup_by_name(const char *name);

extern int fan_enable(channel_tag fan);
extern int fan_disable(channel_tag fan);

extern int fan_set_level(channel_tag fan, unsigned int level);
extern int fan_get_level(channel_tag fan, unsigned int *level);

#if defined (__cplusplus)
}
#endif
#endif
