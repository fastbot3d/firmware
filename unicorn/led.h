/*
 * Unicorn 3D Printer Firmware
 * led.h
 * Led light brightness control interface
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
#ifndef _LED_H
#define _LED_H

#include "common.h"

typedef const struct {
    channel_tag  tag;
    channel_tag  pwm;
    unsigned int level;
} led_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int led_config(led_config_t *pcfgs, int nr_cfgs);

extern int led_init(void);
extern void led_exit(void);

extern int led_enable(channel_tag led);
extern int led_disable(channel_tag led);

extern int led_set_level(channel_tag led, unsigned int level);
extern int led_get_level(channel_tag led, unsigned int *level);

extern channel_tag led_lookup_by_name(const char *name);

#if defined (__cplusplus)
}
#endif
#endif
