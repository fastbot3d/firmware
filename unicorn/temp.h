/*
 * Unicorn 3D Printer Firmware
 * temp.h
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
#ifndef _TEMP_H
#define _TEMP_H

#include "common.h"

typedef int (temp_convert_f) (int in, double *out);

typedef const struct {
    channel_tag    tag;
    channel_tag    analog_input;
    int            in_range_time;
    temp_convert_f *convert; 
} temp_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int temp_config(temp_config_t *pcfgs, int nr_cfgs);

extern int temp_init(void);
extern void temp_exit(void);

extern int temp_get_celsius(channel_tag temp_ch, double *value);
extern int temp_set_setpoint(channel_tag temp_ch, 
                             double setpoint, 
                             double delta_low, 
                             double delta_high);

extern int temp_achieved(channel_tag temp_ch);

#if defined (__cplusplus)
}
#endif
#endif
