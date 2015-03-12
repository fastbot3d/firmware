/*
 * Unicorn 3D Printer Firmware
 * analog.h
 * Analog input measure and output adjustment interface
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
#ifndef _ANALOG_H
#define _ANALOG_H

#include "common.h"

#define ANALOG_TYPE_IN    (0)
#define ANALOG_TYPE_OUT   (1)

typedef const struct {
    channel_tag  tag;
    channel_tag  device_path;
    unsigned int type;
} analog_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int analog_config(analog_config_t *pcfgs, int nr_cfgs);

extern int analog_init(void);
extern void analog_exit(void);

extern int analog_get_input(channel_tag ain, int *vol);
extern int analog_set_output(channel_tag aout, int vol);

channel_tag analog_lookup_by_name(const char *name);

#if defined (__cplusplus)
}
#endif
#endif
