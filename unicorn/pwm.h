/*
 * Unicorn 3D Printer Firmware
 * pwm.h
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
#ifndef _PWM_H
#define _PWM_H

#include "common.h"

typedef const struct {
    channel_tag  tag;
    unsigned int frequency;
    const char   *device_path;
} pwm_config_t;

#define PWM_STATE_ON  0x0011
#define PWM_STATE_OFF 0x1100

#if defined (__cplusplus)
extern "C" {
#endif

extern int pwm_config(pwm_config_t *pcfgs, int nr_cfgs);

extern int  pwm_init(void);
extern void pwm_exit(void);

extern int pwm_enable(channel_tag pwm_ch);
extern int pwm_disable(channel_tag pwm_ch);

extern int pwm_set_output(channel_tag pwm_ch, unsigned int percentage);
extern int pwm_set_freq(channel_tag pwm_ch, unsigned int freq);

extern int pwm_get_state(channel_tag pwm_ch);

extern channel_tag pwm_lookup_by_name(const char *name);

#if defined (__cplusplus)
}
#endif
#endif
