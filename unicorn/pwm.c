/*
 * Unicorn 3D Printer Firmware
 * pwm.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <stdint.h>

#include "pwm.h"
#include "common.h"

typedef struct {
    channel_tag  id;
    const char   *device_path;
    int          state;
    unsigned int frequency;
    unsigned int duty;
    unsigned int period;
} pwm_t;

static pwm_t *pwms = NULL;
static int nr_pwms = 0;

static int pwm_index_lookup(channel_tag pwm_channel)
{
    int idx;

    for (idx = 0; idx < nr_pwms; idx++) {
        if (pwms[idx].id == pwm_channel) {
            return idx;
        }
    }

    if DBG(D_PWM) {
        fprintf(stderr, "pwm_index_lookup failed for '%s'\n",
                tag_name(pwm_channel));
    }
    return -1;
}

channel_tag pwm_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_pwms; idx++) {
        channel_tag tag = pwms[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

int pwm_config(pwm_config_t *pcfgs, int nr_cfgs)
{
    int i;

    if DBG(D_PWM) {
        printf("pwm_config called with %d records\n", nr_cfgs);
    }
    
    pwms = calloc(nr_cfgs, sizeof(pwm_t));
    if (!pwms) {
        return -1;
    }
    nr_pwms = 0;
    
    for (i = 0; i < nr_cfgs; i++) {
        pwm_t        *pd = &pwms[i];
        pwm_config_t *ps = &pcfgs[i];

        char *path = malloc(NAME_MAX);
        if (!path) {
            return -1;
        }

        pd->id          = ps->tag; 
#ifdef HOST
        pd->device_path = "test";
#else
        pd->device_path = sys_path_finder(path, NAME_MAX, ps->device_path);
#endif
        pd->frequency   = ps->frequency;
        pd->state       = PWM_STATE_OFF;

        nr_pwms++;
    }

    return 0;
}

int pwm_init(void)
{
    int i;
    if DBG(D_PWM) {
        printf("pwm_init called.\n");
    }

    for (i = 0; i < nr_pwms; i++) {
        pwm_t *pd = &pwms[i];

#ifdef HOST
        if DBG(D_PWM) {
            printf("pwm_init: request %s\n", pd->id);
        }
#else
        /* setup pwm channel */ 
        pwm_write_sysfs(pd->device_path, "request", 1);
        pwm_write_sysfs(pd->device_path, "polarity", 0);
        pwm_write_sysfs(pd->device_path, "duty_percent", 0);
        if (pd->frequency) {
            pwm_write_sysfs(pd->device_path, "period_freq", pd->frequency);
        }
#endif
    }

    return 0;
}

void pwm_exit(void)
{
    int i;

    if DBG(D_PWM) {
        printf("pwm_exit called.\n");
    }

    for (i = 0; i < nr_pwms; i++) {
        pwm_t *pd = &pwms[i];
        pd->state = PWM_STATE_OFF;
#ifdef HOST
        if DBG(D_PWM) {
            printf("pwm_exit: turn off %s\n", pd->id);
        }
#else
        /* Turn off pwm */           
        pwm_write_sysfs(pd->device_path, "duty_percent", 0);        
        pwm_write_sysfs(pd->device_path, "run", 0);        
        pwm_write_sysfs(pd->device_path, "request", 0);
#endif
        free((void *)pd->device_path);
    }
}

int pwm_set_freq(channel_tag pwm_ch, unsigned int freq)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

    if DBG(D_PWM) {
        printf("pwm_set_freq called.\n");
    }

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];

    if (pd->state == PWM_STATE_ON) {
        if (freq != pd->frequency) {
            pd->frequency = freq;
            #ifdef HOST
            if (DBG(D_PWM)) {
                printf("pwm_set_freq: %s period_freq %d\n", 
                        pd->id, freq);
            }
            #else
            pwm_write_sysfs(pd->device_path, "period_freq", freq);
            #endif
        }
    } else {
        printf("pwm_set_freq: pwm[%d] not enabled\n", idx);
        return -1;
    }

    return 0;
}

int pwm_set_output(channel_tag pwm_ch, unsigned int percentage)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

    if DBG(D_PWM) {
        printf("pwm_set_output called.\n");
    }

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0 || percentage > 100) {
        return -1;
    }

    pd = &pwms[idx];

    if (pd->state == PWM_STATE_ON) {
        #ifdef HOST
        if (DBG(D_PWM)) {
            printf("pwm_set_output: %s duty_percent %d\n", 
                    pd->id, percentage);
        }
        #else
        pwm_write_sysfs(pd->device_path, "duty_percent", percentage);
        #endif
    } else {
        printf("pwm_set_output: pwm[%d] not enabled\n", idx);
        return -1;
    }
    return 0;
}

int pwm_enable(channel_tag pwm_ch)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

    if (DBG(D_PWM)) {
        printf("pwm_enable called.\n");
    }

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }
    pd = &pwms[idx];
    pd->state = PWM_STATE_ON;

#ifdef HOST
    if (DBG(D_PWM)) {
        printf("pwm enable: turn on %s\n", pd->id);
    }
#else
    pwm_write_sysfs(pd->device_path, "run", 1);
    if (pd->frequency) {
        pwm_write_sysfs(pd->device_path, "period_freq", pd->frequency);
    }
#endif

    return 0;
}

int pwm_disable(channel_tag pwm_ch)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

    if DBG(D_PWM) {
        printf("pwm_disable called.\n");
    }

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];
    pd->state = PWM_STATE_OFF;

#ifdef HOST
    if DBG(D_PWM) {
        printf("pwm_disable: turn off %s\n", pd->id);
    }
#else
    pwm_write_sysfs(pd->device_path, "run", 0);
#endif

    return 0;
}

int pwm_get_state(channel_tag pwm_ch)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

    if DBG(D_PWM) {
        printf("pwm_get_state called.\n");
    }

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];
    return pd->state;
}

