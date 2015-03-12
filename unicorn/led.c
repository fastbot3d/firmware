/*
 * Unicorn 3D Printer Firmware
 * led.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pwm.h"
#include "led.h"

typedef struct {
    channel_tag  id;
    channel_tag  pwm;
    unsigned int level;
} led_t;

static led_t *leds = NULL;
static unsigned int nr_leds = 0;

static int led_index_lookup(channel_tag led)
{
    int idx;

    for (idx = 0; idx < nr_leds; idx++) {
        if (leds[idx].id == led) {
            return idx;
        }
    }

    if (DBG(D_LED)) {
        fprintf(stderr, "led_index_lookup failed for '%s'\n",
                tag_name(led));
    }
    return -1;
}

channel_tag led_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_leds; idx++) {
        channel_tag tag = leds[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

int led_config(led_config_t *pcfgs, int nr_cfgs)
{
    int i;

    if (DBG(D_LED)) {
        printf("led_config called with %d records\n", nr_cfgs);
    }
    
    leds = calloc(nr_cfgs, sizeof(led_t));
    if (!leds) {
        return -1;
    }

    nr_leds = 0;

    for (i = 0; i < nr_cfgs; i++) {
        led_t *pd = &leds[i];
        led_config_t *ps = &pcfgs[i];
        
        pd->pwm = pwm_lookup_by_name(ps->pwm);
        if (!pd->pwm) {
            return -1;
        }
        pd->id    = ps->tag;  
        pd->level = ps->level; 
        
        nr_leds++;

        pwm_set_output(pd->pwm, pd->level);
    }

    return 0;
}

int led_init(void)
{
    if (DBG(D_LED)) {
        printf("led_init called.\n");
    }
    
    if (!leds || nr_leds <= 0) {
        return -1;
    }

    return 0;
}

void led_exit(void)
{
    if (DBG(D_LED)) {
        printf("led_exit called.\n");
    }
    
    if (leds) {
        int i;
        for (i = 0; i < nr_leds; i++) { 
            led_t *pd = &leds[i];
            if (pwm_get_state(pd->pwm) == PWM_STATE_ON) {
                pwm_set_output(pd->pwm, 0);
                pwm_disable(pd->pwm);
            }
        }

        free(leds);
    }
}
/*
 * Turn on led, set it to default or previous brightness
 */
int led_enable(channel_tag led)
{
    int idx = -1;
    led_t *pd = NULL;

    if (DBG(D_LED)) {
        printf("led_enable: %s\n", led);
    }

    idx = led_index_lookup(led);
    if (idx < 0) {
        return -1;
    }

    pd = &leds[idx];
    
    if (pwm_get_state(pd->pwm) == PWM_STATE_OFF) {
        pwm_enable(pd->pwm); 

        if (pd->level > 0 && pd->level <= 100) {
            pwm_set_output(pd->pwm, pd->level);
        }
    }

    return 0;
}
/*
 * Turn off led, set it's brightness to zero
 */
int led_disable(channel_tag led)
{
    int idx = -1;
    led_t *pd = NULL;

    if (DBG(D_LED)) {
        printf("led_disable: %s\n", led);
    }

    idx = led_index_lookup(led);
    if (idx < 0) {
        return -1;
    }

    pd = &leds[idx];
    
    if (pwm_get_state(pd->pwm) == PWM_STATE_ON) {
        pwm_set_output(pd->pwm, 0);
        pwm_disable(pd->pwm); 
    }

    return 0;
}
/*
 * Set led level, [0, 100]
 */
int led_set_level(channel_tag led, unsigned int level)
{
    int idx = -1;
    led_t *pd = NULL;

    if (DBG(D_LED)) {
        printf("led_set_level: %s -> %d\n", led, level);
    }

    idx = led_index_lookup(led);
    if (idx < 0 || level > 100) {
        return -1;
    }

    pd = &leds[idx];
    if (pd->level != level) {
        pd->level = level;
        pwm_set_output(pd->pwm, pd->level);
    } else {
        if (DBG(D_LED)) {
            printf("%s is already at level %d\n", led, level);
        }
    }

    return 0;
}
/*
 * Return led level
 */
int led_get_level(channel_tag led, unsigned int *level)
{
    int idx = -1;
    led_t *pd = NULL;

    idx = led_index_lookup(led);
    if (idx < 0) {
        return -1;
    }

    pd = &leds[idx];
    *level = pd->level;

    if (DBG(D_LED)) {
        printf("led_get_level: %s -> %d\n", led, *level);
    }

    return 0;
}
