/*
 * Unicorn 3D Printer Firmware
 * servo.c
 * servo control interface
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
#include "servo.h"

/*
 * pwm control signal: freq  47.62HZ ~ 45.45HZ
 *                     high 1000us ~ 2000us
 *                     low  20000us
 */
#define SERVO_MAX_FREQ      (47.62)
#define SERVO_MIN_FREQ      (45.45)

#define SERVO_MAX_PERIOD_NS (22000000)
#define SERVO_MIN_PERIOD_NS (21000000)

#define SERVO_MAX_DUTY_NS  (2000000)
#define SERVO_MIN_DUTY_NS  (1000000)

#define SERVO_ANGLE_RANGE   (90)

typedef struct {
    channel_tag  id;
    channel_tag  pwm;
    unsigned int angle;
} servo_t;

static servo_t *servos = NULL;
static unsigned int nr_servos = 0;

static int servo_index_lookup(channel_tag servo)
{
    int idx;

    for (idx = 0; idx < nr_servos; idx++) {
        if (servos[idx].id == servo) {
            return idx;
        }
    }

    printf("servo_index_lookup failed for '%s'\n",
                tag_name(servo));
    return -1;
}

channel_tag servo_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_servos; idx++) {
        channel_tag tag = servos[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

channel_tag servo_lookup_by_index(int idx) 
{
    if (idx >= 0 || idx < nr_servos) {
        return servos[idx].id; 
    } else {
        return NULL;
    }
}

int servo_config(servo_config_t *pcfgs, int nr_cfgs)
{
    int i;

    SERVO_DBG("servo_config called with %d records\n", nr_cfgs);
    
    if (!pcfgs || nr_cfgs <= 0) {
        return -1;
    }

    servos = calloc(nr_cfgs, sizeof(servo_t));
    if (!servos) {
        return -1;
    }

    nr_servos = 0;

    for (i = 0; i < nr_cfgs; i++) {
        servo_t *pd = &servos[i];
        servo_config_t *ps = &pcfgs[i];
        
        pd->id    = ps->tag;  
        pd->pwm   = ps->pwm;
        pd->angle = ps->angle; 
        
        nr_servos++;
    }

    return 0;
}

int servo_init(void)
{
    int i;
    float duty = 0;
    float period = 0;

    SERVO_DBG("servo_init called.\n");
    
    if (!servos || nr_servos <= 0) {
        printf("[servo]: no servo configed\n");
        return -1;
    }

    for (i = 0; i < nr_servos; i++) {
        servo_t *pd = &servos[i];

        /* Enable and set servo to default angle */
        if (pd->pwm) {
            if (pd->angle > 0 && pd->angle < 180) {
                duty = SERVO_MIN_DUTY_NS + (float)(pd->angle) 
                       * (SERVO_MAX_DUTY_NS - SERVO_MIN_DUTY_NS) / SERVO_ANGLE_RANGE;
                period = duty + 20000000; 

                pwm_enable(pd->pwm);
                pwm_set_duty(pd->pwm, (uint32_t)duty);
                pwm_set_period(pd->pwm, (uint32_t)period);

                SERVO_DBG("servo pwm period_ns %f, duty_ns %f\n",
                          period, duty);
            } else {
                printf("[servo]: servo_init invalid default angle : %d\n", pd->angle);
            }
        } else {
            printf("[servo]: no pwm configed to servo %d\n", i);
        }
    }

    return 0;
}

void servo_exit(void)
{
	SERVO_DBG("servo_exit called.\n");

    if (servos) {
        int i;
        for (i = 0; i < nr_servos; i++) { 
            servo_t *pd = &servos[i];
            /* Disable all servos */
            if (pd->pwm) {
                if (pwm_get_state(pd->pwm) == PWM_STATE_ON) {
                    pwm_set_output(pd->pwm, 0);
                    pwm_disable(pd->pwm);
                }
            }
        }

        free(servos);
    }
}
/*
 * Set servo angle, [0, 180]
 */
int servo_set_angle(channel_tag servo, unsigned int angle)
{
    int idx = -1;
    float duty = 0;
    float period = 0;
    servo_t *pd = NULL;

	SERVO_DBG("servo_set_angle: %s -> %d\n", servo, angle);

    idx = servo_index_lookup(servo);
    if (idx < 0 || angle < 0 || angle > 180) {
        printf("servo %d not found, or invalid angle value %d\n", 
                idx, angle);
        return -1;
    }
    pd = &servos[idx];

    if (pd->angle != angle) {
        pd->angle = angle;
        /* Change pwm freq and duty_percent */
        if (pd->pwm) {
            
            duty = SERVO_MIN_DUTY_NS + (float)(pd->angle) * (SERVO_MAX_DUTY_NS - SERVO_MIN_DUTY_NS) / SERVO_ANGLE_RANGE;
            period = duty + 20000000; 

            pwm_set_duty(pd->pwm, (uint32_t)duty);
            pwm_set_period(pd->pwm, (uint32_t)period);

			SERVO_DBG("servo pwm period_ns %f, duty_ns %f\n",
					  period, duty);

            /* If pwm not enabled, enable it */
            if (pwm_get_state(pd->pwm) == PWM_STATE_OFF) {
                pwm_enable(pd->pwm); 
            }
        }
    } else {
		SERVO_DBG("%s is already at pos %d\n", servo, angle);
	}

    return 0;
}
/*
 * Return servo angle
 */
int servo_get_angle(channel_tag servo, unsigned int *angle)
{
    int idx = -1;
    servo_t *pd = NULL;

    idx = servo_index_lookup(servo);
    if (idx < 0) {
        printf("[servo]: lookup %s err\n", servo);
        return -1;
    }

    pd = &servos[idx];
    *angle = pd->angle;

	SERVO_DBG("servo_get_angle: %s -> %d\n", servo, *angle);

    return 0;
}
/*
 * Turn on servo, set it to default angle or previous angle
 */
int servo_enable(channel_tag servo)
{
    int idx = -1;
    servo_t *pd = NULL;

	SERVO_DBG("servo_enable: %s\n", servo);

    idx = servo_index_lookup(servo);
    if (idx < 0) {
        return -1;
    }

    pd = &servos[idx];
    if (pd->pwm) { 
        //if (pwm_get_state(pd->pwm) == PWM_STATE_OFF) {
            pwm_enable(pd->pwm); 
        //}
    }

    return 0;
}
/*
 * Turn off servo, set it's angle to zero
 */
int servo_disable(channel_tag servo)
{
    int idx = -1;
    servo_t *pd = NULL;

	SERVO_DBG("servo_disable: %s\n", servo);

    idx = servo_index_lookup(servo);
    if (idx < 0) {
        return -1;
    }

    pd = &servos[idx];

    if (pd->pwm) { 
        if (pwm_get_state(pd->pwm) == PWM_STATE_ON) {
            pwm_set_output(pd->pwm, 0);
            pwm_disable(pd->pwm); 
        }
    }

    return 0;
}
