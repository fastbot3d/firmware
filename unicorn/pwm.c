/*
 * Unicorn 3D Printer Firmware
 * pwm.c
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
    unsigned int period_ns;
    unsigned int duty_ns;
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

    PWM_DBG("pwm_index_lookup failed for '%s'\n",
                tag_name(pwm_channel));
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

    PWM_DBG("pwm_config called with %d records\n", nr_cfgs);
    
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
        //lkj for android pd->device_path = sys_path_finder(path, NAME_MAX, ps->device_path);
        pd->device_path = ps->device_path;
        pd->frequency   = ps->frequency;
        pd->state       = PWM_STATE_OFF;

        nr_pwms++;
    }

    return 0;
}

int pwm_init()
{
    int i;

    PWM_DBG("pwm_init called.\n");

    for (i = 0; i < nr_pwms; i++) {
        pwm_t *pd = &pwms[i];
        pd->state = PWM_STATE_OFF;

		PWM_DBG("pwm_init: request %s\n", pd->id);
		/* setup pwm channel */ 
        pwm_write_sysfs(pd->device_path, "request", 1);

    	if (bbp_board_type == BOARD_BBP1) {
            pwm_write_sysfs(pd->device_path, "polarity", 0);
        }

        if (pd->frequency) {
            pwm_write_sysfs(pd->device_path, "period_freq", pd->frequency);
        }
        pwm_write_sysfs(pd->device_path, "duty_percent", 0);
    }

    return 0;
}

void pwm_exit(void)
{
    int i;

    PWM_DBG("pwm_exit called.\n");

    for (i = 0; i < nr_pwms; i++) {
        pwm_t *pd = &pwms[i];
        pd->state = PWM_STATE_OFF;
		PWM_DBG("pwm_exit: turn off %s\n", pd->id);
		/* Turn off pwm */           
        pwm_write_sysfs(pd->device_path, "duty_percent", 0);        
        pwm_write_sysfs(pd->device_path, "run", 0);        
        pwm_write_sysfs(pd->device_path, "request", 0);
        //FIXME
        //lkj free((void *)pd->device_path);
    }
}

int pwm_set_period(channel_tag pwm_ch, unsigned int period_ns)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

	PWM_DBG("pwm_set_period called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];

    if (period_ns != pd->period_ns) {
        pd->period_ns = period_ns;
        PWM_DBG("pwm_set_period_ns: %s period_ns %d\n", 
                pd->id, period_ns);
        pwm_write_sysfs(pd->device_path, "period_ns", period_ns);
    }

    return 0;
}

int pwm_set_duty(channel_tag pwm_ch, unsigned int duty_ns)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

	PWM_DBG("pwm_set_duty called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];

    if (duty_ns != pd->duty_ns) {
        pd->duty_ns = duty_ns;
        PWM_DBG("pwm_set_duty_ns: %s duty_ns %d\n", pd->id, duty_ns);
        pwm_write_sysfs(pd->device_path, "duty_ns", duty_ns);
    }
    return 0;
}

int pwm_set_freq(channel_tag pwm_ch, unsigned int freq)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

	PWM_DBG("pwm_set_freq called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];

    if (pd->state == PWM_STATE_ON) {
        if (freq != pd->frequency) {
            pd->frequency = freq;
			PWM_DBG("pwm_set_freq: %s period_freq %d\n", 
					pd->id, freq);
			pwm_write_sysfs(pd->device_path, "period_freq", freq);
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

	PWM_DBG("pwm_set_output called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0 || percentage > 100) {
        return -1;
    }

    pd = &pwms[idx];

	if (bbp_board_type == BOARD_BBP1) {
		if (pd->state == PWM_STATE_ON) {
			PWM_DBG("pwm_set_output: %s duty_percent %d\n", 
					pd->id, percentage);
			pwm_write_sysfs(pd->device_path, "duty_percent", percentage);
		} else {
		    printf("pwm_set_output: pwm[%d] is already enable\n", idx);
		    return -1;
		}
	} else {
			PWM_DBG("pwm_set_output: %s duty_percent %d\n", 
					pd->id, percentage);
			pwm_write_sysfs(pd->device_path, "duty_percent", percentage);
	}

    return 0;
}

int pwm_enable(channel_tag pwm_ch)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

	PWM_DBG("pwm_enable called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }
    pd = &pwms[idx];
    pd->state = PWM_STATE_ON;

	PWM_DBG("pwm enable: turn on %s\n", pd->id);
	pwm_write_sysfs(pd->device_path, "run", 1);
    if (pd->frequency) {
        pwm_write_sysfs(pd->device_path, "period_freq", pd->frequency);
    }

    return 0;
}

int pwm_disable(channel_tag pwm_ch)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

	PWM_DBG("pwm_disable called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];
    pd->state = PWM_STATE_OFF;

	PWM_DBG("pwm_disable: turn off %s\n", pd->id);
	pwm_write_sysfs(pd->device_path, "run", 0);

    return 0;
}

int pwm_get_state(channel_tag pwm_ch)
{
    int idx = -1;    
    pwm_t *pd = NULL; 

	//PWM_DBG("pwm_get_state called.\n");

    idx = pwm_index_lookup(pwm_ch);
    if (idx < 0) {
        return -1;
    }

    pd = &pwms[idx];
    return pd->state;
}

