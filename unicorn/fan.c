/*
 * Unicorn 3D Printer Firmware
 * fan.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pwm.h"
#include "fan.h"

typedef struct {
    channel_tag  id;
    channel_tag  pwm;
    unsigned int gpio;
    unsigned int level;
} fan_t;

static fan_t *fans = NULL;
static unsigned int nr_fans = 0;

static int fan_index_lookup(channel_tag fan)
{
    int idx;

    for (idx = 0; idx < nr_fans; idx++) {
        if (fans[idx].id == fan) {
            return idx;
        }
    }

	printf("fan_index_lookup failed for '%s'\n",
			tag_name(fan));
    return -1;
}

channel_tag fan_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_fans; idx++) {
        channel_tag tag = fans[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

channel_tag fan_lookup_by_index(int idx) 
{
    if (idx >= 0 || idx < nr_fans) {
        return fans[idx].id; 
    } else {
        return NULL;
    }
}

int fan_config(fan_config_t *pcfgs, int nr_cfgs)
{
    int i;

    FAN_DBG("fan_config called with %d records\n", nr_cfgs);
    
    if (!pcfgs || nr_cfgs <= 0) {
        return -1;
    }

    fans = calloc(nr_cfgs, sizeof(fan_t));
    if (!fans) {
        return -1;
    }

    nr_fans = 0;

    for (i = 0; i < nr_cfgs; i++) {
        fan_t *pd = &fans[i];
        fan_config_t *ps = &pcfgs[i];
        
        pd->id    = ps->tag;  
        pd->pwm   = ps->pwm;
        pd->gpio  = ps->gpio;
        pd->level = ps->level; 
        
        nr_fans++;
    }

    return 0;
}

int fan_init(void)
{
    int i;

    FAN_DBG("fan_init called.\n");
    
    if (!fans || nr_fans <= 0) {
        return -1;
    }

    for (i = 0; i < nr_fans; i++) {
        fan_t *pd = &fans[i];

        if (pd->gpio) { 
			FAN_DBG("%s request gpio %d\n", pd->id, pd->gpio);

            gpio_request_sysfs(pd->gpio);
            gpio_write_sysfs(pd->gpio, "direction", "out");
            gpio_write_sysfs(pd->gpio, "value", "0");
        } 
    }

    return 0;
}

void fan_exit(void)
{
    FAN_DBG("fan_exit called.\n");
    
    if (fans) {
        int i;
        for (i = 0; i < nr_fans; i++) { 
            fan_t *pd = &fans[i];
            if (pd->pwm) {
                if (pwm_get_state(pd->pwm) == PWM_STATE_ON) {
                    pwm_set_output(pd->pwm, 0);
                    pwm_disable(pd->pwm);
                }
            }

            if (pd->gpio) {
				FAN_DBG("%s, free gpio %d\n", pd->id, pd->gpio);
				gpio_write_sysfs(pd->gpio, "value", "0");
                gpio_free_sysfs(pd->gpio);
            }
        }

        free(fans);
    }
}
/*
 * Turn on fan, set it to default level or previous level
 */
int fan_enable(channel_tag fan)
{
    int idx = -1;
    fan_t *pd = NULL;

	FAN_DBG("fan_enable: %s\n", fan);

    idx = fan_index_lookup(fan);
    if (idx < 0) {
        return -1;
    }

    pd = &fans[idx];
    if (pd->pwm) { 
        if (pwm_get_state(pd->pwm) == PWM_STATE_OFF) {
            pwm_enable(pd->pwm); 
            if (pd->level > 0 && pd->level <= 100) {
                pwm_set_output(pd->pwm, pd->level);
            }
        }
    }

    if (pd->gpio) {
        gpio_write_sysfs(pd->gpio, "value", "1");
    }

    return 0;
}
/*
 * Turn off fan, set it's level to zero
 */
int fan_disable(channel_tag fan)
{
    int idx = -1;
    fan_t *pd = NULL;

	FAN_DBG("fan_disable: %s\n", fan);

    idx = fan_index_lookup(fan);
    if (idx < 0) {
        return -1;
    }

    pd = &fans[idx];

    if (pd->pwm) { 
        if (pwm_get_state(pd->pwm) == PWM_STATE_ON) {
            pwm_set_output(pd->pwm, 0);
            pwm_disable(pd->pwm); 
        }
    }

    if (pd->gpio) {
        gpio_write_sysfs(pd->gpio, "value", "0");
    }

    return 0;
}
/*
 * Set fan output level, [0, 100]
 */
int fan_set_level(channel_tag fan, unsigned int level)
{
    int idx = -1;
    fan_t *pd = NULL;

	FAN_DBG("fan_set_level: %s -> %d\n", fan, level);

    idx = fan_index_lookup(fan);
    if (idx < 0 || level > 100) {
        return -1;
    }

    pd = &fans[idx];

    if (pd->level != level) {
        pd->level = level;
        if (pd->pwm) {
            pwm_set_output(pd->pwm, pd->level);
        }
    } else {
		FAN_DBG("%s is already at level %d\n", fan, level);
	}

    return 0;
}
/*
 * Return fan output level
 */
int fan_get_level(channel_tag fan, unsigned int *level)
{
    int idx = -1;
    fan_t *pd = NULL;

    idx = fan_index_lookup(fan);
    if (idx < 0) {
        printf("[Fan]: lookup %s err\n", fan);
        return -1;
    }

    pd = &fans[idx];
    *level = pd->level;

	FAN_DBG("fan_get_level: %s -> %d\n", fan, *level);

    return 0;
}
