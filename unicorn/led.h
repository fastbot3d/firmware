/*
 * Unicorn 3D Printer Firmware
 * led.h
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
