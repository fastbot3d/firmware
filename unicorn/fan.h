/*
 * Unicorn 3D Printer Firmware
 * fan.h
 */
#ifndef _FAN_H
#define _FAN_H

#include "common.h"

typedef const struct {
    channel_tag  tag;
    channel_tag  pwm;
    unsigned int level;
} fan_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int fan_config(fan_config_t *pcfgs, int nr_cfgs);
extern int fan_init(void);
extern void fan_exit(void);

extern channel_tag fan_lookup_by_name(const char *name);

extern int fan_enable(channel_tag fan);
extern int fan_disable(channel_tag fan);

extern int fan_set_level(channel_tag fan, unsigned int level);
extern int fan_get_level(channel_tag fan, unsigned int *level);

#if defined (__cplusplus)
}
#endif
#endif
