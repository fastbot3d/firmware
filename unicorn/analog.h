/*
 * Unicorn 3D Printer Firmware
 * analog.h
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
