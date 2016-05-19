/*
 * Unicorn 3D Printer Firmware
 * heater.h
*/
#ifndef _HEATER_H
#define _HEATER_H

#include "common.h"

typedef struct {
    double P;
    double I;
    double D;
    double I_limit;
    double FF_factor;
    double FF_offset;
} pid_settings;

typedef const struct {
    channel_tag  tag;
    channel_tag  temp;
    channel_tag  pwm;
    pid_settings pid;
    double       setpoint;
} heater_config_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern int heater_config(heater_config_t *pcfgs, int nr_cfgs);

extern int heater_init(void);
extern void heater_exit(void);

extern void heater_stop(void);

extern channel_tag heater_lookup_by_name(const char *name);
extern channel_tag heater_lookup_by_index(int idx);

extern int heater_set_setpoint(channel_tag heater, double setpoint);
extern int heater_get_setpoint(channel_tag heater, double *setpoint);

extern int heater_set_pid_values(channel_tag heater, const pid_settings *pid);
extern int heater_get_pid_values(channel_tag heater, pid_settings *pid);

extern int heater_save_settings(void);
extern int heater_load_settings(void);

extern int heater_get_celsius(channel_tag heater, double *pcelsius);
extern int heater_temp_reached(channel_tag heater);

extern int heater_enable(channel_tag heater, int state);

extern int heater_set_raw_pwm(channel_tag heater, double percentage);

#if defined (__cplusplus)
}
#endif
#endif
