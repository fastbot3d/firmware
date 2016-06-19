/*
 * Unicorn 3D Printer Firmware
 * pwm.h
*/
#ifndef _PWM_H
#define _PWM_H

#include "common.h"

typedef const struct {
    channel_tag  tag;
    unsigned int frequency;
    unsigned int period_ns;
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

extern int pwm_set_freq(channel_tag pwm_ch, unsigned int freq);
extern int pwm_set_output(channel_tag pwm_ch, unsigned int percentage);

extern int pwm_set_period(channel_tag pwm_ch, unsigned int period_ns);
extern int pwm_set_duty(channel_tag pwm_ch, unsigned int duty_ns);

extern int pwm_get_state(channel_tag pwm_ch);

extern channel_tag pwm_lookup_by_name(const char *name);

#if defined (__cplusplus)
}
#endif
#endif
