/*
 * Unicorn 3D Printer Firmware
 * unicorn.c
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
#include <stdlib.h>
#include <string.h>

#include "parameter.h"
#include "analog.h"
#include "thermistor.h"
#include "temp.h"
#include "pwm.h"
#include "fan.h"
#include "led.h"
#include "heater.h"
#include "lmsw.h"
#include "stepper.h"
#include "planner.h"
#include "gcode.h"
#include "unicorn.h"

//#define PWM_PCA9685 1

/* ADC input */
#define AIN_CH_EXT          "/sys/bus/iio/devices/iio:device0/in_voltage4_raw"
#define AIN_CH_BED          "/sys/bus/iio/devices/iio:device0/in_voltage6_raw"

/* DAC output */
#define AOUT_CH_VREF_X      "/sys/devices/platform/vref_consumer_x/microvolts"
#define AOUT_CH_VREF_Y      "/sys/devices/platform/vref_consumer_y/microvolts"
#define AOUT_CH_VREF_Z      "/sys/devices/platform/vref_consumer_z/microvolts"
#define AOUT_CH_VREF_EXT    "/sys/devices/platform/vref_consumer_ext1/microvolts"
#define AOUT_CH_VREF_EXT2   "/sys/devices/platform/vref_consumer_ext2/microvolts"

/* PWM output */
#if defined (PWM_PCA9685)
#define PWM_EXT_DEV         "/sys/class/pwm/1-007f:0"
#define PWM_BED_DEV         "/sys/class/pwm/1-007f:6"
#define PWM_FAN_EXT_DEV     "/sys/class/pwm/1-007f:15"
#define PWM_FAN_SYS_DEV     "/sys/class/pwm/1-007f:3"
#define PWM_LED_STATUS_DEV  "/sys/class/pwm/1-007f:14"
#define PWM_LED_SYS_DEV     "/sys/class/pwm/1-007f:13"
#else
#define PWM_EXT_DEV         "/sys/class/pwm/ehrpwm.0:0"
#define PWM_BED_DEV         "/sys/class/pwm/ehrpwm.1:0"
#define PWM_FAN_EXT_DEV     "/sys/class/pwm/ecap.0"
#define PWM_FAN_SYS_DEV     "/sys/class/pwm/ehrpwm.1:1"
#endif

#define PWM_EXT_FREQ        (1024)
#define PWM_BED_FREQ        (1024)
#define PWM_FAN_EXT_FREQ    (512)
#define PWM_FAN_SYS_FREQ    (1024)

#if defined (PWM_PCA9685)
#define PWM_LED_STATUS_FREQ (1000)
#define PWM_LED_SYS_FREQ    (1000)
#endif

/* generate name tags */
GENERATE_TAG(thermistor_ext);
GENERATE_TAG(thermistor_bed);

GENERATE_TAG(vref_x);
GENERATE_TAG(vref_y);
GENERATE_TAG(vref_z);
GENERATE_TAG(vref_e);
GENERATE_TAG(vref_e2);

GENERATE_TAG(temp_ext);
GENERATE_TAG(temp_bed);

GENERATE_TAG(pwm_ext);
GENERATE_TAG(pwm_bed);
GENERATE_TAG(pwm_fan_ext);
GENERATE_TAG(pwm_fan_sys);
#if defined (PWM_PCA9685)
GENERATE_TAG(pwm_led_status);
GENERATE_TAG(pwm_led_sys);
#endif

GENERATE_TAG(fan_ext);
GENERATE_TAG(fan_sys);

#if defined (PWM_PCA9685)
GENERATE_TAG(led_status);
GENERATE_TAG(led_sys);
#endif

GENERATE_TAG(heater_ext);
GENERATE_TAG(heater_bed);

GENERATE_TAG(stepper_x);
GENERATE_TAG(stepper_y);
GENERATE_TAG(stepper_z);
GENERATE_TAG(stepper_e);
GENERATE_TAG(stepper_e2);

static const analog_config_t analog_config_data[] = {
    {
        .tag         = thermistor_bed,
        .device_path = AIN_CH_BED,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = thermistor_ext,
        .device_path = AIN_CH_EXT,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = vref_x,
        .device_path = AOUT_CH_VREF_X,
        .type        = ANALOG_TYPE_OUT,
    },
    {
        .tag         = vref_y,
        .device_path = AOUT_CH_VREF_Y,
        .type        = ANALOG_TYPE_OUT,
    },
    {
        .tag         = vref_z,
        .device_path = AOUT_CH_VREF_Z,
        .type        = ANALOG_TYPE_OUT,
    },
    {
        .tag         = vref_e,
        .device_path = AOUT_CH_VREF_EXT,
        .type        = ANALOG_TYPE_OUT,
    },
    {
        .tag         = vref_e2,
        .device_path = AOUT_CH_VREF_EXT2,
        .type        = ANALOG_TYPE_OUT,
    },
};

static const temp_config_t temp_config_data[] = {
    {
        .tag           = temp_ext,
        .analog_input  = thermistor_ext,
        .convert       = temp_convert_extruder,
        .in_range_time = 30,
    },
    {
        .tag           = temp_bed,
        .analog_input  = thermistor_bed,
        .convert       = temp_convert_bed,
        .in_range_time = 60,
    },
};

static const pwm_config_t pwm_config_data[] = {
    {
        .tag         = pwm_ext,
        .device_path = PWM_EXT_DEV,
        .frequency   = PWM_EXT_FREQ,
    },
    {
        .tag         = pwm_bed,
        .device_path = PWM_BED_DEV,
        .frequency   = PWM_BED_FREQ,
    },
    {
        .tag         = pwm_fan_ext,
        .device_path = PWM_FAN_EXT_DEV,
        .frequency   = PWM_FAN_EXT_FREQ,
    },
    {
        .tag         = pwm_fan_sys,
        .device_path = PWM_FAN_SYS_DEV,
        .frequency   = PWM_FAN_SYS_FREQ,
    },
#if defined (PWM_PCA9685)
    {
        .tag         = pwm_led_status,
        .device_path = PWM_LED_STATUS_DEV,
        .frequency   = PWM_LED_STATUS_FREQ,
    },
    {
        .tag         = pwm_led_sys,
        .device_path = PWM_LED_SYS_DEV,
        .frequency   = PWM_LED_SYS_FREQ,
    },
#endif
};

static const fan_config_t fan_config_data[] = {
    {
        .tag   = fan_ext,
        .pwm   = pwm_fan_ext,
        .level = 50,
    },
    {
        .tag   = fan_sys,
        .pwm   = pwm_fan_sys,
        .level = 50,
    },
};

#if defined (PWM_PCA9685)
static const led_config_t led_config_data[] = {
    {
        .tag   = led_status,
        .pwm   = pwm_led_status,
        .level = 50,
    },
    {
        .tag   = led_sys,
        .pwm   = pwm_led_sys,
        .level = 50,
    },
};
#endif

static const heater_config_t heater_config_data[] = {
    {
        .tag  = heater_ext,
        .temp = temp_ext,
        .pwm  = pwm_ext,
        .pid  = 
        {
            .P = 10.0,
            .I = 0.5,
            .D = 0.0,
            .I_limit = 10.0,
            .FF_factor = 0.033,
            .FF_offset = 40.0,
        },
    },
    {
        .tag  = heater_bed,
        .temp = temp_bed,
        .pwm  = pwm_bed,
        .pid  = 
        {
            .P = 25.0,
            .I = 0.6,
            .D = 0.0,
            .I_limit = 80.0,
            .FF_factor = 1.03,
            .FF_offset = 29.0,
        },
    },
};

static const stepper_config_t stepper_config_data[] = {
    {
        .tag  = stepper_x, 
        .vref = vref_x,
    },
    {
        .tag = stepper_y, 
        .vref = vref_y,
    },
    {
        .tag = stepper_z, 
        .vref = vref_z,
    },
    {
        .tag = stepper_e, 
        .vref = vref_e,
    },
    {
        .tag = stepper_e2, 
        .vref = vref_e2,
    },
};

/*
 * Setup all sub sys of unicorn
 */
static int unicorn_setup(void)
{
    int ret = 0; 

    ret = analog_config(analog_config_data, NR_ITEMS(analog_config_data));
    if (ret < 0) {
        fprintf(stderr, "analog_config failed\n");
        goto out;
    }

    ret = temp_config(temp_config_data, NR_ITEMS(temp_config_data));
    if (ret < 0) {
        fprintf(stderr, "temp_config failed\n");
        goto out;
    }

    ret = pwm_config(pwm_config_data, NR_ITEMS(pwm_config_data));
    if (ret < 0) {
        fprintf(stderr, "pwm_config failed\n");
        goto out;
    }
    
    ret = fan_config(fan_config_data, NR_ITEMS(fan_config_data));
    if (ret < 0) {
        fprintf(stderr, "fan_config failed\n");
        goto out;
    }

#if defined (PWM_PCA9685)
    ret = led_config(led_config_data, NR_ITEMS(led_config_data));
    if (ret < 0) {
        fprintf(stderr, "led_config failed\n");
        goto out;
    }
#endif

    ret = heater_config(heater_config_data, NR_ITEMS(heater_config_data));
    if (ret < 0) {
        fprintf(stderr, "heater_config failed\n");
        goto out;
    }

    ret = stepper_config(stepper_config_data, NR_ITEMS(stepper_config_data));
    if (ret < 0) {
        fprintf(stderr, "stepper_config failed\n");
        goto out;
    }

out:
    return ret;
}
/*
 * Init unicorn, create sub sys and threads
 */
int unicorn_init(void)
{
    int ret = 0;
    
    /* set up unicorn system */
    ret = unicorn_setup();
    if (ret < 0) {
        return ret;
    }

    /* init sub systems of unicorn */
    ret = parameter_init();
    if (ret < 0) {
        return ret;
    }

    ret = analog_init();
    if (ret < 0) {
        return ret;
    }

    ret = temp_init();
    if (ret < 0) {
        return ret;
    }

    ret = pwm_init();
    if (ret < 0) {
        return ret;
    }

    ret = fan_init();
    if (ret < 0) {
        return ret;
    }

#if defined (PWM_PCA9685)
    ret = led_init();
    if (ret < 0) {
        return ret;
    }
#endif

    ret = heater_init();
    if (ret < 0) {
        return ret;
    }

    ret = lmsw_init();
    if (ret < 0) {
        return ret;
    }

    ret = stepper_init();
    if (ret < 0) {
        return ret;
    }

    ret = plan_init();
    if (ret < 0) {
        return ret;
    }

    ret = gcode_init();
    if (ret < 0) {
        return ret;
    }

    return ret;
}
/*
 * Exit from unicorn, delete sub sys and threads
 */
void unicorn_exit(int blocking)
{
    gcode_exit();
    plan_exit();
    stepper_exit(blocking);
    lmsw_exit();
    heater_exit();
    led_exit();
    fan_exit();
    pwm_exit();
    temp_exit();
    analog_exit();
    parameter_exit();
}
/*
 * Pause printing
 */
int unicorn_pause(void)
{
    channel_tag fan = fan_lookup_by_name("fan_sys");
#if defined (PWM_PCA9685)
    channel_tag led = led_lookup_by_name("led_status");
#endif

    stepper_pause();

    /* stop heating */ 
    
    /* turn off fans and leds */
    fan_disable(fan);

#if defined (PWM_PCA9685)
    led_disable(led);
#endif
    return 0;
}
/*
 * Resume printing
 */
int unicorn_resume(void)
{
    channel_tag fan = fan_lookup_by_name("fan_sys");

#if defined (PWM_PCA9685)
    channel_tag led = led_lookup_by_name("led_status");
    /* turn on leds */
    led_enable(led);
    led_set_level(led, 99);
#endif

    /* turn on leds */

    /* start heating */

    /* waiting to achieve target temperature */

    
    /* turn on system fan */
    fan_enable(fan);
    fan_set_level(fan, 99);

    stepper_resume();

    return 0;
}
/*
 * Start printing
 */
int unicorn_start(void)
{
    channel_tag fan = fan_lookup_by_name("fan_sys");
#if defined (PWM_PCA9685)
    channel_tag led = led_lookup_by_name("led_status");
#endif
    /* turn on system fan */
    fan_enable(fan);
    fan_set_level(fan, 99);

#if defined (PWM_PCA9685)
    /* turn on leds */
    led_enable(led);
    led_set_level(led, 99);
#endif

    stepper_start();
    
    return 0;
}
/*
 * Stop printing
 */
int unicorn_stop(void)
{
    channel_tag fan = fan_lookup_by_name("fan_sys");
#if defined (PWM_PCA9685)
    channel_tag led = led_lookup_by_name("led_status");
#endif

    /* turn off system fan */
    fan_disable(fan);

#if defined (PWM_PCA9685)
    /* turn off leds */
    led_disable(led);
#endif
    
    /* turn off heating */
    gcode_stop();
    plan_stop();
    heater_stop();

    return 0;
}
/*
 * Start print gcode file
 */
int unicorn_print(char *file)
{
    char *ptr = NULL;
    FILE *fp = fopen(file, "r");
    if (!fp) {
        return -1;
    }

    while (1) {
        ptr = gcode_get_line_from_file(fp);
        if (!ptr) {
            break;
        }

        gcode_process_line();
    }

    return 0;
}

