/*
 * Unicorn 3D Printer Firmware
 * unicorn.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "common.h"
#include "parameter.h"
#include "analog.h"
#include "thermistor.h"
#include "temp.h"
#include "pwm.h"
#include "fan.h"
#include "heater.h"
#ifdef SERVO
#include "servo.h"
#endif
#include "lmsw.h"
#include "stepper.h"
#include "planner.h"
#include "gcode.h"
#include "unicorn.h"
#include "eeprom.h"
#include "test.h"

#include "util/Fifo.h"
#include "util/Pause.h"

static volatile int unicorn_mode = FW_MODE_REMOTE;

#if 0  //move common.h
/* ADC input */
#if 0
#define AIN_CH_EXT           "/sys/bus/iio/devices/iio:device0/in_voltage4_raw"
#define AIN_CH_EXT2          "/sys/bus/iio/devices/iio:device0/in_voltage5_raw"
#define AIN_CH_EXT3          "/sys/class/hwmon/hwmon0/device/vout"
//#define AIN_CH_EXT3          "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
#define AIN_CH_EXT4          "/sys/bus/iio/devices/iio:device0/in_voltage2_raw"
#define AIN_CH_EXT5          "/sys/bus/iio/devices/iio:device0/in_voltage3_raw"
#define AIN_CH_BED           "/sys/bus/iio/devices/iio:device0/in_voltage6_raw"
#else
	#define AIN_CH_EXT           "/sys/bus/iio/devices/iio:device0/voltage4"
	#define AIN_CH_EXT2          "/sys/bus/iio/devices/iio:device0/voltage5"
	#define AIN_CH_EXT3          "/sys/class/hwmon/hwmon0/device/vout"
	//#define AIN_CH_EXT3          "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
	#define AIN_CH_EXT4          "/sys/bus/iio/devices/iio:device0/voltage3"
	#define AIN_CH_EXT5          "/sys/bus/iio/devices/iio:device0/voltage2"
	#define AIN_CH_EXT6          "/sys/bus/iio/devices/iio:device0/voltage1"
	#define AIN_CH_BED           "/sys/bus/iio/devices/iio:device0/voltage6"
#endif
#endif

/* DAC output */
#define AOUT_CH_VREF_X       "/sys/devices/platform/vref_consumer_x/microvolts"
#define AOUT_CH_VREF_Y       "/sys/devices/platform/vref_consumer_y/microvolts"
#define AOUT_CH_VREF_Z       "/sys/devices/platform/vref_consumer_z/microvolts"
#define AOUT_CH_VREF_EXT     "/sys/devices/platform/vref_consumer_ext1/microvolts"
#define AOUT_CH_VREF_EXT2    "/sys/devices/platform/vref_consumer_ext2/microvolts"
#define AOUT_CH_VREF_EXT3    "/sys/devices/platform/vref_consumer_ext3/microvolts"
#define AOUT_CH_VREF_USER    "/sys/devices/platform/vref_consumer_ext4/microvolts"
#define AOUT_CH_VREF_USER2   "/sys/devices/platform/vref_consumer_ext5/microvolts"

/* PWM output */
#define PCA_PWM_EXT_DEV      "/sys/class/pwm/2-007f:0"
#define PCA_PWM_EXT2_DEV     "/sys/class/pwm/2-007f:1"
#define PCA_PWM_BED_DEV      "/sys/class/pwm/2-007f:2"
#define PCA_PWM_FAN_EXT_DEV  "/sys/class/pwm/2-007f:3"
#define PCA_PWM_FAN_EXT2_DEV "/sys/class/pwm/2-007f:4"
#define PCA_PWM_FAN_3_DEV    "/sys/class/pwm/2-007f:5"
#define PCA_PWM_FAN_4_DEV    "/sys/class/pwm/2-007f:6"
#define PCA_PWM_FAN_5_DEV    "/sys/class/pwm/2-007f:7"
#define PCA_PWM_FAN_6_DEV    "/sys/class/pwm/2-007f:8"
#define PCA_PWM_EXT3_DEV     "/sys/class/pwm/2-007f:9"
#define PCA_PWM_EXT4_DEV     "/sys/class/pwm/2-007f:10"
#define PCA_PWM_EXT5_DEV     "/sys/class/pwm/2-007f:11" 

#define PWM_EXT_DEV          "/sys/class/pwm/ehrpwm.0:0"
#define PWM_EXT2_DEV         "/sys/class/pwm/ehrpwm.0:1"
#define PWM_BED_DEV          "/sys/class/pwm/ehrpwm.1:0"
#define PWM_FAN_EXT_DEV      "/sys/class/pwm/ehrpwm.1:1"
#define PWM_FAN_EXT2_DEV     "/sys/class/pwm/ecap.0"

#define PWM_EXT_FREQ         (500)
#define PWM_EXT2_FREQ        (500)
#define PWM_BED_FREQ         (500)
#define PWM_FAN_EXT_FREQ     (500)
#define PWM_FAN_EXT2_FREQ    (500)
#define PWM_FAN_3_FREQ       (500)
#define PWM_FAN_4_FREQ       (500)
#define PWM_FAN_5_FREQ       (500)
#define PWM_FAN_6_FREQ       (500)
#define PWM_EXT3_FREQ        (500)
#define PWM_EXT4_FREQ        (500)
#define PWM_EXT5_FREQ        (500)

#ifdef SERVO
#define PWM_SERVO_DEV        "/sys/class/pwm/ecap.0"
#define PWM_SERVO_FREQ       (48)
#endif

/* generate name tags */
GENERATE_TAG(thermistor_ext);
GENERATE_TAG(thermistor_ext2);
GENERATE_TAG(thermistor_ext3);
GENERATE_TAG(thermistor_ext4);
GENERATE_TAG(thermistor_ext5);
GENERATE_TAG(thermistor_ext6);
GENERATE_TAG(thermistor_bed);

GENERATE_TAG(vref_x);
GENERATE_TAG(vref_y);
GENERATE_TAG(vref_z);
GENERATE_TAG(vref_e);
GENERATE_TAG(vref_e2);
GENERATE_TAG(vref_e3);
GENERATE_TAG(vref_u);
GENERATE_TAG(vref_u2);

GENERATE_TAG(temp_bed);
GENERATE_TAG(temp_ext);
GENERATE_TAG(temp_ext2);
GENERATE_TAG(temp_ext3);
GENERATE_TAG(temp_ext4);
GENERATE_TAG(temp_ext5);
GENERATE_TAG(temp_ext6);

GENERATE_TAG(pwm_bed);
GENERATE_TAG(pwm_ext);
GENERATE_TAG(pwm_ext2);
GENERATE_TAG(pwm_ext3);
GENERATE_TAG(pwm_ext4);
GENERATE_TAG(pwm_ext5);

GENERATE_TAG(pwm_fan_ext);
GENERATE_TAG(pwm_fan_ext2);
GENERATE_TAG(pwm_fan_ext3);
#ifdef SERVO
GENERATE_TAG(pwm_servo);
#endif
GENERATE_TAG(pwm_fan_3);
GENERATE_TAG(pwm_fan_4);
GENERATE_TAG(pwm_fan_5);
GENERATE_TAG(pwm_fan_6);

GENERATE_TAG(fan_ext);
GENERATE_TAG(fan_ext2);
GENERATE_TAG(fan_ext3);
GENERATE_TAG(fan_3);
GENERATE_TAG(fan_4);
GENERATE_TAG(fan_5);
GENERATE_TAG(fan_6);

#ifdef SERVO
GENERATE_TAG(servo_0);
#endif

GENERATE_TAG(heater_bed);
GENERATE_TAG(heater_ext);
GENERATE_TAG(heater_ext2);
GENERATE_TAG(heater_ext3);
GENERATE_TAG(heater_ext4);
GENERATE_TAG(heater_ext5);

GENERATE_TAG(stepper_x);
GENERATE_TAG(stepper_y);
GENERATE_TAG(stepper_z);
GENERATE_TAG(stepper_e);
GENERATE_TAG(stepper_e2);
GENERATE_TAG(stepper_e3);
GENERATE_TAG(stepper_u);
GENERATE_TAG(stepper_u2);

static const analog_config_t analog_config_bbp1[] = {
    {
        .tag         = thermistor_ext,
        .device_path = AIN_CH_EXT,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = thermistor_ext2,
        .device_path = AIN_CH_EXT2,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = thermistor_bed,
        .device_path = AIN_CH_BED,
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

static const analog_config_t analog_config_bbp1s[] = {
    {
        .tag         = thermistor_ext,
        .device_path = AIN_CH_EXT,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = thermistor_ext2,
        .device_path = AIN_CH_EXT2,
        .type        = ANALOG_TYPE_IN,
    },
	{
        .tag         = thermistor_ext3,
        .device_path = AIN_CH_EXT3,
        .type        = ANALOG_TYPE_IN,
    },
	{
        .tag         = thermistor_ext4,
        .device_path = AIN_CH_EXT4,
        .type        = ANALOG_TYPE_IN,
    },
	{
        .tag         = thermistor_ext5,
        .device_path = AIN_CH_EXT5,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = thermistor_ext6,
        .device_path = AIN_CH_EXT6,
        .type        = ANALOG_TYPE_IN,
    },
    {
        .tag         = thermistor_bed,
        .device_path = AIN_CH_BED,
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
    {
        .tag         = vref_e3,
        .device_path = AOUT_CH_VREF_EXT3,
        .type        = ANALOG_TYPE_OUT,
    },
    {
        .tag         = vref_u,
        .device_path = AOUT_CH_VREF_USER,
        .type        = ANALOG_TYPE_OUT,
    },
    {
        .tag         = vref_u2,
        .device_path = AOUT_CH_VREF_USER2,
        .type        = ANALOG_TYPE_OUT,
    },
};

static const temp_config_t temp_config_data_bbp1[] = {
    {
        .tag           = temp_ext,
        .analog_input  = thermistor_ext,
        .convert       = temp_convert_extruder1,
        .in_range_time = 10,
    },
    {
        .tag           = temp_ext2,
        .analog_input  = thermistor_ext2,
        .convert       = temp_convert_extruder2,
        .in_range_time = 10,
    },
    {
        .tag           = temp_bed,
        .analog_input  = thermistor_bed,
        .convert       = temp_convert_bed,
        .in_range_time = 10,
    },
};


static const temp_config_t temp_config_data_bbp1s[] = {
    {
        .tag           = temp_ext,
        .analog_input  = thermistor_ext,
        .convert       = temp_convert_extruder1,
        .in_range_time = 10,
    },
    {
        .tag           = temp_ext2,
        .analog_input  = thermistor_ext2,
        .convert       = temp_convert_extruder2,
        .in_range_time = 10,
    },
    {
        .tag           = temp_ext3,
        .analog_input  = thermistor_ext3,
        .convert       = temp_convert_extruder3,
        .in_range_time = 10,
    },
    {
        .tag           = temp_ext4,
        .analog_input  = thermistor_ext4,
        .convert       = temp_convert_extruder4,   //FIXME
        .in_range_time = 10,
    },
	{
        .tag           = temp_ext5,
        .analog_input  = thermistor_ext5,
        .convert       = temp_convert_extruder5,   //FIXME
        .in_range_time = 10,
    },{
        .tag           = temp_ext6,
        .analog_input  = thermistor_ext6,
        .convert       = temp_convert_extruder6,   //FIXME
        .in_range_time = 10,
    },
    {
        .tag           = temp_bed,
        .analog_input  = thermistor_bed,
        .convert       = temp_convert_bed,
        .in_range_time = 10,
    },
};

static const pwm_config_t pwm_config_bbp1[] = {
    {
        .tag         = pwm_ext,
        .device_path = PWM_EXT_DEV,
        .frequency   = PWM_EXT_FREQ,
    },
    {
        .tag         = pwm_ext2,
        .device_path = PWM_EXT2_DEV,
        .frequency   = PWM_EXT2_FREQ,
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
        .tag         = pwm_fan_ext2,
        .device_path = PWM_FAN_EXT2_DEV,
        .frequency   = PWM_FAN_EXT2_FREQ,
    },
};

static const pwm_config_t pwm_config_bbp1s[] = {
    {
        .tag         = pwm_ext,
        .device_path = PCA_PWM_EXT_DEV,
        .frequency   = PWM_EXT_FREQ,
    },
    {
        .tag         = pwm_ext2,
        .device_path = PCA_PWM_EXT2_DEV,
        .frequency   = PWM_EXT2_FREQ,
    },
    {
        .tag         = pwm_ext3,
        .device_path = PCA_PWM_EXT3_DEV,
        .frequency   = PWM_EXT3_FREQ,
    },
    {
        .tag         = pwm_bed,
        .device_path = PCA_PWM_BED_DEV,
        .frequency   = PWM_BED_FREQ,
    },
    {
        .tag         = pwm_fan_ext,
        .device_path = PCA_PWM_FAN_EXT_DEV,
        .frequency   = PWM_FAN_EXT_FREQ,
    },
    {
        .tag         = pwm_fan_ext2,
        .device_path = PCA_PWM_FAN_EXT2_DEV,
        .frequency   = PWM_FAN_EXT2_FREQ,
    },
    {
        .tag         = pwm_fan_3,
        .device_path = PCA_PWM_FAN_3_DEV,
        .frequency   = PWM_FAN_3_FREQ,
    },
    {
        .tag         = pwm_fan_4,
        .device_path = PCA_PWM_FAN_4_DEV,
        .frequency   = PWM_FAN_4_FREQ,
    },
    {
        .tag         = pwm_fan_5,
        .device_path = PCA_PWM_FAN_5_DEV,
        .frequency   = PWM_FAN_5_FREQ,
    },
    {
        .tag         = pwm_fan_6,
        .device_path = PCA_PWM_FAN_6_DEV,
        .frequency   = PWM_FAN_6_FREQ,
    },
    {
        .tag         = pwm_ext3,
        .device_path = PCA_PWM_EXT3_DEV,
        .frequency   = PWM_EXT3_FREQ,
    },
    {
        .tag         = pwm_ext4,
        .device_path = PCA_PWM_EXT4_DEV,
        .frequency   = PWM_EXT4_FREQ,
    },
    {
        .tag         = pwm_ext5,
        .device_path = PCA_PWM_EXT5_DEV,
        .frequency   = PWM_EXT5_FREQ,
    },
#ifdef SERVO
    {
        .tag         = pwm_servo,
        .device_path = PWM_SERVO_DEV,
        .frequency   = 0,
    },
#endif
};

static const fan_config_t fan_config_bbp1[] = {
    {
        .tag   = fan_ext,
        .pwm   = pwm_fan_ext,
        .gpio  = 0,
        .level = 50,
    },
    {
        .tag   = fan_ext2,
        .pwm   = pwm_fan_ext2,
        .gpio  = 0,
        .level = 50,
    },
    {
        .tag   = fan_3,
        .pwm   = NULL,
        .gpio  = 41,
        .level = 100,
    },
    {
        .tag   = fan_4,
        .pwm   = NULL,
        .gpio  = 40,
        .level = 100,
    },
    {
        .tag   = fan_5,
        .pwm   = NULL,
        .gpio  = 14,
        .level = 100,
    },
};

static const fan_config_t fan_config_bbp1s[] = {
    {
        .tag   = fan_ext,
        .pwm   = pwm_fan_ext,
        .gpio  = 0,
        .level = 50,
    },
    {
        .tag   = fan_ext2,
        .pwm   = pwm_fan_ext2,
        .gpio  = 0,
        .level = 50,
    },
    {
        .tag   = fan_3,
        .pwm   = pwm_fan_3,
        .gpio  = 0,
        .level = 99,
    },
    {
        .tag   = fan_4,
        .pwm   = pwm_fan_4,
        .gpio  = 0,
        .level = 99,
    },
    {
        .tag   = fan_5,
        .pwm   = pwm_fan_5,
        .gpio  = 0,
        .level = 99,
    },
    {
        .tag   = fan_6,
        .pwm   = pwm_fan_6,
        .gpio  = 0,
        .level = 99,
    },
};

#ifdef SERVO
static const servo_config_t servo_config_data[] = {
    {
        .tag   = servo_0,
        .pwm   = pwm_servo,
        .angle = 10,
    },
};
#endif

static const heater_config_t heater_config_data_bbp[] = {
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
        .tag  = heater_ext2,
        .temp = temp_ext2,
        .pwm  = pwm_ext2,
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
static const heater_config_t heater_config_data_bbp1s[] = {
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
        .tag  = heater_ext2,
        .temp = temp_ext2,
        .pwm  = pwm_ext2,
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
        .tag  = heater_ext3,
        .temp = temp_ext3,
        .pwm  = pwm_ext3,
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
        .tag  = heater_ext4,
        .temp = temp_ext4,
        .pwm  = pwm_ext4,
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
        .tag  = heater_ext5,
        .temp = temp_ext5,
        .pwm  = pwm_ext5,
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
#if 0
    {
        .tag  = heater_ext6,
        .temp = temp_ext6,
        .pwm  = pwm_ext6,
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
#endif
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

static const stepper_config_t stepper_config_bbp1[] = {
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

static const stepper_config_t stepper_config_bbp1s[] = {
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
    {
        .tag = stepper_e3, 
        .vref = vref_e3,
    },
    {
        .tag = stepper_u, 
        .vref = vref_u,
    },
    {
        .tag = stepper_u2, 
        .vref = vref_u2,
    },
};
/*
 * Set unicorn running mode:
 *     FW_MODE_REMOTE, FW_MODE_LOCAL, FW_MODE_TEST
 */
int unicorn_set_mode(int mode)
{
    int ret = 0;
    if (mode > FW_MODE_MIN && mode < FW_MODE_MAX) {
        unicorn_mode = mode;
    } else {
        ret = -1;
    }
    return ret;
}
/*
 * Get unicorn running mode
 */
int unicorn_get_mode(void)
{
    return unicorn_mode;
}
/* 
 * Set feedrate multiply: [10, 1000]
 */
void unicorn_set_feedrate(int multiply)
{
    gcode_set_feed(multiply);
}
/*
 * Setup all sub sys of unicorn
 */
static int unicorn_setup()
{
    int ret = 0; 

    if (bbp_board_type == BOARD_BBP1) {
        printf("Board is FastBot BBP 1\n");
        ret = analog_config(analog_config_bbp1, NR_ITEMS(analog_config_bbp1));
        if (ret < 0) {
            printf("analog_config failed\n");
            goto out;
        }

        ret = temp_config(temp_config_data_bbp1, NR_ITEMS(temp_config_data_bbp1));
        if (ret < 0) {
            printf("temp_config failed\n");
            goto out;
        }

        ret = pwm_config(pwm_config_bbp1, NR_ITEMS(pwm_config_bbp1));
        if (ret < 0) {
            printf("pwm_config failed\n");
            goto out;
        }
        
        ret = fan_config(fan_config_bbp1, NR_ITEMS(fan_config_bbp1));
        if (ret < 0) {
            printf("fan_config failed\n");
            goto out;
        }

        ret = heater_config(heater_config_data_bbp, NR_ITEMS(heater_config_data_bbp));
        if (ret < 0) {
            printf("heater_config failed\n");
            goto out;
        }

        ret = stepper_config(stepper_config_bbp1, NR_ITEMS(stepper_config_bbp1));
        if (ret < 0) {
            printf("stepper_config failed\n");
            goto out;
        }
    } else if (bbp_board_type == BOARD_BBP1S) {
        printf("Board is FastBot BBP 1s\n");
        ret = analog_config(analog_config_bbp1s, NR_ITEMS(analog_config_bbp1s));
        if (ret < 0) {
            printf("analog_config failed\n");
            goto out;
        }

        ret = temp_config(temp_config_data_bbp1s, NR_ITEMS(temp_config_data_bbp1s));
        if (ret < 0) {
            printf("temp_config failed\n");
            goto out;
        }

        ret = pwm_config(pwm_config_bbp1s, NR_ITEMS(pwm_config_bbp1s));
        if (ret < 0) {
            printf("pwm_config failed\n");
            goto out;
        }
        
        ret = fan_config(fan_config_bbp1s, NR_ITEMS(fan_config_bbp1s));
        if (ret < 0) {
            printf("fan_config failed\n");
            goto out;
        }

        ret = heater_config(heater_config_data_bbp1s, NR_ITEMS(heater_config_data_bbp1s));
        if (ret < 0) {
            printf("heater_config failed\n");
            goto out;
        }

#ifdef SERVO
		if (bbp_board_type == BOARD_BBP1S) {
			ret = servo_config(servo_config_data, NR_ITEMS(servo_config_data));
			if (ret < 0) {
				printf("servo_config failed\n");
				goto out;
			}
		}
#endif

        ret = stepper_config(stepper_config_bbp1s, NR_ITEMS(stepper_config_bbp1s));
        if (ret < 0) {
            printf("stepper_config failed\n");
            goto out;
        }
    } else {
        printf("Not supported board!\n");
    }
out:
    return ret;
}
/*
 * Init unicorn, create sub sys and threads
 */
Fifo_Handle hFifo_st2plan;
Fifo_Handle hFifo_plan2st;
Pause_Handle hPause_printing;

int bbp_board_type = -1;

int unicorn_init(void)
{
    channel_tag fan = NULL;
    int ret = 0;
    board_info_t board_info; 

	printf("unicorn_init...\n");
    ret = eeprom_read_board_info(EEPROM_DEV, &board_info);
    if (ret < 0) {
        printf("Read eeprom board info failed\n");
        return -1;
    }

    if (!strncasecmp("bbp1s", board_info.name, 5)) {
            bbp_board_type = BOARD_BBP1S;
    } else if(!strncasecmp("bbp1", board_info.name, 4)){
            bbp_board_type = BOARD_BBP1;
    }

	printf("board name:%s, version:%s, get board type:%d \n", board_info.name, board_info.version, bbp_board_type);
    
    /* 
     * Create fifo 
     * fifo_plan2st, fifo_st2plan
     * fifo_plan2gc, fifo_gc2plan
     */
    Fifo_Attrs fAttrs = Fifo_Attrs_DEFAULT;
    hFifo_plan2st = Fifo_create(&fAttrs);
    hFifo_st2plan = Fifo_create(&fAttrs);
    if ((hFifo_st2plan == NULL) ||
        (hFifo_plan2st == NULL)) {
        printf("Create Fifo failed\n");
        return -1;
    }
    
    Pause_Attrs pAttrs = Pause_Attrs_DEFAULT;
    hPause_printing = Pause_create(&pAttrs);
    if (hPause_printing == NULL) { 
        printf("Create pause err\n");
        return -1;
    } else {
        Pause_on(hPause_printing);
    }

    /* set up unicorn system */
    ret = unicorn_setup();
    if (ret < 0) {
        return ret;
    }

    /* init sub systems of unicorn */
    ret = parameter_init(EEPROM_DEV);
    if (ret < 0) {
        printf("parameter_init failed\n");
        return ret;
    }

    ret = analog_init();
    if (ret < 0) {
        printf("analog_init failed\n");
        return ret;
    }

    ret = temp_init();
    if (ret < 0) {
        printf("temp_init failed\n");
        return ret;
    }

    ret = pwm_init();
    if (ret < 0) {
        printf("pwm_init failed\n");
        return ret;
    }

    ret = fan_init();
    if (ret < 0) {
        printf("fan_init failed\n");
        return ret;
    }

    ret = heater_init();
    if (ret < 0) {
        printf("heater_init failed\n");
        return ret;
    }

#ifdef SERVO
    if (bbp_board_type == BOARD_BBP1S) {
		ret = servo_init();
		if (ret < 0) {
			printf("servo_init failed\n");
			return ret;
		}
	}
#endif

    ret = lmsw_init();
    if (ret < 0) {
        printf("lmsw_init failed\n");
        return ret;
    }

    ret = plan_init();
    if (ret < 0) {
        printf("plan_init failed\n");
        return ret;
    }

    ret = stepper_init();
    if (ret < 0) {
        printf("stepper_init failed\n");
        return ret;
    }
    
    ret = gcode_init();
    if (ret < 0) {
        printf("gcode_init failed\n");
        return ret;
    }

    fan = fan_lookup_by_name("fan_3");
    if (fan) {
        fan_set_level(fan, DEFAULT_FAN_MAX_LEVEL);
        fan_enable(fan);
    }

    fan = fan_lookup_by_name("fan_4");
    if (fan) {
        fan_enable(fan);
        fan_set_level(fan, DEFAULT_FAN_MAX_LEVEL);
    }

    fan = fan_lookup_by_name("fan_5");
    if (fan) {
        fan_enable(fan);
        fan_set_level(fan, DEFAULT_FAN_MAX_LEVEL);
    }

    fan = fan_lookup_by_name("fan_6");
    if (fan) {
        fan_enable(fan);
        fan_set_level(fan, DEFAULT_FAN_MAX_LEVEL);
    }
  
	printf("unicorn_init ok!!!\n");
    return ret;
}
/*
 * Exit from unicorn, delete sub sys and threads
 */
void unicorn_exit(int blocking)
{
    printf("unicorn_exit...\n");

    /* Flush stepper and planner fifo */ 
    if (hFifo_plan2st) {
        Fifo_flush(hFifo_plan2st);
    }
    if (hFifo_st2plan) {
        Fifo_flush(hFifo_st2plan);
    }

    if (hPause_printing) {
        Pause_off(hPause_printing);
    }

    heater_exit();
    gcode_exit();
    stepper_exit(blocking);
    plan_exit();

#ifdef SERVO
    if (bbp_board_type == BOARD_BBP1S) {
    	servo_exit();
	}
#endif

    lmsw_exit();
    pwm_exit();
    temp_exit();
    analog_exit();
    parameter_exit();

    /* Delete fifo */
    if (hFifo_plan2st) { 
        Fifo_delete(hFifo_plan2st);
    }
    
    if (hFifo_st2plan) {
        Fifo_delete(hFifo_st2plan);
    }
    
    if (hPause_printing) {
        Pause_delete(hPause_printing);
    }

    fan_exit(); //???? glibc detected *** /usr/bin/unicorn: corrupted double-linked list: 0x0007e330 
    printf("unicorn_exit..., done\n");
}
/*
 * Pause printing
 */
int unicorn_pause(void)
{
    if (hPause_printing) {
        Pause_on(hPause_printing);
    }

    stepper_pause();

    /* 
     * stop heating 
     */ 
    //TODO: 
    
    return 0;
}
/*
 * Resume printing
 */
int unicorn_resume(void)
{
    /* 
     * Start heating 
     * waiting to achieve target temperature 
     */
    //TODO:

    stepper_resume();

    if (hPause_printing) {
        Pause_off(hPause_printing);
    }

    return 0;
}
/*
 * Start printing
 */
int unicorn_start(int fd_rd, int fd_wr, int fd_rd_emerg)
{
    /* Turn on fans  */
	printf("unicorn_start\n");

    channel_tag fan = NULL;
    fan = fan_lookup_by_name("fan_3");
    if (fan) {
        fan_set_level(fan, DEFAULT_FAN_MAX_LEVEL);
        fan_enable(fan);
    }
    fan = fan_lookup_by_name("fan_4");
    if (fan) {
        fan_set_level(fan, DEFAULT_FAN_MAX_LEVEL);
        fan_enable(fan);
    }
    
    heater_start();
	plan_start();
    stepper_start();
    gcode_start(fd_rd, fd_wr, fd_rd_emerg);

    if (hPause_printing) {
        Pause_off(hPause_printing);
    }

	printf("unicorn_start ok!!!\n");
    return 0;
}
/*
 * Stop printing
 */
int unicorn_stop(bool blocking)
{
    channel_tag fan = NULL;

    if (blocking) {
        printf("unicorn_stop: blocking\n");
    } else {
        printf("unicorn_stop: non-blocking\n");
    }
    
    /* If paused, pause thread off */
    if (!blocking) {
        if (hPause_printing) {
            Pause_on(hPause_printing);
        }
    }

	printf("stepper_stop\n");
    stepper_stop(blocking);
    printf("ok\n");

	fan = fan_lookup_by_name("fan_ext");
    if (fan) {
        fan_set_level(fan, 0);
        fan_disable(fan);
    }

	fan = fan_lookup_by_name("fan_ext2");
    if (fan) {
        fan_set_level(fan, 0);
        fan_disable(fan);
    }

    printf("gcode_stop\n");
    gcode_stop();
    printf("ok\n");

    //if (!blocking) {
        /* Clean up all fifo data */
        stepper_clean_up_fifo();
    //}

    printf("plan_stop\n");
    plan_stop();
    printf("ok\n");

    printf("heater_stop\n");
    heater_stop();
    printf("ok\n");
    

    if (!blocking) {
        if (unicorn_get_mode() == FW_MODE_REMOTE) {
            if (hPause_printing) {
                Pause_off(hPause_printing);
            }
        }
    }

    printf("unicorn stop ok!\n");
    return 0;
}

int unicorn_disconnect_octoprint()
{
    channel_tag fan = NULL;

    /* If paused, pause thread off */
	if (hPause_printing) {
		Pause_on(hPause_printing);
	}

	printf("stepper_idle\n");
	stepper_idle();
    printf("ok\n");

	fan = fan_lookup_by_name("fan_ext");
    if (fan) {
        fan_set_level(fan, 0);
        fan_disable(fan);
    }

	fan = fan_lookup_by_name("fan_ext2");
    if (fan) {
        fan_set_level(fan, 0);
        fan_disable(fan);
    }

    printf("gcode_stop\n");
    gcode_stop();
    printf("ok\n");

    //if (!blocking) {
        /* Clean up all fifo data */
        stepper_clean_up_fifo();
    //}

    printf("plan_stop\n");
    plan_stop();
    printf("ok\n");

    printf("heater_stop\n");
    heater_stop();
    printf("ok\n");
    
	if (unicorn_get_mode() == FW_MODE_REMOTE) {
		if (hPause_printing) {
			Pause_off(hPause_printing);
		}
	}

    printf("unicorn disconnect ok!\n");
    return 0;
}

int unicorn_restart(void)
{
    stepper_start();
    plan_start();

    if (hPause_printing) {
        Pause_off(hPause_printing);
    }

    return 0;
}
static void halt_test(char *msg)
{
	printf("------------------------------------------\n");
	printf("ERROR:, unicorn test halt, msg:%s", msg);
	printf("------------------------------------------\n");
	while(1){
		sleep(1);
	}
}

#if 0
static void *test_thread_worker(void *arg)
{
	sleep(4);
	printf("test thread start---\n");
    stepper_test();
	printf("test thread done---\n");
    return NULL;
}

static void start_test_thread()
{
    int ret, i;
    pthread_attr_t attr;
    struct sched_param sched;
	pthread_t test_thread;

	if (pthread_attr_init(&attr)) {
		return;
	}

	if (pthread_attr_setschedpolicy(&attr, PTHREAD_EXPLICIT_SCHED)) {
		return;
	}
	sched.sched_priority = sched_get_priority_max(SCHED_FIFO);

	if (pthread_attr_setschedparam(&attr, &sched)) {
		return ;
	}

	printf("--- Creating test_thread..."); 
	ret = pthread_create(&test_thread, &attr, test_thread_worker, NULL);
	if (ret) {
		printf("create gcode thread failed with ret %d\n", ret);
		return ;
	} else {
		printf("done ---\n");
	}
}
#endif


/* 
 * unicorn testing
 */
#define NUM_HEATERS  (3)
#define TARGET_TEMP  50.0
int unicorn_test(void)
{
    int i = 0;
    int idx = 0; 
    int num_fans = 0;
    double celsius = 0.0;
    channel_tag heater, fan, pwm;

    if (bbp_board_type == BOARD_BBP1S) {
    	num_fans = 6;
	} else if (bbp_board_type == BOARD_BBP1) {
    	num_fans = 5;
	}

#if 0
	start_test_thread();
	int count = 10;
	while (1) {
    	printf("wait evtout count=%d\n", count);
        prussdrv_pru_wait_event(PRU_EVTOUT_1);
        prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
		if(count-- == 0){
			return;
		}
	}
#endif

    printf("Test 1: Fan testing...\n");
    printf("\n-------------------------------------------------------------\n");
    printf("Test 1: Fan testing...\n");
    printf("        Please watch the led lights and leds on board\n");
    printf("        1.1 close all fans\n");
    for (idx = 0; idx < num_fans; idx++) {
        fan = fan_lookup_by_index(idx);
        if (fan) {
            fan_disable(fan);
        }
    }

    printf("        1.2 open each fan for 1s then close one by one\n");
    for (idx = 0; idx < num_fans; idx++) {
        fan = fan_lookup_by_index(idx);
        if (fan) {
            printf("         Open %s\n", tag_name(fan));
            fan_enable(fan);
            fan_set_level(fan, 80);
            sleep(1);
            printf("         Close %s\n", tag_name(fan));
            fan_set_level(fan, 0);
            fan_disable(fan);
        }
    }

    printf("        1.3 turn on all fans for 1s, then turn off, repeat 3 times\n");
    for (i = 0; i < 2; i++) {
        for (idx = 0; idx < num_fans; idx++) {
            fan = fan_lookup_by_index(idx);
            if (fan) {
                fan_enable(fan);
                fan_set_level(fan, 80);
            }
        }

        sleep(1);

        for (idx = 0; idx < num_fans; idx++) {
            fan = fan_lookup_by_index(idx);
            if (fan) {
                fan_disable(fan);
            }
        }
        sleep(1);
    }

    printf("-------------------------------------------------------------\n");
    printf("\n-------------------------------------------------------------\n");
    printf("Test 2: LMSW testing...\n");
    printf("        2.1 Please press min x: \n");
    wait_lmsw_min_test(X_AXIS);
    printf("        min_x ok\n");

    printf("        2.2 Please press min y: \n");
    wait_lmsw_min_test(Y_AXIS);
    printf("        min_y ok\n");

    printf("        2.3 Please press min z: \n");
    wait_lmsw_min_test(Z_AXIS);
    printf("        min_z ok\n");

    printf("        2.4 Please press max x: \n");
    wait_lmsw_max(X_AXIS);
    printf("        max_x ok\n");

    printf("        2.5 Please press max y: \n");
    wait_lmsw_max(Y_AXIS);
    printf("        max_y ok\n");

    printf("        2.6 Please press max z: \n");
    wait_lmsw_max(Z_AXIS);
    printf("        max_z ok\n");
    printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 3: Heater testing...\n");
    printf("        open each heater for 1s then close\n");
    printf("        Please watch the led beside the heater interface!\n");

    if (bbp_board_type == BOARD_BBP1S) {
        pwm = pwm_lookup_by_name("pwm_ext");
        if (pwm) {
            pwm_set_output(pwm, 80);
        }
        sleep(1);
        pwm_set_output(pwm, 0);
        sleep(1);

        pwm = pwm_lookup_by_name("pwm_ext2");
        if (pwm) {
            pwm_set_output(pwm, 80);
        }
        sleep(1);
        pwm_set_output(pwm, 0);
        sleep(1);

        pwm = pwm_lookup_by_name("pwm_bed");
        if (pwm) {
            pwm_set_output(pwm, 80);
        }
        sleep(1);
        pwm_set_output(pwm, 0);
        sleep(1);

        for (i = 0; i < 2; i++) {
            pwm = pwm_lookup_by_name("pwm_ext");
            if (pwm) {
                pwm_set_output(pwm, 80);
            }
            pwm = pwm_lookup_by_name("pwm_ext2");
            if (pwm) {
                pwm_set_output(pwm, 80);
            }
            pwm = pwm_lookup_by_name("pwm_bed");
            if (pwm) {
                pwm_set_output(pwm, 80);
            }

            sleep(4);

            pwm = pwm_lookup_by_name("pwm_ext");
            if (pwm) {
                pwm_set_output(pwm, 0);
            }
            pwm = pwm_lookup_by_name("pwm_ext2");
            if (pwm) {
                pwm_set_output(pwm, 0);
            }
            pwm = pwm_lookup_by_name("pwm_bed");
            if (pwm) {
                pwm_set_output(pwm, 0);
            }

            sleep(2);
        }
    }

    if (bbp_board_type == BOARD_BBP1) {
        for (idx = 0; idx < NUM_HEATERS; idx++) {
            heater = heater_lookup_by_index(idx);
            if (heater) {
                printf("         Open %s\n", tag_name(heater));
                heater_enable(heater);
                heater_set_raw_pwm(heater, 40);

                sleep(5);

                printf("         Close %s\n", tag_name(heater));
                heater_set_raw_pwm(heater, 0);
                heater_disable(heater);
            }
        }
    }

    printf("done.\n");
    printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 4: Stepper testing...\n");
    printf("        4.1 Turn on all stepper\n");
    stepper_test();

    printf("        4.2 Stop Stepper\n");
    printf("            Please press min x to stop stepper\n");
    wait_lmsw_min(X_AXIS);
    printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 5: USB Host testing...\n");
    printf("        5.1 please insert U mass storage\n");
	if(unicorn_test_usb_storage() == -1){
		halt_test("Usb mass storage is BAD!!!!!!!!!!!!!!!\n");
	}
	printf("Usb mass storage is OK\n");
	printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 6: Internal emmc testing...\n");
	if(unicorn_test_emmc() == -1){
		halt_test("Internal emmc is BAD!!!!!!!!!!!!!!\n");
	}
	printf("Internal emmc is OK\n");
	printf("done.\n");
	printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 7: eeprom testing...\n");
	if(unicorn_test_eeprom() == -1){
		halt_test("eeprom is BAD!!!!!!!!!!!!\n");
	}
    printf("eeprom is OK\n");
	printf("done.\n");
    printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 8: rtc testing...\n");
	if(unicorn_test_rtc() == -1) {
		halt_test("rtc is BAD!!!!!!!!!!!!\n");
	}
	printf("rtc is OK\n");
	printf("done.\n");
	printf("-------------------------------------------------------------\n");

    printf("\n-------------------------------------------------------------\n");
    printf("Test 9: Network testing...\n");
    printf("        Start to ping the router..\n");
	if(unicorn_test_network() == -1) {
		halt_test("network is BAD!!!!!!!!!!!!\n");
	}
	
    printf("done.\n");
    printf("-------------------------------------------------------------\n");

    printf("Test 10: USB OTG testing...\n");
    printf("        11.1 Please connect usb otg line to PC\n");
    printf("        11.2 Could you see the boot partion? If yes, Please press min_x..\n");
    wait_lmsw_min_test(X_AXIS);
    printf("done.\n");
    printf("-------------------------------------------------------------\n");

	if (bbp_board_type == BOARD_BBP1S) {
        printf("Test 11: Test max6675 temperature...\n");
		printf("long press min y key to exit\n");
		if(unicorn_test_max6675() == -1) {
			halt_test("max6675 is BAD!!!!!!!!!!!!\n");
		}
        printf("\n-------------------------------------------------------------\n");
	}

    #if 0
    if (bbp_board_type == BOARD_BBP1S) {
		#ifdef SERVO
        printf("Test 12: Test servo...\n");
        if (unicorn_test_servo()) {
			halt_test("servo is BAD!!!!!!!!!!!!\n");
        }
        printf("\n-------------------------------------------------------------\n");
		#endif
    }
	#endif


    printf("\n-------------------------------------------------------------\n");
    printf("Test 12: Temp adc testing...\n");
    heater_start();
    if (bbp_board_type == BOARD_BBP1S) {
		char *heater_array_bbp1s[] = {"heater_ext", "heater_ext2", "heater_bed"};
		for (idx = 0; idx < sizeof(heater_array_bbp1s)/sizeof(heater_array_bbp1s[0]); idx++) {
			heater = heater_lookup_by_name(heater_array_bbp1s[idx]);
			if (heater) {
				heater_enable(heater);
				heater_set_setpoint(heater, TARGET_TEMP);
			}
    	}
		for (i = 0; i < 80 && stepper_check_lmsw(X_AXIS) == 0; i++) {
			for (idx = 0; idx < sizeof(heater_array_bbp1s)/sizeof(heater_array_bbp1s[0]); idx++) {
				heater = heater_lookup_by_name(heater_array_bbp1s[idx]);
				if (heater) {
                    celsius = 0.0;
					heater_get_celsius(heater, &celsius);
					printf("%s -> %f\n", tag_name(heater), celsius);
					sleep(1);
				}
			}
		}
	} else if (bbp_board_type == BOARD_BBP1) {
		for (idx = 0; idx < NUM_HEATERS; idx++) {
			heater = heater_lookup_by_index(idx);
			if (heater) {
				heater_enable(heater);
				heater_set_setpoint(heater, TARGET_TEMP);
			}
		}
		printf("long press min x key to exit\n");
		heater = heater_lookup_by_name("heater_bed");
		for (i = 0; i < 80 && stepper_check_lmsw(X_AXIS) == 0; i++) {
			if (heater_temp_reached(heater)) {
				printf("ok\n");
				break;
			} else {
				sleep(1);
			}

			for (idx = 0; idx < NUM_HEATERS; idx++) {
				heater = heater_lookup_by_index(idx);
				if (heater) {
                    celsius = 0.0;
					heater_get_celsius(heater, &celsius);
					printf("%s -> %f\n", tag_name(heater), celsius);
				}
			}
			printf("\n");
		}
	}

    heater_stop();
    printf("done.\n");
    printf("-------------------------------------------------------------\n");

	halt_test("All is Good!\n");

    return 0;
}
