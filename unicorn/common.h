/*
 * Unicorn 3D Printer Firmware
 * common.h
 */
#ifndef _COMMON_H
#define _COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include <dirent.h>
#include <pthread.h>
#include <syslog.h>

#include "list.h"
#include "mcode_list.h"

/* EEPROM */
#define EEPROM_DEV          		"/sys/bus/i2c/devices/1-0057/eeprom"
#define PRU_UPLOAD_FIRMWARE_PATH 	"/.octoprint/pru.bin"

#define AUTO_LEVEL_BIN	 	"/autolevel.bin"

#define ERROR_MIN_ADC  10
#define ERROR_MAX_ADC  4090
#define ERROR_MIN_THERMOCOUPLE 10
#define ERROR_MAX_THERMOCOUPLE 4090

#define CONFIG_GPIO_NODE "/sys/class/stepper_spi_class/config_gpio"
#define READ_GPIO_NODE "/sys/class/stepper_spi_class/gpio"
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

#define D_MIN        0x0000
#define D_COMM       0x0001
#define D_GCODE      0x0002
#define D_STEPPER    0x0004
#define D_PID        0x0008
#define D_HEATER     0x0010
#define D_PWM        0x0020
#define D_TEMP       0x0040
#define D_ANALOG     0x0080
#define D_FAN        0x0100
#define D_LMSW       0x0200
#define D_HOME       0x0400
#define D_PRUSS      0x0800
#define D_SERVO      0x1000
#define D_PLAN       0x2000
#define D_MAX        0x4000

#define DBG(x) ( (x) && ((debug & (x)) == (x)) )

#if 0
	//for debug background
     syslog(LOG_DEBUG, "[COMM]:"x, ##__VA_ARGS__);   \
     syslog(LOG_DEBUG, "[GCODE]:"x, ##__VA_ARGS__);   \
     syslog(LOG_DEBUG, "[STEPPER]:"x, ##__VA_ARGS__);   \

#endif

#define MIN_DBG(x, ...)                 \
        if (DBG(D_MIN)) {               \
            printf("[MIN]:"x, ##__VA_ARGS__);   \
        }                   

#define COMM_DBG(x, ...)                 \
        if (DBG(D_COMM)) {               \
            printf("[COMM]:"x, ##__VA_ARGS__);   \
        }                   

#define GCODE_DBG(x, ...)                 \
        if (DBG(D_GCODE)) {               \
            printf("[GCODE]:"x, ##__VA_ARGS__);   \
        }                   

#define STEPPER_DBG(x, ...)                 \
        if (DBG(D_STEPPER)) {               \
            printf("[STEPPER]:"x, ##__VA_ARGS__);   \
        }                   

#define PID_DBG(x, ...)                 \
        if (DBG(D_PID)) {               \
            printf("[PID]:"x, ##__VA_ARGS__);   \
        }                   

#define HEATER_DBG(x, ...)                 \
        if (DBG(D_HEATER)) {               \
            printf("[HEATER]:" x, ##__VA_ARGS__);   \
        }                   

#define PWM_DBG(x, ...)                 \
        if (DBG(D_PWM)) {               \
            printf("[PWM]:"x, ##__VA_ARGS__);   \
        }                   

#define TEMP_DBG(x, ...)                 \
        if (DBG(D_TEMP)) {               \
            printf("[TEMP]:"x, ##__VA_ARGS__);   \
        }                   

#define ANALOG_DBG(x, ...)                 \
        if (DBG(D_ANALOG)) {               \
            printf("[ANALOG]:"x, ##__VA_ARGS__);   \
        }                   

#define FAN_DBG(x, ...)                 \
        if (DBG(D_FAN)) {               \
            printf("[FAN]:"x, ##__VA_ARGS__);   \
        }                   

#define LMSW_DBG(x, ...)                 \
        if (DBG(D_LMSW)) {               \
            printf("[LMSW]:"x, ##__VA_ARGS__);   \
        }                   

#define HOME_DBG(x, ...)                 \
        if (DBG(D_HOME)) {               \
            printf("[HOME]:"x, ##__VA_ARGS__);   \
        }                   

#define PRUSS_DBG(x, ...)                 \
        if (DBG(D_PRUSS)) {               \
            printf("[PRUSS]:"x, ##__VA_ARGS__);   \
        }                   

#define SERVO_DBG(x, ...)                 \
        if (DBG(D_SERVO)) {               \
            printf("[SERVO]:"x, ##__VA_ARGS__);   \
        }                   

#define PLAN_DBG(x, ...)                 \
        if (DBG(D_PLAN)) {               \
            printf("[PLAN]:"x, ##__VA_ARGS__);   \
        }                   

extern volatile uint32_t debug;

#define constrain(val,low,high) ((val)<(low)?(low):((val)>(high)?(high):(val)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define NR_ITEMS(x) (sizeof((x))/sizeof(*(x)))

struct sub_system {
    const char *name;
    int (*init)(void);
    void (*exit)(void);
};

#define SUB_SYS_TAG(name) static const char name[] = #name
#define GENERATE_TAG(name) static const char name[] = #name

typedef const char *channel_tag;

typedef enum {
    X_AXIS   = 0,
    Y_AXIS   = 1,
    Z_AXIS   = 2,
    E_AXIS   = 3,
    E2_AXIS  = 4,
    E3_AXIS  = 5,
    U_AXIS   = 6,
    U2_AXIS  = 7,

    MAX_X_AXIS= 8,
    MAX_Y_AXIS= 9,
    MAX_Z_AXIS= 10,
    AUTOLEVEL_Z_AXIS= 11,
    
    MAX_AXIS = 12,
} axis_e;

typedef enum {
    BOARD_BBP1 = 1,
    BOARD_BBP1S = 2,
    BOARD_UNKNOW = -1,
} BBP_BOARD_TYPE;
extern int bbp_board_type;

static inline bool check_axis_valid(axis_e axis)
{
    if (axis >= X_AXIS && axis < MAX_AXIS) {
        return true;
    } 
    
    return false;
}

#define NSEC_PER_SEC     (1000000000)

#if defined (__cplusplus)
extern "C" {
#endif

static inline const char* tag_name (channel_tag tag)
{
    return (char *)tag;
}

static inline char axis_name(axis_e axis)
{
    char names[] = {'X', 'Y', 'Z', 'E', };
    return names[axis];
}

extern void timestamp_init(void);
extern double timestamp_get(void);

extern const char* sys_path_finder(char *buffer, size_t max_size, const char *path);

extern int sub_sys_thread_create(const char *name, 
                                 pthread_t *thread, 
                                 const pthread_attr_t *attr,
                                 void *(*worker_thread)(void *), 
                                 void *arg);

extern int sub_sys_init(const char *name, int (*init)(void));
extern void sub_sys_exit(const char *name, void (*exit)(void));

extern int pwm_write_sysfs(const char *path, const char *file, int value);
extern int pwm_read_sysfs(const char *path, const char *file, int *value);

extern int analog_write_sysfs(const char *path, int value);
extern int analog_read_sysfs(const char *path, int *value);

extern int gpio_request_sysfs(unsigned int gpio);
extern int gpio_free_sysfs(unsigned int gpio);

extern int gpio_write_sysfs(unsigned int gpio, const char *file, char *value);
extern int gpio_read_sysfs(unsigned int gpio, const char *file, char *value);

extern unsigned long data_crc(void *data, int size);
extern unsigned long calculate_pru_file_crc(char *filename);
extern int read_max6675_thermocouple(double *celsius);
extern int read_ad597_thermocouple(char *path, double *celsius);
extern int set_gpio(int gpio, int direction, int level);
extern int read_gpio(int gpio);
extern void save_autolevel(float autolevel[], int count);
extern void load_autolevel(float autolevel[], int count);

#if defined (__cplusplus)
}
#endif
#endif
