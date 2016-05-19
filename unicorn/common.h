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

#ifdef DEBUG
#define D_GC_PROC    0x0001
#define D_LMSW       0x0002
#define D_HOME       0x0004
#define D_TRAJECT    0x0008
#define D_PRUSS      0x0010
#define D_POSITION   0x0020
#define D_HEATER     0x0040
#define D_PID        0x0080
#define D_PWM        0x0100
#define D_TEMP       0x0200
#define D_ANALOG     0x0400
#define D_VERBOSE    0x0800
#define D_COMM       0x1000
#define D_FAN        0x2000
#define D_LED        0x4000
#define D_STEPPER    0x8000
#else
#define D_GC_PROC    0
#define D_LMSW       0
#define D_HOME       0
#define D_TRAJECT    0
#define D_PRUSS      0
#define D_POSITION   0
#define D_HEATER     0
#define D_PID        0
#define D_PWM        0
#define D_TEMP       0 
#define D_ANALOG     0
#define D_VERBOSE    0
#define D_COMM       0
#define D_FAN        0
#define D_LED        0
#define D_STEPPER    0
#endif

#define DBG(x) ( (x) && ((debug & (x)) == (x)) )
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
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2,
    E_AXIS = 3,
    E2_AXIS = 4,
} axis_e;

static inline bool check_axis_valid(axis_e axis)
{
    if (axis >= X_AXIS && axis <= E2_AXIS) {
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
    char names[] = {'X', 'Y', 'Z', 'E', 'E',}; //FIXME
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

#if defined (__cplusplus)
}
#endif
#endif
