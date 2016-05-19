/*
 * Unicorn 3D Printer Firmware
 * common.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <glob.h>
#include <pthread.h>
#include <time.h>

#include "common.h"

volatile uint32_t debug = D_INIT;

#ifdef _POSIX_MONOTONIC_CLOCK
static clockid_t clk = CLOCK_MONOTONIC;
#else
# error NO SUITABLE CLOCK SOURCE AVAILABLE
#endif

static struct timespec t0;

void timestamp_init(void)
{
    clock_gettime(clk, &t0);
}

double timestamp_get(void)
{
    struct timespec t1;
    clock_gettime(clk, &t1);
    int nsecs = t1.tv_nsec - t0.tv_nsec;
    int secs  = t1.tv_sec - t0.tv_sec;
    return secs + 0.000000001 * nsecs;
}
/*
 *  Use glob to find sys device paths with suffixes.
 *  Check return value or result in buffer for success.
 */
const char* sys_path_finder(char *buffer, size_t max_size, const char *path)
{
    char *ret = NULL;
    glob_t globbuf;

    int result = glob(path, GLOB_ERR | GLOB_NOSORT, NULL, &globbuf);
    if (result == GLOB_NOMATCH) {
        *buffer = '\0';
        ret = NULL;
        fprintf(stderr, "sys_path_finder: %s none found\n", path);
    } else {
        if (globbuf.gl_pathc != 1) {
            *buffer = '\0';
            ret = NULL;
            fprintf(stderr, "sys_path_finder: %s more than one found\n", path);
        } else {
            strncpy( buffer, globbuf.gl_pathv[ 0], max_size);
            ret = buffer;
            printf( "sys_path_finder( '%s') returns '%s'\n", path, buffer);
        }
    }

    globfree(&globbuf);
    return ret;
}

/* 
 * sub system function caller
 */
int sub_sys_thread_create(const char *name, 
                          pthread_t *thread, 
                          const pthread_attr_t *attr,
                          void *(*worker_thread)(void *), 
                          void *arg)
{
    int ret = 0; 
    fprintf(stderr, "--- Creating %s_thread...", name); 
    
    ret = pthread_create(thread, attr, worker_thread, arg);
    if (ret) {
        fprintf(stderr, "create thread : failed with ret %d\n", ret);
    } else {
        fprintf(stderr, "done ---\n");
    }

    return ret;
}

int sub_sys_init(const char *name, int (*init)(void))
{
    int ret = 0;
    fprintf(stderr, "--- Starting '%s' init ...\n", name); 
    
    ret = init();
    if (ret) {
        fprintf(stderr, "sub sys init: failed with ret %d\n", ret);
    } else {
        fprintf(stderr, "done!\n");
    }

    return ret;
}

void sub_sys_exit(const char *name, void (*exit)(void))
{
    fprintf(stderr, "--- Quiting '%s' exit ...\n", name); 
    exit();
    fprintf(stderr, "done!\n");
}

/*
 *  sysfs interface
 */
static int write_sysfs(char *file, char *val)
{
    FILE *fp;
    char *valString;

    valString = malloc(strlen(val) + 1);

    if (valString == NULL) {
        printf("Failed to allocate memory for temporary string\n");
        return -1;
    }

    fp = fopen(file, "w");

    if (fp == NULL) {
        printf("Failed to open %s for writing\n", file);
        free(valString);
        return -1;
    }

    if (fwrite(val, strlen(val) + 1, 1, fp) != 1) {
        printf("Failed to write sysfs variable %s to %s\n",
                  file, val);
        fclose(fp);
        free(valString);
        return -1;
    }

    fclose(fp);

    free(valString);

    return 0;
}

static int read_sysfs(char *file, char *val, int length)
{
    FILE *fp;
    int ret;
    int len;
    char *tok;

    fp = fopen(file, "r");

    if (fp == NULL) {
        printf("Failed to open %s for reading\n", file);
        return -1;
    }

    memset(val, '\0', length);

    ret = fread(val, 1, length, fp);
    if (ret < 1) {
        printf("Failed to read sysfs variable from %s\n", file);
        return -1;
    }

    tok = strtok(val, "\n");
    len = tok ? strlen(tok) : strlen(val);
    val[len] = '\0';

    fclose(fp);

    return 0;
}

int pwm_write_sysfs(const char *path, const char *file, int value)
{
    char fn[100];
    char val[8];
    
    snprintf(fn, sizeof(fn), "%s/%s", path, file);
    sprintf(val, "%d", value);
    
    return write_sysfs(fn, val);
}

int pwm_read_sysfs(const char *path, const char *file, int *value)
{
    int ret = 0;
    char fn[100], val[8];

    snprintf(fn, sizeof(fn), "%s/%s", path, file);
    
    ret = read_sysfs(fn, val, 8);
    if (ret)  {
        printf("Failed to read '%s'\n", fn);
        return -1;
    }

    *value = atoi(val);
    return 0;
}

int analog_write_sysfs(const char *path, int value)
{
    char fn[100];
    char val[8];
    
    snprintf(fn, sizeof(fn), "%s", path);
    sprintf(val, "%d", value);
    return write_sysfs(fn, val);
}

int analog_read_sysfs(const char *path, int *value)
{
    int ret = 0;
    char fn[100], val[8];

    snprintf(fn, sizeof(fn), "%s", path);
    
    ret = read_sysfs(fn, val, 8);
    if (ret)  {
        printf("Failed to read '%s'\n", fn);
        return -1;
    }

    *value = atoi(val);
    return 0;
}

