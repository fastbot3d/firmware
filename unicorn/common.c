/*
 * Unicorn 3D Printer Firmware
 * common.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
//#include <glob.h> android don't have
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

#if 0  //for android
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
            COMM_DBG("sys_path_finder( '%s') returns '%s'\n", path, buffer);
        }
    }

    globfree(&globbuf);
#endif
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
    COMM_DBG("--- Creating %s_thread...", name); 

    ret = pthread_create(thread, attr, worker_thread, arg);
    if (ret) {
        fprintf(stderr, "create thread : failed with ret %d\n", ret);
    } else {
        COMM_DBG("done ---\n");
    }

    return ret;
}

int sub_sys_init(const char *name, int (*init)(void))
{
    int ret = 0;
    COMM_DBG("--- Starting '%s' init ...\n", name); 

    ret = init();
    if (ret) {
        fprintf(stderr, "sub sys init: failed with ret %d\n", ret);
    } else {
        COMM_DBG("done!\n");
    }

    return ret;
}

void sub_sys_exit(const char *name, void (*exit)(void))
{
    COMM_DBG("--- Quiting '%s' exit ...\n", name); 

    //FIXME
    //exit();
    
    COMM_DBG( "done!\n");
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
    int fp;
    int ret;
    int len;
    char *tok;

    fp = open(file, O_RDONLY);

    if (fp <= 0) {
        printf("Failed to open %s for reading\n", file);
        return -1;
    }

    memset(val, '\0', length);

    ret = read(fp, val, length);
    if (ret < 1) {
        //printf("Failed to read sysfs variable from %s\n", file);
        printf("Failed to read %s\n", file);
		close(fp);
        return -1;
    }

    tok = strtok(val, "\n");
    len = tok ? strlen(tok) : strlen(val);
    val[len] = '\0';

    close(fp);

    return 0;
}

int pwm_write_sysfs(const char *path, const char *file, int value)
{
    char fn[100];
    //FIXME: Please check it out!
    //char val[8];
    char val[12];
    
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

int read_ad597_thermocouple(char *path, double *celsius)
{
	int temp;

	if (path == NULL){
		return 0;
	}

	analog_read_sysfs(path, &temp);

	*celsius = temp * 1800.0f/4096 * 6.1f / 10; // 10 mv per celsius

	// thermistor connection wire is break.
	if(temp > ERROR_MAX_THERMOCOUPLE){
		*celsius = -1000.0f;
		return -1;
	}

	//return temp;
	return 0;
}

int read_max6675_thermocouple(double *celsius)
{
	int temp;

	analog_read_sysfs(AIN_CH_EXT3, &temp);
	*celsius = temp/4.0f;

	// thermistor connection wire is break.
	if(temp > ERROR_MAX_THERMOCOUPLE){
		*celsius = -1000.0f;
		return -1;
	}

	//return temp;
	return 0;
}

static int gpio_write_file(const char *file, char *value)
{
    char fn[100];
    snprintf(fn, sizeof(fn), "/sys/class/gpio/%s", file);
    return write_sysfs(fn, value);
}

int gpio_write_sysfs(unsigned int gpio, const char *file, char *value)
{
    char fn[100];

    snprintf(fn, sizeof(fn), "gpio%d/%s", gpio, file);
    return gpio_write_file(fn, value);
}

int gpio_read_sysfs(unsigned int gpio, const char *file, char *value)
{
    return 0;
}

int gpio_request_sysfs(unsigned int gpio)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", gpio);
    return gpio_write_file("export", buf);
}

int gpio_free_sysfs(unsigned int gpio)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", gpio);
    return gpio_write_file("unexport", buf);
}

unsigned long data_crc(void *data, int size)
{
	unsigned long crc = 0;
	unsigned int *crc_data=(unsigned int*)data;
	int total_pos = (size / 4) * 4;
	int cur_pos = 0;

	if(total_pos > 0 && total_pos < 4) {
		return 	*(unsigned char*)data;
	}
	while(cur_pos < total_pos){
		crc += 	*crc_data;
		cur_pos +=4;
		crc_data++;
	}
	return crc;
}

unsigned long calculate_pru_file_crc(char *filename)
{
    int ret = 0;
    int fd = -1;
    uint32_t count;
    uint32_t data[2048];
    
    memset(&data, 0xFF, sizeof(data));
	fd = open(filename, O_RDONLY);
	if(fd < 0){
		printf("file %s don't exist\n", filename);
    	ret = 0;
		return ret;
	}

    count = read(fd, (void *)data, (uint32_t)sizeof(data));
    if (count <= 0) {
        ret = 0;
        goto out;
    }

	ret = data_crc(data, 1024 * 8);

out:
    if (fd > 0) {
        close(fd);
    }
	return ret;
}

//sys/class/stepper_spi_class/config_gpio
//echo "gpio,direction,level"  -> 0 for input, 1 for output.  0 for low level, 1 for high level
int set_gpio(int gpio, int direction, int level)
{
	int fd;
	int ret;
	char buf[30] = {0};

	if (gpio < 0 || gpio > 128) {
		printf("write gpio is out of range\n");
		return -1;
	}
	if(((direction != 0) && (direction !=1)) || ((level != 0) && (level != 1))){
		printf("gpio,direction,level -> 0 for input, 1 for output.  0 for low level, 1 for high level\n");
		return -1;
	}

	fd = open(CONFIG_GPIO_NODE, O_WRONLY);
	if (fd <= 0) {
		printf("Failed to open %s for writing\n", CONFIG_GPIO_NODE);
		return -1;
	}

	sprintf(buf, "%d,%d,%d", gpio, direction, level);

	ret = write(fd, buf, strlen(buf));
	if (ret < 1) {
		printf("Failed to write %s \n", CONFIG_GPIO_NODE);
		close(fd);
		return -1;
	}

	close(fd);

	return 0;
}

int read_gpio(int gpio)
{
    int fd;
    int ret;
    char buf[30] = {0};

	if (gpio < 0 || gpio > 128) {
		printf("read gpio is out of range\n");
		return -1;
	}

    fd = open(READ_GPIO_NODE, O_WRONLY);
    if (fd <= 0) {
        printf("Failed to open %s for writing\n", READ_GPIO_NODE);
        return -1;
    }

    sprintf(buf, "%d", gpio);
    ret = write(fd, buf, strlen(buf));
    if (ret < 1) {
        printf("Failed to write %s \n", READ_GPIO_NODE);
		close(fd);
        return -1;
    }
    printf("write ret:%d\n", ret);
	close(fd);

	memset(buf, 0, sizeof(buf));
    fd = open(READ_GPIO_NODE, O_RDONLY);
    ret = read(fd, buf, 1);
    if (ret < 1) {
        printf("Failed to read %s \n", READ_GPIO_NODE);
		close(fd);
        return -1;
    }

	close(fd);

    printf("read buf:%s\n", buf);
	ret = atoi(buf);

    printf("read ret:%d\n", ret);

	return ret;
}

void save_autolevel(float *autolevel, int count)
{
	int fd = open(AUTO_LEVEL_BIN, O_WRONLY | O_CREAT);
	write(fd, autolevel, count);
	close(fd);
}


void load_autolevel(float *autolevel, int count)
{
	int fd = open(AUTO_LEVEL_BIN, O_RDONLY);
	if (fd > 0) {
		read(fd, autolevel, count);
		close(fd);
	}
}



