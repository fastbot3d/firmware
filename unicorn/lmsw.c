/*
 * Unicorn 3D Printer Firmware
 * limit_switch.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <pthread.h>
#include <linux/input.h>

#include "unicorn.h"
#include "lmsw.h"

#define MIN_X_DEV "/dev/input/by-path/platform-lmsw_min_x-event"
#define MIN_Y_DEV "/dev/input/by-path/platform-lmsw_min_y-event"
#define MIN_Z_DEV "/dev/input/by-path/platform-lmsw_min_z-event"
#define MAX_X_DEV "/dev/input/by-path/platform-lmsw_max_x-event"
#define MAX_Y_DEV "/dev/input/by-path/platform-lmsw_max_y-event"
#define MAX_Z_DEV "/dev/input/by-path/platform-lmsw_max_z-event"

int min_x_fd = 0;
int min_y_fd = 0;
int min_z_fd = 0;
int max_x_fd = 0;
int max_y_fd = 0;
int max_z_fd = 0;

/*
 * Check lmsw active
 * blocking waiting for gpio irq
 * return 0, if success, otherwise return -1
 */
int wait_lmsw_min(axis_e axis)
{
    int i, rd;
    int fd = -1;
    int limited = 0;
    struct input_event ev[64];
    struct timeval timeout;
	fd_set fds;
    
    if (!check_axis_valid(axis)) {
        return -1;
    }

    switch (axis) {
    case X_AXIS:
        fd = min_x_fd;
        break;
    case Y_AXIS:
        fd = min_y_fd;
        break;
    case Z_AXIS:
        fd = min_z_fd;
        break;
    default:
        break;
    }

    if (fd < 0) {
        return -1;
    }


	FD_ZERO(&fds);
	FD_SET(fd,  &fds);
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;

	rd = select(fd + 1, &fds, 0, 0, &timeout);
	if (rd <= 0) {
        printf("read lmsw event timeout\n");
		return -1;
	}

    rd = read(fd, ev, sizeof(struct input_event) * 64);
    if (rd < (int) sizeof(struct input_event)) {
        fprintf(stderr, "read lmsw event err\n");
        return -1;
    }
    
    for (i = 0; i < rd / sizeof(struct input_event); i++) {
        if (ev[i].code == KEY_F1 && axis == X_AXIS) {
            limited = 1;
			break;
        }
        if (ev[i].code == KEY_F2 && axis == Y_AXIS) {
            limited = 1;
			break;
        }
        if (ev[i].code == KEY_F3 && axis == Z_AXIS) {
            limited = 1;
			break;
        }
    }

    return limited;
}

int wait_lmsw_min_test(axis_e axis)
{
    int i, rd;
    int fd = -1;
    int limited = 0;
    struct input_event ev[64];
    
    if (!check_axis_valid(axis)) {
        return -1;
    }

    switch (axis) {
    case X_AXIS:
        fd = min_x_fd;
        break;
    case Y_AXIS:
        fd = min_y_fd;
        break;
    case Z_AXIS:
        fd = min_z_fd;
        break;
    default:
        break;
    }

    if (fd < 0) {
        return -1;
    }

    rd = read(fd, ev, sizeof(struct input_event) * 64);
    if (rd < (int) sizeof(struct input_event)) {
        fprintf(stderr, "read lmsw event err\n");
        return -1;
    }
    
    for (i = 0; i < rd / sizeof(struct input_event); i++) {
        if (ev[i].code == KEY_F1 && axis == X_AXIS) {
            limited = 1;
			break;
        }
        if (ev[i].code == KEY_F2 && axis == Y_AXIS) {
            limited = 1;
			break;
        }
        if (ev[i].code == KEY_F3 && axis == Z_AXIS) {
            limited = 1;
			break;
        }
    }

    return limited;
}

int wait_lmsw_max(axis_e axis)
{
    int i, rd;
    int fd = -1;
    int limited = 0;
    struct input_event ev[64];

    if (!check_axis_valid(axis)) {
        return -1;
    }
    
    switch (axis) {
    case X_AXIS:
        fd = max_x_fd;
        break;
    case Y_AXIS:
        fd = max_y_fd;
        break;
    case Z_AXIS:
        fd = max_z_fd;
        break;
    default:
        break;
    }

    if (fd < 0) {
        return -1;
    }

    rd = read(fd, ev, sizeof(struct input_event) * 64);
    if (rd < (int) sizeof(struct input_event)) {
        fprintf(stderr, "read lmsw event err\n");
        return -1;
    }
    
    for (i = 0; i < rd / sizeof(struct input_event); i++) {
        if (ev[i].code == KEY_F4 && axis == X_AXIS) {
            limited = 1;
        }
        if (ev[i].code == KEY_F5 && axis == Y_AXIS) {
            limited = 1;
        }
        if (ev[i].code == KEY_F6 && axis == Z_AXIS) {
            limited = 1;
        }
    }

    return limited;
}

//for android
static char *input_name[] = {"lmsw_min_x", "lmsw_min_y", "lmsw_min_z", "lmsw_max_x", "lmsw_max_y", "lmsw_max_z"};
static char *input_dev[6];  
static char *input_dev_default[] = {"/dev/input/by-path/platform-lmsw_min_x-event"
								, "/dev/input/by-path/platform-lmsw_min_y-event"
								, "/dev/input/by-path/platform-lmsw_min_z-event"
								, "/dev/input/by-path/platform-lmsw_max_x-event"
								, "/dev/input/by-path/platform-lmsw_max_y-event"
								, "/dev/input/by-path/platform-lmsw_max_z-event" };

static void lmsw_search_dev(void)
{
	char buf[3096] = {0};
	int fd = -1;
	int i = 0;

	for (i=0; i<sizeof(input_name)/sizeof(input_name[0]); i++) {
			input_dev[i] = malloc(100);
			if (!input_dev[i]) {
				printf("failed to malloc for input\n");
			}
			memset(input_dev[i], 0, 100);
			strcpy(input_dev[i], "/dev/input/");
	}

	fd = open("/proc/bus/input/devices", O_RDONLY);
	read(fd, buf, sizeof(buf));

	for (i=0; i<sizeof(input_name)/sizeof(input_name[0]); i++) {
			char *search_name = strstr(buf, input_name[i]);
			if (search_name) {
				char *search_dev= strstr(search_name, "event");
				if (search_dev) {
						char *end = search_dev;
						while (true) {
							if( ( *(end+1) == ' ') || (*(end+1) == '\0') || ( *(end+1) == '\n'))
								break;
							end++;
						}
						strncpy(input_dev[i] + strlen(input_dev[i]), search_dev, end - search_dev + 1);
						printf("i=%d, dev=%s\n", i, input_dev[i]);
				}
			} else {
				strcpy(input_dev[i], input_dev_default[i]);
				printf("use default, i=%d, dev=%s\n", i, input_dev[i]);
			}
	}
}

int lmsw_init(void)
{
    int ret = 0;

    //if DBG(D_LMSW) {
        printf("lmsw_init called.\n");
    //}
	lmsw_search_dev();
    
    /* open lmsw input event device */
    min_x_fd = open(input_dev[0], O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_min_x input device failed\n");
        ret = -1;
    }
    min_y_fd = open(input_dev[1], O_RDONLY);
    if (min_y_fd < 0) {
        fprintf(stderr, "open lmsw_min_y input device failed\n");
        ret = -1;
    }
    min_z_fd = open(input_dev[2], O_RDONLY);
    if (min_z_fd < 0) {
        fprintf(stderr, "open lmsw_min_z input device failed\n");
        ret = -1;
    }

    max_x_fd = open(input_dev[3], O_RDONLY);
    if (max_x_fd < 0) {
        fprintf(stderr, "open lmsw_max_x input device failed\n");
        ret = -1;
    }
    max_y_fd = open(input_dev[4], O_RDONLY);
    if (max_y_fd < 0) {
        fprintf(stderr, "open lmsw_max_y input device failed\n");
        ret = -1;
    }
    max_z_fd = open(input_dev[5], O_RDONLY);
    if (max_z_fd < 0) {
        fprintf(stderr, "open lmsw_max_z input device failed\n");
        ret = -1;
    }

    return ret;
}

void lmsw_exit(void)
{
    //if DBG(D_LMSW) {
        printf("lmsw_exit called.\n");
    //}

    /* close lmsw input event device */
    if (min_x_fd) {
        close(min_x_fd);
    }
    if (min_y_fd) {
        close(min_y_fd);
    }
    if (min_z_fd) {
        close(min_z_fd);
    }

    if (max_x_fd) {
        close(max_x_fd);
    }
    if (max_y_fd) {
        close(max_y_fd);
    }
    if (max_z_fd) {
        close(max_z_fd);
    }
}
