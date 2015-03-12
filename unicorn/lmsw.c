/*
 * Unicorn 3D Printer Firmware
 * limit_switch.c
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
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>

#include <pthread.h>
#include <linux/input.h>

#include "unicorn.h"
#include "lmsw.h"

#define MIN_X_DEV "/dev/input/event0"
#define MIN_Y_DEV "/dev/input/event1"
#define MIN_Z_DEV "/dev/input/event2"
#define MAX_X_DEV "/dev/input/event3"
#define MAX_Y_DEV "/dev/input/event4"
#define MAX_Z_DEV "/dev/input/event5"

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
        }
        if (ev[i].code == KEY_F2 && axis == Y_AXIS) {
            limited = 1;
        }
        if (ev[i].code == KEY_F3 && axis == Z_AXIS) {
            limited = 1;
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

int lmsw_init(void)
{
    int ret = 0;

    if DBG(D_LMSW) {
        printf("lmsw_init called.\n");
    }
    
    /* open lmsw input event device */
    min_x_fd = open(MIN_X_DEV, O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_min_x input device failed\n");
        ret = -1;
    }
    min_y_fd = open(MIN_Y_DEV, O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_min_y input device failed\n");
        ret = -1;
    }
    min_z_fd = open(MIN_Z_DEV, O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_min_z input device failed\n");
        ret = -1;
    }

    max_x_fd = open(MAX_X_DEV, O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_max_x input device failed\n");
        ret = -1;
    }
    max_y_fd = open(MAX_Y_DEV, O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_max_y input device failed\n");
        ret = -1;
    }
    max_z_fd = open(MAX_Z_DEV, O_RDONLY);
    if (min_x_fd < 0) {
        fprintf(stderr, "open lmsw_max_z input device failed\n");
        ret = -1;
    }

    return ret;
}

void lmsw_exit(void)
{
    if DBG(D_LMSW) {
        printf("lmsw_exit called.\n");
    }

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

