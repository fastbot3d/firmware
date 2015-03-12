/*
 * Unicorn 3D Printer Firmware
 * gcode.h
 * G-Code Interpreter
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
#ifndef _GCODE_H
#define _GCODE_H

#include "common.h"

#if defined (__cplusplus)
extern "C" {
#endif

extern int gcode_init(void);
extern void gcode_exit(void);

extern void gcode_set_feed(int multiply);
extern void gcode_stop(void);

extern void *gcode_get_line_from_file(FILE *fp);
extern int gcode_process_line(void);

#if defined (__cplusplus)
}
#endif
#endif
