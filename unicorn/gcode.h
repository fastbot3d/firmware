/*
 * Unicorn 3D Printer Firmware
 * gcode.h
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
