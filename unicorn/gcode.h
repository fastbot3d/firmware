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

extern int gcode_start(int fd_rd, int fd_wr, int fd_rd_emerg);
extern void gcode_stop(void);

extern void *gcode_get_line_from_file(FILE *fp);
extern int gcode_get_line_from_remote(void);

extern int gcode_send_response_remote(char *str);
extern int gcode_send_byte_response_remote(char *str, int size);
extern int gcode_process_line(char *line, bool send_ok);
extern int gcode_process_multi_line(char *multi_line);
extern int gcode_process_line_from_file();

extern void gcode_set_extruder_feed(int multiply);

#if defined (__cplusplus)
}
#endif
#endif
