/*
 * unicorn 3D Printer Firmware
 * unicorn.h
*/
#ifndef _UNICORN_H
#define _UNICORN_H

#include <stdbool.h>

#if defined (__cplusplus)
extern "C" {
#endif

#define FW_MODE_MIN     0
#define FW_MODE_LOCAL   1
#define FW_MODE_REMOTE  2
#define FW_MODE_TEST    3
#define FW_MODE_MAX     4

extern int unicorn_init(void);
extern void unicorn_exit(int blocking);

extern int unicorn_pause(void);
extern int unicorn_resume(void);

extern int unicorn_start(int fd_rd, int fd_wr, int fd_rd_emerg);
#if 1
extern int unicorn_stop(bool blocking);
#else
extern int unicorn_stop(void);
#endif
extern int unicorn_disconnect_octoprint();
extern int unicorn_restart(void);

extern int unicorn_set_mode(int mode);
extern int unicorn_get_mode(void);
extern void unicorn_set_feedrate(int multiply);

extern int unicorn_test(void);

#if defined (__cplusplus)
}
#endif
#endif
