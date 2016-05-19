/*
 * unicorn 3D Printer Firmware
 * unicorn.h
*/
#ifndef _UNICORN_H
#define _UNICORN_H

#if defined (__cplusplus)
extern "C" {
#endif

extern int unicorn_init(void);
extern void unicorn_exit(int blocking);

extern int unicorn_pause(void);
extern int unicorn_resume(void);

extern int unicorn_start(void);
extern int unicorn_stop(void);

extern void unicorn_sync(void);

extern int unicorn_print(char *file);

#if defined (__cplusplus)
}
#endif
#endif
