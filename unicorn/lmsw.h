/*
 * Unicorn 3D Printer Firmware
 * limit_switch.h
*/
#ifndef _LMSW_H
#define _LMSW_H

#include "common.h"
#include "unicorn.h"

#if defined (__cplusplus)
extern "C" {
#endif

extern int wait_lmsw_min(axis_e axis);
extern int wait_lmsw_max(axis_e axis);

extern int lmsw_init(void);
extern void lmsw_exit(void);

#if defined (__cplusplus)
}
#endif
#endif
