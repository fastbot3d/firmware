/*
 * Unicorn 3D Printer Firmware
 * pruss.h
*/
#ifndef _PRUSS_H
#define _PRUSS_H

#include <stdint.h>

#define PRU_NUM		(0)

#if defined (__cplusplus)
extern "C" {
#endif

extern int pruss_init(void);
extern void pruss_exit(void);

extern int pruss_reset(void);
extern int pruss_disable(void);
extern int pruss_enable(void);

#if defined (__cplusplus)
}
#endif
#endif
