/*
 * Unicorn 3D Printer Firmware
 * stepper_pruss.h
*/
#ifndef _STEPPER_PRUSS_H
#define _STEPPER_PRUSS_H

#include <stepper_spi.h>

#include "common.h"
#include "stepper.h"

#define STATE_EMPTY    (0)
#define STATE_FILLED   (1)
#define STATE_EXIT     (2)

#define STATE_IDLE     (3)
#define STATE_PAUSE    (4)
#define STATE_HOME     (5)
#define STATE_PRINT    (6)
#define STATE_STOP     (7)
#define STATE_RESUME   (8)

#define QUEUE_LEN      (200)

/*
 * Initialize pruss motor control
 */
extern int pruss_stepper_init(void);
/*
 * Shut down pruss motor control, wait for queue to all empty.
 */
extern void pruss_stepper_exit(void);

extern int pruss_queue_move(block_t *block);

extern int pruss_send_cmd(st_cmd_t *cmd);

extern int pruss_queue_wait(void);
extern int pruss_queue_is_full(void);
extern int pruss_queue_get_max_rate(void);
extern int pruss_queue_get_len(void);

#if defined (__cplusplus)
}
#endif
#endif
