/*--------------------------------------------------------------------
 * pru_queue.h
 * header of pru queue 
 --------------------------------------------------------------------*/
#ifndef _PRU_QUEUE_H_
#define _PRU_QUEUE_H_

#define STATE_EMPTY    (0)
#define STATE_FILLED   (1)
#define STATE_EXIT     (2)

#define QUEUE_LEN      (64)

#define NR_MOTORS      (5)

struct movement {
  /* Speed is steps/second of the axis with the highest number of steps, which
   * is the fastest axis. All other axes are scaled down accordingly.
   */
  float start_speed;
  float travel_speed;
  float end_speed;

  /* steps/s^2 for fastest axis. */
  float acceleration;       

  /* Aux-bits to switch. */
  unsigned char aux_bits;   

  /* Steps for axis. Negative for reverse. */
  int steps[NR_MOTORS]; 
};

/*
 * Initialize pruss motor control
 */
extern int pruss_init(void);
/*
 * Shut down motor control, wait for queue to empty.
 */
extern void pruss_exit(void);
/*
 * Shut down motor control immediately, don't wait for current queue to empty.
 */
void pruss_exit_no_wait(void);

/* Enqueue a coordinated move command.
 * If there is space in the ringbuffer, this function returns immediately,
 * otherwise it waits until a slot frees up.
 * Returns 0 on success, 1 if this is a no-op with no steps to move and 2 if
 * number of steps per single command is exceeded (right now, 64k). If
 * err_stream is non-NULL, prints error message there.
 * Automatically enables motors if not already.
 */
extern int pruss_enqueue(const struct movement *mv);

/* 
 * Wait, until all elements in the ring-buffer are consumed.
 */
extern void pruss_wait_queue_empty(void);

extern void pruss_exit_no_wait(void);

#endif
