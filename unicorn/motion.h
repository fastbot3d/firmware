/*
 * Unicorn 3D Printer Firmware
 * motion.h
*/
#ifndef _MOTION_H
#define _MOTION_H

#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION   25


#if defined (__cplusplus)
extern "C" {
#endif

/*
 * Execute an arc in offset mode format.
 * position    -> current xyz;
 * target      -> target xyz;
 * offset      -> offset from current xyz;
 * axis_xxx    -> defines circle plane in tool space, 
 *                axis_linear is the direction of helical travel;
 * radius      -> circle radius;
 * isclockwise -> boolean, whether is clockwise,
 *                used for vector transformation direction.
 */
extern void mc_arc(float *position, float *target, float *offset, 
                   uint8_t axis_0, uint8_t axis_1, uint8_t axis_1inear,
                   float feed_rate, float radius,
                   uint8_t isclockwise, uint8_t extruder);

#if defined (__cplusplus)
}
#endif
#endif
