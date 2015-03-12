/*
 * Unicorn 3D Printer Firmware
 * motion.h
 * High level interface for issuing motion commands
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
