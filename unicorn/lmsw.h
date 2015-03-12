/*
 * Unicorn 3D Printer Firmware
 * limit_switch.h
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
