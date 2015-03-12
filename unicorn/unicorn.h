/*
 * unicorn 3D Printer Firmware
 * unicorn.h
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
