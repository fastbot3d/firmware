/*
 * Unicorn 3D Printer Firmware
 * sdcard.h
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
#ifndef _SDCARD_H
#define _SDCARD_H

#include "common.h"

void sdcard_list_files();
void sdcard_select_file(const char* name);
void sdcard_capture_start();
void sdcard_capture_stop();
unsigned char sdcard_iscapturing();
unsigned char sdcard_write_line(const char* line);
void sdcard_set_position(unsigned int filepos);
void sdcard_print_status();
int sdcard_get_char(unsigned char* chr);
void sdcard_replay_start();
void sdcard_replay_pause();
void sdcard_replay_stop();
int sdcard_isreplaying();
int sdcard_isreplaypaused();
void sdcard_handle_state();
void sdcard_mount();
void sdcard_unmount();
unsigned char sdcard_ismounted();
unsigned char sdcard_card_detected();

#if defined (__cplusplus)
extern "C" {
#endif







#if defined (__cplusplus)
}
#endif
#endif
