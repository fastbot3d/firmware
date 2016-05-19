/*
 * Unicorn 3D Printer Firmware
 * sdcard.h
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
