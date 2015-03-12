/*
 * Unicorn 3D Printer Firmware
 * eeprom.h
 * EEPROM Read & Write interface
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
#ifndef _EEPROM_H
#define _EEPROM_H

#include <stdint.h>

typedef struct {
    uint32_t header;
    uint8_t  name[8];
    uint8_t  version[4];
    uint8_t  serial[12];
    uint8_t  opt[12];
    uint8_t  reserved[88];
} board_info_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern uint32_t eeprom_get_board_info_offset(void);
extern uint32_t eeprom_get_param_offset(void);
extern uint32_t eeprom_get_pru_code_offset(uint32_t pru_nr);

extern int eeprom_write_block(const char *device, uint8_t *data, uint32_t count, uint32_t offset);
extern int eeprom_read_block(const char *device, uint8_t *data, uint32_t count, uint32_t offset);

extern int eeprom_write_board_info(const char *device, board_info_t *info);
extern int eeprom_read_board_info(const char *device, board_info_t *info);

extern int eeprom_write_pru_code(const char *device, uint32_t pru_nr, const char *file);
extern int eeprom_read_pru_code(const char *device, uint32_t pru_nr, const char *file);

#if defined (__cplusplus)
}
#endif
#endif
