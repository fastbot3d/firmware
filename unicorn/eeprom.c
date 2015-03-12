/*
 * Unicorn 3D Printer Firmware
 * eeprom.c
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>

#include "parameter.h"
#include "eeprom.h"

union board_eeprom_config {
    board_info_t info;
    uint8_t data[512];
};

union param_eeprom_config {
    parameter_t param;
    uint8_t data[512];
};

#define BOARD_CONFIG_LEN  (sizeof(struct board_eeprom_config))
#define PARAM_CONFIG_LEN  (sizeof(parameter_t))
#define RESERVED_LEN (1024 - BOARD_CONFIG_LEN - PARAM_CONFIG_LEN)

struct pru_code_block {
    uint32_t opcodes[2048];
};

struct eeprom {
    union board_eeprom_config board_config;
    union param_eeprom_config param_config;
    struct pru_code_block pru0_code;
    struct pru_code_block pru1_code;
};

unsigned int eeprom_get_board_info_offset(void)
{
    return (offsetof(struct eeprom, board_config));
}

unsigned int eeprom_get_param_offset(void)
{
    return (offsetof(struct eeprom, param_config));
}

unsigned int eeprom_get_pru_code_offset(unsigned int pru_nr)
{
    unsigned int offset = 0;

    switch (pru_nr) 
    {
    case 0: 
        offset = offsetof(struct eeprom, pru0_code);
        break;
    case 1:
        offset = offsetof(struct eeprom, pru1_code);
        break;
    default:
        fprintf(stderr, "eeprom_get_pru_code_offset: Wrong pru_nr %d\n", 
                pru_nr);
        break;
    }

    return offset;
}

int eeprom_read_block(const char *device, uint8_t *data, uint32_t count, uint32_t offset)
{
    if (!data) {
        return -1;
    }

    int fd = open(device, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open EEPROM for reading");
        return -1;
    }
    
    if (lseek(fd, offset, SEEK_SET) < 0) {
        perror("Failed to seek EEPROM");
        return -1;
    }

    int len = read(fd, data, count);
    if (len != count) {
        perror("Failed to read from EEPROM");
        fprintf(stderr, "Failed to read %d bytes at offset %d from EEPROM\n",
                count, offset);
        return -1;
    }

    return 0;
}

int eeprom_write_block(const char *device, uint8_t *data, uint32_t count, uint32_t offset)
{
    int i;
    int ret; 
    int fd = -1;

    fd = open(device, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open EEPROM for writing");
        ret = -1;
        goto out;
    }
    
    ret = lseek(fd, offset, SEEK_SET);
    if (ret < 0 || ret != offset) {
        perror("Failed to lseek EEPROM");
        ret = -1;
        goto out;
    }
    
    int chunksize = 16;
    for (i = 0; i < count; i += chunksize) {
        if (i + chunksize > count) {
            chunksize = (int)count - i;
        }
        
        int len = write(fd, &data[i], chunksize);
        if (len != chunksize) {
            perror("Failed to write to EEPROM");
            if (len >= 0) {
                fprintf(stderr, "Short write (%d) at byte %d.\n", len, i);
            }
            ret = -1;
            goto out;
        }
    }
    close(fd);

    /* Verify EEPROm contents against data */
    fd = open(device, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open EEPROm for reading");
        ret = -1;
        goto out;
    }
    
    ret = lseek(fd, offset, SEEK_SET);
    for (i = 0; i < count; ++i) {
        uint8_t byte;
        int len = read(fd, &byte, sizeof(byte));
        if (len != sizeof(byte)) {
            perror("Failed to read from EEPROM");
            if (len >= 0) {
                fprintf(stderr, "Short read (%d) at byte %d.\n", len, i);
            }
            ret = -1;
            goto out;
        }
        
        if (byte != data[i]) {
            fprintf(stderr, "EEPROM verification failed at opcode[%d]: is %02x, should be %02x.\n",
                    i, byte, data[i]);
        }
    }
    ret = 0;
out:
    if (fd >= 0) {
        close(fd);
    }
    return ret;
}

int eeprom_write_board_info(const char *device, board_info_t *info)
{
    return eeprom_write_block(device, 
                              (uint8_t *)info, 
                              sizeof(board_info_t), 
                              eeprom_get_board_info_offset());
}

int eeprom_read_board_info(const char *device, board_info_t *info)
{
    return eeprom_read_block(device, 
                             (uint8_t *)info, 
                             sizeof(board_info_t), 
                             eeprom_get_board_info_offset());
}

int eeprom_write_pru_code(const char *device, uint32_t pru_nr, const char *file)
{
    uint32_t offset;
    int ret;
    int fd = -1;
    uint32_t data[2048];
    
    if (pru_nr < 0 || pru_nr > 1) {
        return -1;
    }

    offset = eeprom_get_pru_code_offset(pru_nr);
    
    fd = open(file, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open source for reading");
        ret = -1;
        goto out;
    }

    memset(&data, 0xFF, sizeof(data));
    int count = read(fd, &data, sizeof(data));
    if (count < 0) {
        perror("Failed to read from file");
        ret = -1;
        goto out;
    }
    
    ret = eeprom_write_block(device, (void *)data, (uint32_t)count, offset);
    if (ret < 0) {
        ret = -1;
        goto out;
    }
    ret = 0;
out:
    if (fd >= 0) {
        close(fd);
    }
    return ret;
}

int eeprom_read_pru_code(const char *device, uint32_t pru_nr, const char *file)
{
    return 0;
}

