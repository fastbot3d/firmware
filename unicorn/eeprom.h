/*
 * Unicorn 3D Printer Firmware
 * eeprom.h
*/
#ifndef _EEPROM_H
#define _EEPROM_H

#include <stdint.h>
#include "common.h"

#define HDR_NO_OF_MAC_ADDR  3
#define HDR_ETH_ALEN        6
#define HDR_NAME_LEN        8
typedef struct am335x_baseboard_id {
    unsigned int  magic;  
    char name[HDR_NAME_LEN];
    char version[4];       
    char serial[12];      
    char config[32];    
    char mac_addr[HDR_NO_OF_MAC_ADDR][HDR_ETH_ALEN]; 
    uint8_t  reserved[88];
} board_info_t;

struct convert_entry {
    unsigned int adc_value;
    float	celsius;
};

typedef enum {
	EXT1_TEMP_CURVE=0, 
	EXT2_TEMP_CURVE=1, 
	BED0_TEMP_CURVE=2, 
	TEMP_CURVE_NUM=3,
	UNKNOW_TEMP_CURVE=-1
} TEMP_CURVE_TYPE;

#define MAX_CURVE_ITEMS  160
struct temp_curve {
	TEMP_CURVE_TYPE type; //not save in eeprom 
	char name[4];
	int array_len;
	struct convert_entry curve[MAX_CURVE_ITEMS];
};
typedef const struct {
    channel_tag tag;
    channel_tag device_path;
} eeprom_config_t;

extern struct temp_curve ext1_temp_curve;
extern struct temp_curve ext2_temp_curve;
extern struct temp_curve bed0_temp_curve;

#if defined (__cplusplus)
extern "C" {
#endif

extern uint32_t eeprom_get_board_info_offset(void);
extern uint32_t eeprom_get_param_offset(void);
extern uint32_t eeprom_get_pru_code_offset(uint32_t pru_nr);

extern int eeprom_write_board_info(const char *device, board_info_t *info);
extern int eeprom_read_board_info(const char *device, board_info_t *info);

extern int eeprom_write_pru_code(const char *device, uint32_t pru_nr, const char *file);
extern int eeprom_read_pru_code(const char *device, uint32_t pru_nr, const char *file);

extern int eeprom_write_block(const char *device, uint8_t *data, uint32_t count, uint32_t offset);
extern int eeprom_read_block(const char *device, uint8_t *data, uint32_t count, uint32_t offset);
extern int curve_config_save_to_eeprom(char *curve_type, const char *device, const char *curve_file_path);
extern int load_temp_curve_from_eeprom(char *eeprom_dev);
extern unsigned int eeprom_get_temp_curve_offset(TEMP_CURVE_TYPE type);

#if defined (__cplusplus)
}
#endif
#endif
