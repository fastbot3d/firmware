/*
 * Unicorn 3D Printer Firmware
 * eeprom.c
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

struct temp_curve_config {
	struct temp_curve ext1_curve;
	struct temp_curve ext2_curve;
	struct temp_curve bed0_curve;
    uint8_t reversed[100];
};

struct eeprom {
    union board_eeprom_config board_config;
    union param_eeprom_config param_config;
    struct pru_code_block pru0_code;
	struct temp_curve_config temp_curve_config;
    struct pru_code_block pru1_code;
};

unsigned int eeprom_get_board_info_offset(void)
{
    return (offsetof(struct eeprom, board_config));
}

static unsigned int eeprom_get_temp_curve_config_offset(void)
{
    return (offsetof(struct eeprom, temp_curve_config));
}

unsigned int eeprom_get_temp_curve_offset(TEMP_CURVE_TYPE type)
{
	int offset = 0, offset_sub = 0;

	switch((int)type){
		case EXT1_TEMP_CURVE:
			offset_sub = offsetof(struct temp_curve_config, ext1_curve);
			break;
		case EXT2_TEMP_CURVE:
			offset_sub = offsetof(struct temp_curve_config, ext2_curve);
			break;
		case BED0_TEMP_CURVE:
			offset_sub = offsetof(struct temp_curve_config, bed0_curve);
			break;
	}
	offset = eeprom_get_temp_curve_config_offset() + offset_sub;
    return offset;
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

    int chunksize = 1024;
	int i = 0;
	int ret = count;
    for (i = 0; i < count; i += chunksize) {
        if (i + chunksize > count) {
            chunksize = (int)count - i;
        }
        
        int len = read(fd, &data[i], chunksize);
        if (len != chunksize) {
            perror("Failed to read EEPROM 1024byte");
            if (len >= 0) {
                fprintf(stderr, "error, Short read (%d) at byte %d.\n", len, i);
            }
            ret = -1;
			break;
        }
    }

    if (fd > 0) {
        close(fd);
    }
    return ret;
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

    if (fd > 0) {
    	close(fd);
	}

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
    if (fd > 0) {
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
        perror("[eeprom]: Failed to open source file for reading");
        ret = -1;
        goto out;
    }

    memset(&data, 0xFF, sizeof(data));
    int count = read(fd, &data, sizeof(data));
    if (count < 0) {
        perror("[eeprom]: Failed to read from file");
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
    if (fd > 0) {
        close(fd);
    }
    return ret;
}

int eeprom_read_pru_code(const char *device, uint32_t pru_nr, const char *file)
{
    uint32_t offset;
    int ret;
    int fd = -1;
    uint32_t count;
    uint32_t data[2048];
    
    if (pru_nr < 0 || pru_nr > 1) {
        return -1;
    }

    offset = eeprom_get_pru_code_offset(pru_nr);
    
    fd = open(file, O_WRONLY | O_CREAT);
    if (fd < 0) {
        perror("[eeprom]: Failed to open file for writing");
        ret = -1;
        goto out;
    }

    memset(&data, 0xFF, sizeof(data));
    count = eeprom_read_block(device, (void *)data, (uint32_t)sizeof(data), offset);
    if (count < 0) {
        ret = -1;
        goto out;
    }

    ret = write(fd, &data, count);
    if (ret < 0) {
        perror("[eeprom]: Failed to write to file %s");
        ret = -1;
        goto out;
    }
    
    ret = 0;
out:
    if (fd > 0) {
        close(fd);
    }
    return ret;
}

static char *temp_curve_name[3] = {"ext1", "ext2", "bed0"};

struct temp_curve ext1_temp_curve = {
		.type = EXT1_TEMP_CURVE,
		.name = "ext1",
		.array_len = 0 ,
};

struct temp_curve ext2_temp_curve = {
		.type = EXT2_TEMP_CURVE,
		.name = "ext2",
		.array_len = 0 ,
};

struct temp_curve bed0_temp_curve = {
		.type = BED0_TEMP_CURVE,
		.name = "bed0",
		.array_len = 0 ,
};

int load_temp_curve_from_eeprom(char *eeprom_dev)
{
	unsigned char data[8];
	unsigned int data_len = 0;
	int offset_ext = 0;
	int i = 0;
	struct temp_curve *curve;

	ext1_temp_curve.array_len = 0;
	ext2_temp_curve.array_len = 0;
	bed0_temp_curve.array_len = 0;

	for(i=0; i< TEMP_CURVE_NUM; i++) {
		offset_ext = eeprom_get_temp_curve_offset(i);
		eeprom_read_block(eeprom_dev, data, 8, offset_ext);
		COMM_DBG("data:%c %c %c %c, %hhu, %hhu, %hhu, %hhu\n", 
					data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		data_len = (data[7] <<24) | (data[6] << 16) | (data[5] << 8) | (data[4] << 0);
		COMM_DBG("data_len:%d\n", data_len);
		if( !strncmp((const char *)data, temp_curve_name[i],4) && (data_len >0) ){
			if( i == EXT1_TEMP_CURVE){
				curve = &ext1_temp_curve;
			} else if( i == EXT2_TEMP_CURVE){
				curve = &ext2_temp_curve;
			} else if( i == BED0_TEMP_CURVE){
				curve = &bed0_temp_curve;
			}

			curve->array_len = data_len/sizeof(struct convert_entry);

			eeprom_read_block(eeprom_dev, (uint8_t *)&(curve->curve), 
										data_len, 
										offset_ext + 8);
			#if 1
			int p_i=0;
			for(p_i=0; p_i<curve->array_len; p_i++){
				COMM_DBG("p_i=%d, adc=%d, temp:%f\n", p_i, curve->curve[p_i].adc_value,  curve->curve[p_i].celsius);
			}
			#endif
		}
	}

	if(ext1_temp_curve.array_len >0) {
		printf("ext1 temp curve is from eeprom\n");	
	}
	if(ext2_temp_curve.array_len >0) {
		printf("ext2 temp curve is from eeprom\n");	
	}
	if(bed0_temp_curve.array_len >0) {
		printf("bed0 temp curve is from eeprom\n");	
	}

	return 0;
}

int curve_config_save_to_eeprom(char *curve_type, const char *device, const char *curve_file_path)
{

    int offset;
    int fd = -1;
	int ret = -1;
    unsigned char data[2048];

	TEMP_CURVE_TYPE type = UNKNOW_TEMP_CURVE;

	if(!strncmp(curve_type, "extruder1", 9)){
		type = EXT1_TEMP_CURVE; 
	} else if(!strncmp(curve_type, "extruder2", 9)){
		type = EXT2_TEMP_CURVE; 
	} else if(!strncmp(curve_type, "bed", 3)){
		type = BED0_TEMP_CURVE; 
	} else {
        printf("[eeprom]: Failed,  unkonwn curve_type:%s", curve_type);
		ret = -1;
		return ret;
	}

	offset = eeprom_get_temp_curve_offset(type);

	fd = open(curve_file_path, O_RDONLY);
    if (fd < 0) {
        perror("[eeprom]: Failed to open source file for reading");
        ret = -1;
        goto out;
    }

    memset(&data, 0xFF, sizeof(data));
    int count = read(fd, ((char *)data + 8), sizeof(data));
    if (count < 0) {
        perror("[eeprom]: Failed to read from file");
        ret = -1;
        goto out;
    }

	strncpy((void *)data, temp_curve_name[type], 4);
	*((int*)((char *)data + 4)) = count;
    
    ret = eeprom_write_block(device, (void *)data, (uint32_t)count + 8, offset);
    if (ret < 0) {
        ret = -1;
        goto out;
    }
    ret = 0;
out:
    if (fd > 0) {
        close(fd);
    }
    return ret;
}

