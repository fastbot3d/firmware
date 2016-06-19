
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>

#include "parameter.h"
#include "eeprom.h"

#include "temp_curve_100K.h"
#include "temp_curve_thermistor_5_104gt2.h" 
#include "temp_curve_thermistor_B3_Pico_500c.h"
#include "temp_curve_thermistor_ATC_Semitec_204GT-2.h"
#include "temp_curve_thermistor_DyzEnd_500C.h"
#include "temp_curve_thermistor_11-QWG-104F-3950_100k.h"
#include "temp_curve_thermistor_default_100k.h"

#define BOARD_INFO_BIN_FILE_NAME "board_info.bin"
#define BOARD_PARAMETER_BIN_FILE_NAME "parameter_info.bin"

#if 0
#define HDR_NO_OF_MAC_ADDR  3
#define HDR_ETH_ALEN        6
#define HDR_NAME_LEN        8
struct am335x_baseboard_id {
    unsigned int  magic;  
    char name[HDR_NAME_LEN];
    char version[4];       
    char serial[12];      
    char config[32];	
    char mac_addr[HDR_NO_OF_MAC_ADDR][HDR_ETH_ALEN]; 
    uint8_t  reserved[88];
} ;
#endif

static struct am335x_baseboard_id g_board_info;
static parameter_t g_param;
static void parameter_default(void);
static void make_bin_file(char *fileName, void *data, int len);
static void make_temp_curve_bin();
static void make_board_info_bin(char *boardName, char *version);
static char file_dir[256] = {0};

int main(int argc, char **argv)
{
	if(argc < 2){
		printf("usage: %s dir_path \n", argv[0]);
		return 0;
	}

	strcpy(file_dir, argv[1]);
	printf("store dir path:%s \n", file_dir);


	make_board_info_bin("BBP1", "A3A");
	make_board_info_bin("BBP1S", "A5A");
	
	make_temp_curve_bin();
#if 0
	printf("create file:%s\n", BOARD_PARAMETER_BIN_FILE_NAME);
	parameter_default();
	make_bin_file(BOARD_PARAMETER_BIN_FILE_NAME , (void *)&g_param, sizeof(g_param));
#endif
	return 0;
}


static void make_bin_file(char *fileName, void *data, int len)
{
	int ret = -1;
	int fd = -1; 
	char tmp_file_dir[256] = {0};
	
	strcpy(tmp_file_dir, file_dir);
	strcat(tmp_file_dir, "/");
	strcat(tmp_file_dir, fileName);
	//fd = open(fileName, O_WRONLY | O_CREAT);
	fd = open(tmp_file_dir, O_WRONLY | O_CREAT);
	if(fd < 0) {
		printf("error, failed to create file:%s\n", tmp_file_dir);
		perror("failed\n");
		return;
	}

	ret = write(fd, data, len);
	if(ret < 0) {
		printf("error, failed to write file:%s\n", tmp_file_dir);
		return;
	}

	if(fd >0)
		close(fd);
}

static void make_temp_curve_100k_bin()
{
}

static void make_temp_curve_bin()
{
	printf("create file:temp_curve\n");
	make_bin_file("temp_curve_100K.bin", curve_100K_data, sizeof(curve_100K_data));
	make_bin_file("temp_curve_thermistor_2_ATC_Semitec_204GT2.bin",curve_thermistor_ATC_Semitec_204GT_2, sizeof(curve_thermistor_ATC_Semitec_204GT_2));
	make_bin_file("temp_curve_thermistor_5_ATC_Semitec_104gt2.bin", curve_thermistor_5_104gt2_data, sizeof(curve_thermistor_5_104gt2_data));
	make_bin_file("temp_curve_thermistor_B3_Pico_500c.bin", curve_thermistor_B3_Pico_500c_data, sizeof(curve_thermistor_B3_Pico_500c_data));
	make_bin_file("temp_curve_thermistor_DyzEnd_500C.bin", curve_thermistor_DyzEnd_500c_data, sizeof(curve_thermistor_DyzEnd_500c_data));
	make_bin_file("temp_curve_thermistor_100K_3950.bin", curve_thermistor_100K_3950_data, sizeof(curve_thermistor_100K_3950_data));
	make_bin_file("thermistor_default_100.bin", thermistor_default_100, sizeof(thermistor_default_100));

	//make_bin_file("temp_curve_200K.bin", curve_200K_data, sizeof(curve_200K_data));
}

static void make_board_info_bin(char *boardName, char *version)
{
	memset(&g_board_info, 0, sizeof(g_board_info));

	g_board_info.magic = 0xEE3355AA;
	char board_name[HDR_NAME_LEN] = {0};
	char file_name[50] = {0};

	strncpy(board_name, boardName,  HDR_NAME_LEN);
	sprintf(file_name, "%s_%s",  board_name, BOARD_INFO_BIN_FILE_NAME);
	printf("create file:%s\n", file_name);

	strncpy(g_board_info.name, board_name,  HDR_NAME_LEN);
	strncpy(g_board_info.version, version,  4);
	strncpy(g_board_info.serial,  "20150611",  12);
	strncpy(g_board_info.config,  "111111111111",  32);
	make_bin_file(file_name, (void *)&g_board_info, sizeof(g_board_info));
}

static void parameter_default(void)
{
	unsigned char i = 0;
	g_param.chk_sum = 0;
	
	char ver[4] = FW_VERSION;
	
	for (i = 0; i < 3; i++) {
		g_param.version[i] = ver[i];
	}
	
	g_param.version[3] = 0;
	
	float f_tmp1[NUM_AXIS] = MAX_FEEDRATE;
	float f_tmp2[NUM_AXIS] = AXIS_STEP_PER_UNIT;
	float f_tmp3[4] = HOMING_FEEDRATE;
	float f_tmp4[3] = HOMING_OFFSET;
	unsigned long ul_tmp1[NUM_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND;
	
	for (i = 0; i < 4; i++) {
		g_param.max_feedrate[i] = f_tmp1[i];
		g_param.axis_steps_per_unit[i] = f_tmp2[i];
		g_param.max_acceleration_units_per_sq_second[i] = ul_tmp1[i];

		if (i < 3) {
			g_param.homing_feedrate[i] = f_tmp3[i];
			g_param.add_homing[i] = f_tmp4[i];
		}
	}
	
	g_param.minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
	g_param.retract_acceleration = RETRACT_ACCELERATION; 
	g_param.max_xy_jerk = MAX_XY_JERK; 
	g_param.max_z_jerk  = MAX_Z_JERK;
	g_param.max_e_jerk  = MAX_E_JERK;
	g_param.mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
	g_param.move_acceleration = ACCELERATION;       
    
    g_param.retract_length   = RETRACT_LENGTH;
    g_param.retract_feedrate = RETRACT_FEEDRATE;
    g_param.retract_zlift    = RETRACT_ZLIFT;
    g_param.retract_recover_length   = RETRACT_RECOVER_LENGTH;
    g_param.retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE; 
    g_param.retract_acceleration     = RETRACT_ACCELERATION;

	g_param.min_software_endstops = MIN_SOFTWARE_ENDSTOPS;
	g_param.max_software_endstops = MAX_SOFTWARE_ENDSTOPS;

	g_param.x_max_length = X_MAX_LENGTH;
	g_param.y_max_length = Y_MAX_LENGTH;
	g_param.z_max_length = Z_MAX_LENGTH;
	
	g_param.disable_x_en = DISABLE_X_EN;
	g_param.disable_y_en = DISABLE_Y_EN;
	g_param.disable_z_en = DISABLE_Z_EN;
	g_param.disable_e_en = DISABLE_E_EN;

	g_param.x_home_dir = X_HOME_DIR;
	g_param.y_home_dir = Y_HOME_DIR;
	g_param.z_home_dir = Z_HOME_DIR;
	
	g_param.x_endstop_invert = X_ENDSTOP_INVERT;
	g_param.y_endstop_invert = Y_ENDSTOP_INVERT;
	g_param.z_endstop_invert = Z_ENDSTOP_INVERT;
	
	g_param.x_min_endstop_aktiv = X_MIN_ACTIV;
	g_param.x_max_endstop_aktiv = X_MAX_ACTIV;
	g_param.y_min_endstop_aktiv = Y_MIN_ACTIV;
	g_param.y_max_endstop_aktiv = Y_MAX_ACTIV;
	g_param.z_min_endstop_aktiv = Z_MIN_ACTIV;
	g_param.z_max_endstop_aktiv = Z_MAX_ACTIV;
	
	g_param.invert_x_dir = INVERT_X_DIR;
	g_param.invert_y_dir = INVERT_Y_DIR;
	g_param.invert_z_dir = INVERT_Z_DIR;
	g_param.invert_e_dir = INVERT_E_DIR;
	
	unsigned int uc_temp1[MAX_AXIS] = AXIS_CURRENT;
	unsigned char uc_temp2[MAX_AXIS] = AXIS_USTEP;
	
	for (i = 0; i < MAX_AXIS ; i++) {
		g_param.axis_current[i] = uc_temp1[i];
		g_param.axis_ustep[i]   = uc_temp2[i];
	}

#ifdef SERVO
    unsigned int angle[2] = SERVO_ENDSTOP_ANGLE;
    for (i = 0; i < 2; i++) {
        g_param.servo_endstop_angle[2] = angle[i];
    }
#endif
}
