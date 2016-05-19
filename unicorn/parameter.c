/*
 * Unicorn 3D Printer Firmware
 * parameter.c
*/
#include <stdio.h>
#include <stdlib.h>

#include "eeprom.h"
#include "parameter.h"

parameter_t pa;

/*
 * Print parameter
 */
void parameter_dump(void)
{
	printf("Version: %s \r\n",&pa.version[0]);

	printf("Maximum feedrates (mm/s):\r\nM202 X%d Y%d Z%d E%d \r\n",
            (int)pa.max_feedrate[0],
            (int)pa.max_feedrate[1],
            (int)pa.max_feedrate[2],
            (int)pa.max_feedrate[3]);

	printf("Homing Feedrate mm/min \r\nM207 X%d Y%d Z%d\r\n",
            (int)pa.homing_feedrate[0], 
            (int)pa.homing_feedrate[1], 
            (int)pa.homing_feedrate[2]);

    printf("Home offset\r\nM206 X%f Y%f Z%f\r\n", 
            pa.add_homing[0], 
            pa.add_homing[1], 
            pa.add_homing[2]);

	printf("Steps per unit:\r\nM92 X%f Y%f Z%f E%f\r\n", 
            pa.axis_steps_per_unit[0], 
            pa.axis_steps_per_unit[1], 
            pa.axis_steps_per_unit[2], 
            pa.axis_steps_per_unit[3]);

	printf("Maximum Acceleration (mm/s2):\r\nM201 X%d Y%d Z%d E%d\r\n",
            (int)pa.max_acceleration_units_per_sq_second[0],
            (int)pa.max_acceleration_units_per_sq_second[1],
            (int)pa.max_acceleration_units_per_sq_second[2],
            (int)pa.max_acceleration_units_per_sq_second[3]);

	printf("Acceleration: S=acceleration, T=retract acceleration\r\nM204 S%d T%d\r\n",
            (int)pa.move_acceleration,
            (int)pa.retract_acceleration);

	printf("Advanced variables (mm/s): S=Min feedrate, T=Min travel feedrate, X=max xy jerk,  Z=max Z jerk,");
	printf(" E=max E jerk\r\nM205 S%d T%d X%d Z%d E%d\r\n",
            (int)pa.minimumfeedrate,
            (int)pa.mintravelfeedrate,
            (int)pa.max_xy_jerk,
            (int)pa.max_z_jerk,
            (int)pa.max_e_jerk);

	printf("Maximum Area unit:\r\nM520 X%d Y%d Z%d\r\n",
            (int)pa.x_max_length,
            (int)pa.y_max_length,
            (int)pa.z_max_length);

	printf("Disable axis when unused:\r\nM521 X%d Y%d Z%d E%d\r\n",
            pa.disable_x_en,
            pa.disable_y_en,
            pa.disable_z_en,
            pa.disable_e_en);

	printf("Use Software Endstop I=Min, A=Max:\r\nM522 I%d A%d\r\n",
            pa.min_software_endstops,
            pa.max_software_endstops);
	
	printf("Minimum Endstop Input:\r\nM523 X%d Y%d Z%d\r\n",
            pa.x_min_endstop_aktiv,
            pa.y_min_endstop_aktiv,
            pa.z_min_endstop_aktiv);

	printf("Maximum Endstop Input:\r\nM524 X%d Y%d Z%d\r\n",
            pa.x_max_endstop_aktiv,
            pa.y_max_endstop_aktiv,
            pa.z_max_endstop_aktiv);
	
	printf("Homing Direction (-1=minimum,1=maximum):\r\nM525 X%d Y%d Z%d\r\n",
            pa.x_home_dir,
            pa.y_home_dir,
            pa.z_home_dir);

	printf("Endstop invert:\r\nM526 X%d Y%d Z%d\r\n",
            pa.x_endstop_invert,
            pa.y_endstop_invert,
            pa.z_endstop_invert);

	printf("Axis invert:\r\nM510 X%d Y%d Z%d E%d\r\n",
            pa.invert_x_dir,
            pa.invert_y_dir,
            pa.invert_z_dir,
            pa.invert_e_dir);
	
	printf("Motor Current \r\n  M907 X%d Y%d Z%d E%d B%d \r\n",
            pa.axis_current[0],
            pa.axis_current[1],
            pa.axis_current[2],
            pa.axis_current[3],
            pa.axis_current[4]);

	printf("Motor Current (mA) (range 0-1900):\r\nM906 X%d Y%d Z%d E%d B%d \r\n",
            pa.axis_current[0],
            pa.axis_current[1],
            pa.axis_current[2],
            pa.axis_current[3],
            pa.axis_current[4]);

	printf("Motor Microstepping (1,2,4,8,16): \r\nM350 X%d Y%d Z%d E%d B%d \r\n",
            pa.axis_ustep[0],
            pa.axis_ustep[1],
            pa.axis_ustep[2],
            pa.axis_ustep[3],
            pa.axis_ustep[4]);
}

int parameter_init(void)
{
	unsigned char i = 0;
		
	pa.chk_sum = 0;
	
	char ver[4] = FLASH_VERSION;
	
	for (i = 0; i < 3; i++) {
		pa.version[i] = ver[i];
	}
	
	pa.version[3] = 0;
	
	float f_tmp1[NUM_AXIS] = MAX_FEEDRATE;
	float f_tmp2[NUM_AXIS] = AXIS_STEP_PER_UNIT;
	float f_tmp3[3] = HOMING_FEEDRATE;
	float f_tmp4[3] = HOMING_OFFSET;
	unsigned long ul_tmp1[NUM_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND;
	
	for (i = 0; i < 4; i++) {
		pa.max_feedrate[i] = f_tmp1[i];
		pa.axis_steps_per_unit[i] = f_tmp2[i];
		pa.max_acceleration_units_per_sq_second[i] = ul_tmp1[i];

		if (i < 3) {
			pa.homing_feedrate[i] = f_tmp3[i];
			pa.add_homing[i] = f_tmp4[i];
		}
	}
	
	pa.minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
	pa.retract_acceleration = RETRACT_ACCELERATION; 
	pa.max_xy_jerk = MAX_XY_JERK; 
	pa.max_z_jerk  = MAX_Z_JERK;
	pa.max_e_jerk  = MAX_E_JERK;
	pa.mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
	pa.move_acceleration = ACCELERATION;       
	
	pa.min_software_endstops = MIN_SOFTWARE_ENDSTOPS;
	pa.max_software_endstops = MAX_SOFTWARE_ENDSTOPS;

	pa.x_max_length = X_MAX_LENGTH;
	pa.y_max_length = Y_MAX_LENGTH;
	pa.z_max_length = Z_MAX_LENGTH;
	
	pa.disable_x_en = DISABLE_X_EN;
	pa.disable_y_en = DISABLE_Y_EN;
	pa.disable_z_en = DISABLE_Z_EN;
	pa.disable_e_en = DISABLE_E_EN;

	pa.x_home_dir = X_HOME_DIR;
	pa.y_home_dir = Y_HOME_DIR;
	pa.z_home_dir = Z_HOME_DIR;
	
	pa.x_endstop_invert = X_ENDSTOP_INVERT;
	pa.y_endstop_invert = Y_ENDSTOP_INVERT;
	pa.z_endstop_invert = Z_ENDSTOP_INVERT;
	
	pa.x_min_endstop_aktiv = X_MIN_ACTIV;
	pa.x_max_endstop_aktiv = X_MAX_ACTIV;
	pa.y_min_endstop_aktiv = Y_MIN_ACTIV;
	pa.y_max_endstop_aktiv = Y_MAX_ACTIV;
	pa.z_min_endstop_aktiv = Z_MIN_ACTIV;
	pa.z_max_endstop_aktiv = Z_MAX_ACTIV;
	
	pa.invert_x_dir = INVERT_X_DIR;
	pa.invert_y_dir = INVERT_Y_DIR;
	pa.invert_z_dir = INVERT_Z_DIR;
	pa.invert_e_dir = INVERT_E_DIR;
	
	unsigned char uc_temp1[5] = AXIS_CURRENT;
	unsigned char uc_temp2[5] = AXIS_USTEP;
	
	for (i = 0; i < 5; i++) {
		pa.axis_current[i] = uc_temp1[i];
		pa.axis_ustep[i]   = uc_temp2[i];
	}
	
    //parameter_dump();

    return 0;
}

void parameter_exit(void)
{


}

int eeprom_write_param(const char *device, parameter_t *param)
{
    return eeprom_write_block(device, 
                              (uint8_t *)param, 
                              sizeof(parameter_t), 
                              eeprom_get_param_offset());
}

int eeprom_read_param(const char *device, parameter_t *param)
{
    return eeprom_read_block(device, 
                             (uint8_t *)param, 
                             sizeof(parameter_t), 
                             eeprom_get_param_offset());
}
/*
 * Load parameter from eeprom
 */
void parameter_load_from_eeprom(void)
{

}
/* 
 * Save parameter to eeprom
 */
void parameter_save_to_eeprom(void)
{

}
/*
 * Save parameter to sd card
 */
void parameter_save_to_sd(void)
{

}
