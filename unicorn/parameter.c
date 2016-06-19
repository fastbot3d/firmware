/*
 * Unicorn 3D Printer Firmware
 * parameter.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "eeprom.h"
#include "parameter.h"
#include "unicorn.h"
#include "stepper_pruss.h"

char *eeprom_dev = NULL;
parameter_t pa;

void parameter_dump(int (*gcode_send_response_remote)(char *));

static unsigned long calculate_eeprom_param_crc(parameter_t *param)
{
	void *data = (void *)(&(param->chk_sum) + sizeof(unsigned long));
	int size = sizeof(parameter_t) - sizeof(param->chk_sum);

	return data_crc(data, size);
}

static void set_eeprom_param_crc(parameter_t *param)
{
	 param->chk_sum = calculate_eeprom_param_crc(param);
}

static unsigned long get_eeprom_param_crc(parameter_t *param)
{
	return calculate_eeprom_param_crc(param);
}

static int eeprom_write_param(parameter_t *param)
{
    if (!eeprom_dev) {
        return -1;
    } else {
        return eeprom_write_block(eeprom_dev, 
                                  (uint8_t *)param, 
                                  sizeof(parameter_t), 
                                  eeprom_get_param_offset());
    }
}

static int eeprom_read_param(parameter_t *param)
{
    if (!eeprom_dev) {
        return -1;
    } else {
        return eeprom_read_block(eeprom_dev, 
                                 (uint8_t *)param, 
                                 sizeof(parameter_t), 
                                 eeprom_get_param_offset());
    }
}
/*
 * Load parameter from eeprom
 */
int parameter_load_from_eeprom(void)
{
    return eeprom_read_param(&pa); 
}
/* 
 * Save parameter to eeprom
 */
int parameter_save_to_eeprom(void)
{
	set_eeprom_param_crc(&pa);
    return eeprom_write_param(&pa);
}
/*
 * Save parameter to sd card
 */
int parameter_save_to_sd(void)
{
    return 0;
}
/*
 * Restore parameter to default value
 */
void parameter_restore_default(void)
{
	unsigned char i = 0;
	pa.chk_sum = 0;
	
	char ver[4] = FW_VERSION;
	
	for (i = 0; i < 3; i++) {
		pa.version[i] = ver[i];
	}
	
	pa.version[3] = 0;
	
	float f_tmp1[NUM_AXIS] = MAX_FEEDRATE;
	float f_tmp2[NUM_AXIS] = AXIS_STEP_PER_UNIT;
	float f_tmp3[4] = HOMING_FEEDRATE;
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
	
	pa.max_xy_jerk = MAX_XY_JERK; 
	pa.max_z_jerk  = MAX_Z_JERK;
	pa.max_e_jerk  = MAX_E_JERK;
	pa.minimumfeedrate   = DEFAULT_MINIMUMFEEDRATE;
	pa.mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
    pa.minsegmenttime    = DEFAULT_MINSEGMENTTIME;
	pa.retract_acceleration = RETRACT_ACCELERATION; 
	pa.move_acceleration = ACCELERATION;       
    
    pa.retract_length   = RETRACT_LENGTH;
    pa.retract_feedrate = RETRACT_FEEDRATE;
    pa.retract_zlift    = RETRACT_ZLIFT;
    pa.retract_recover_length   = RETRACT_RECOVER_LENGTH;
    pa.retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE; 
    pa.retract_acceleration     = RETRACT_ACCELERATION;

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
	pa.autolevel_endstop_invert= 0;

    pa.bbp1s_dual_xy_mode = MOTOR56_MODE_EXTRUDER;
	
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
	
	unsigned int uc_temp1[MAX_AXIS] = AXIS_CURRENT;
	unsigned char uc_temp2[MAX_AXIS] = AXIS_USTEP;
	
	for (i = 0; i < MAX_AXIS; i++) {
		pa.axis_current[i] = uc_temp1[i];
		pa.axis_ustep[i]   = uc_temp2[i];
	}

	pa.ext_count = 1;
	pa.has_bed = 1;

	memset(&(pa.ext_offset), 0, sizeof(pa.ext_offset));

    pa.auto_current = AUTO_CURRENT_ADJUST;
    pa.slow_down = SLOWDOWN;

    pa.machine_type = MACHINE_TYPE;
    

    pa.radius = DELTA_RADIUS;
    pa.delta_print_radius = DELTA_RADIUS/2;
    pa.diagonal_rod = DELTA_DIAGONAL_ROD;
    pa.z_home_pos = MANUAL_Z_HOME_POS;
    pa.segments_per_second = DELTA_SEGMENTS_PER_SECOND;

    pa.endstop_adjust[0] = 0.0;
    pa.endstop_adjust[1] = 0.0;
    pa.endstop_adjust[2] = 0.0;

    unsigned int angle[2] = SERVO_ENDSTOP_ANGLE;
#ifdef SERVO 
    for (i = 0; i < 2; i++) {
        pa.servo_endstop_angle[i] = angle[i];
    }
#endif
	pa.slowdown_percent = 100;
    pa.bbp1_extend_func = BBP1_EXTEND_FUNC_DUAL_Z;
    pa.max_heat_pwm_hotend = 80;
    pa.max_heat_pwm_bed = 40;


	pa.autoLeveling = 0;
	pa.probeDeviceType = PROBE_DEVICE_PROXIMIRY; 
	pa.endstopOffset[X_AXIS] = X_PROBE_OFFSET_FROM_EXTRUDER;
	pa.endstopOffset[Y_AXIS] = Y_PROBE_OFFSET_FROM_EXTRUDER;
	pa.endstopOffset[Z_AXIS] = Z_PROBE_OFFSET_FROM_EXTRUDER;
	pa.probePoint1[X_AXIS] = ABL_PROBE_PT_1_X;
	pa.probePoint1[Y_AXIS] = ABL_PROBE_PT_1_Y;
	pa.probePoint2[X_AXIS] = ABL_PROBE_PT_2_X;
	pa.probePoint2[Y_AXIS] = ABL_PROBE_PT_2_Y;
	pa.probePoint3[X_AXIS] = ABL_PROBE_PT_3_X;
	pa.probePoint3[Y_AXIS] = ABL_PROBE_PT_3_Y;

	pa.probeLeftPos = LEFT_PROBE_BED_POSITION;
	pa.probeRightPos = RIGHT_PROBE_BED_POSITION;
	pa.probeBackPos = BACK_PROBE_BED_POSITION;
	pa.probeFrontPos = FRONT_PROBE_BED_POSITION;
	pa.probeGridPoints = AUTO_LEVELING_GRID_POINTS;
    pa.zRaiseBeforeProbing = Z_RAISE_BEFORE_PROBING;
    pa.zRaiseBetweenProbing = Z_RAISE_BETWEEN_PROBING;

	pa.dangerousThermistor = MAX_DANGEROUS_CELSIUS;
	pa.dangerousThermocouple = MAX_THERMOCOUPLE_DANGEROUS_CELSIUS;

//#define Z_PROBE_DEPLOY_START_LOCATION {20, 96, 30, 0}   // X, Y, Z, E start location for z-probe deployment sequence
//#define Z_PROBE_DEPLOY_END_LOCATION {5, 96, 30, 0}    // X, Y, Z, E end location for z-probe deployment sequence
//#define Z_PROBE_RETRACT_START_LOCATION {49, 84, 20, 0}  // X, Y, Z, E start location for z-probe retract sequence
//#define Z_PROBE_RETRACT_END_LOCATION {49, 84, 1, 0}     // X, Y, Z, E end location for z-probe retract sequence 
    pa.delta_deploy_start_location[0] = 20;
    pa.delta_deploy_start_location[1] = 96;
    pa.delta_deploy_start_location[2] = 30;
    pa.delta_deploy_end_location[0] = 5;
    pa.delta_deploy_end_location[1] = 96;
    pa.delta_deploy_end_location[2] = 30;

    pa.delta_retract_start_location[0] = 49;
    pa.delta_retract_start_location[1] = 84;
    pa.delta_retract_start_location[2] = 20;
	pa.delta_retract_end_location[0] = 49;
	pa.delta_retract_end_location[2] = 84;
	pa.delta_retract_end_location[2] = 1;

	pa.radius_adj[0] = 0;
	pa.radius_adj[1] = 0;
	pa.radius_adj[2] = 0;
	pa.diagonal_rod_adj[0] = 0;
	pa.diagonal_rod_adj[1] = 0;
	pa.diagonal_rod_adj[2] = 0;
	pa.endstop_adj[0] = 0;
	pa.endstop_adj[1] = 0;
	pa.endstop_adj[2] = 0;

	pa.autolevel_down_rate = 800;
}
/*
 * parameter module init
 */
int parameter_init(char *dev)
{
    int ret = 0;

    if (!dev) {
        return -1;
    } else {
        eeprom_dev = dev;
    }
    
    memset(&pa, 0, sizeof(parameter_t));

    /* Read parameter from eeprom
     * Check eeprom parameter available, if not, use default parameters,
     * else load parameter from eeprom
     */
#if 0
    parameter_restore_default();
#else
	unsigned long crc = -1;
	ret = parameter_load_from_eeprom();
	if (ret < 0) {	
		printf("read param from eeprom err!\n");
    	parameter_restore_default();
	} else {
        crc = get_eeprom_param_crc(&pa);
		if((crc == pa.chk_sum) && (crc != 0)){
            printf("param from eeprom is ok\n");
        } else {
            printf("param use default \n");
            parameter_restore_default();
        }
	}

	pa.slowdown_percent = 100;

	pa.axis_current[E2_AXIS] = pa.axis_current[E_AXIS];
	pa.axis_current[E3_AXIS] = pa.axis_current[E_AXIS];
	pa.axis_current[U_AXIS] =  pa.axis_current[Z_AXIS];

	pa.axis_ustep[E2_AXIS]   = pa.axis_ustep[E_AXIS];
	pa.axis_ustep[E3_AXIS]   = pa.axis_ustep[E_AXIS];
	pa.axis_ustep[U_AXIS]   = pa.axis_ustep[Z_AXIS];
#endif

    parameter_dump(NULL);

	load_temp_curve_from_eeprom(EEPROM_DEV);

    return ret;
}
/*
 * parameter module exit
 */
void parameter_exit(void)
{


}
/*
 * Print parameter for debug
 */
void parameter_dump(int (*gcode_send_response_remote)(char *))
{
	int i=0;
	char send_buf[3024] = {0};
	char machine_type[20] = {0};


	printf("Firmware Version: %s \r\n",&pa.version[0]);
    
    printf("---Calibration setting:\n");
	printf("Steps per unit:\r\nM92 X%f Y%f Z%f E%f\r\n", 
            pa.axis_steps_per_unit[0], 
            pa.axis_steps_per_unit[1], 
            pa.axis_steps_per_unit[2], 
            pa.axis_steps_per_unit[3]);

	printf("Maximum Area unit:\r\nM520 X%d Y%d Z%d\r\n",
            (int)pa.x_max_length,
            (int)pa.y_max_length,
            (int)pa.z_max_length);

	printf("Homing Direction (-1=minimum,1=maximum):\r\nM525 X%d Y%d Z%d\r\n",
            pa.x_home_dir,
            pa.y_home_dir,
            pa.z_home_dir);

    printf("Home offset\r\nM206 X%f Y%f Z%f\r\n", 
            pa.add_homing[0], 
            pa.add_homing[1], 
            pa.add_homing[2]);

	printf("Axis invert:\r\nM510 X%d Y%d Z%d E%d\r\n",
            pa.invert_x_dir,
            pa.invert_y_dir,
            pa.invert_z_dir,
            pa.invert_e_dir);

	printf("Disable axis when unused:\r\nM521 X%d Y%d Z%d E%d\r\n",
            pa.disable_x_en,
            pa.disable_y_en,
            pa.disable_z_en,
            pa.disable_e_en);

	printf("Motor Current (mA) (range 0-1900):\r\nM906 X%d Y%d Z%d E%d B%d \r\n",
            pa.axis_current[0],
            pa.axis_current[1],
            pa.axis_current[2],
            pa.axis_current[3],
            pa.axis_current[4]);

	printf("Motor Microstepping (1,2,4,8,16,32): \r\nM909 X%d Y%d Z%d E%d B%d \r\n",
            pa.axis_ustep[0],
            pa.axis_ustep[1],
            pa.axis_ustep[2],
            pa.axis_ustep[3],
            pa.axis_ustep[4]);

    printf("---Endstop setting:\n");
	printf("Endstop invert:\r\nM526 X%d Y%d Z%d A%d \r\n",
            pa.x_endstop_invert,
            pa.y_endstop_invert,
            pa.z_endstop_invert,
            pa.autolevel_endstop_invert);

	printf("Minimum Endstop Input:\r\nM523 X%d Y%d Z%d\r\n",
            pa.x_min_endstop_aktiv,
            pa.y_min_endstop_aktiv,
            pa.z_min_endstop_aktiv);

	printf("Maximum Endstop Input:\r\nM524 X%d Y%d Z%d\r\n",
            pa.x_max_endstop_aktiv,
            pa.y_max_endstop_aktiv,
            pa.z_max_endstop_aktiv);
	
	printf("Use Software Endstop I=Min, A=Max:\r\nM522 I%d A%d\r\n",
            pa.min_software_endstops,
            pa.max_software_endstops);

    printf("---Retract setting:\n");
	printf("Retract length \r\nM207 S%d F%d Z%d\r\n",
            (int)pa.retract_length, 
            (int)pa.retract_feedrate, 
            (int)pa.retract_zlift);
	printf("bbp1_extend_func :%d\n", pa.bbp1_extend_func);

	printf("Retract recover length \r\nM208 S%d F%d\r\n",
            (int)pa.retract_recover_length, 
            (int)pa.retract_recover_feedrate);

    printf("---movement setting:\n");
	printf("Maximum feedrates (mm/s):\r\nM203 X%d Y%d Z%d E%d \r\n",
            (int)pa.max_feedrate[0],
            (int)pa.max_feedrate[1],
            (int)pa.max_feedrate[2],
            (int)pa.max_feedrate[3]);

	printf("Homing feedrate \r\nM210 X%d Y%d Z%d E%d D%d \r\n",
            (int)pa.homing_feedrate[0], 
            (int)pa.homing_feedrate[1], 
            (int)pa.homing_feedrate[2], 
            (int)pa.homing_feedrate[3], 
			(int)pa.autolevel_down_rate);

    printf("---Acceleration setting:\n");
	printf("Maximum Acceleration (mm/s2):\r\nM201 X%d Y%d Z%d E%d\r\n",
            (int)pa.max_acceleration_units_per_sq_second[0],
            (int)pa.max_acceleration_units_per_sq_second[1],
            (int)pa.max_acceleration_units_per_sq_second[2],
            (int)pa.max_acceleration_units_per_sq_second[3]);

	printf("Acceleration: S=acceleration, T=retract acceleration\r\nM204 S%d T%d\r\n",
            (int)pa.move_acceleration,
            (int)pa.retract_acceleration);

	printf("Advanced variables (mm/s): S=Min feedrate, T=Min travel feedrate, X=max xy jerk,  Z=max Z jerk,");
	printf(" E=max E jerk\r\nM205 S%d F%d X%d Z%d E%d\r\n",
            (int)pa.minimumfeedrate,
            (int)pa.mintravelfeedrate,
            (int)pa.max_xy_jerk,
            (int)pa.max_z_jerk,
            (int)pa.max_e_jerk);

    printf("---Servo Endstop angle:\n");
	printf("Servo endstop angle \r\nM914 S%d E%d\r\n",
            (int)pa.servo_endstop_angle[0], 
            (int)pa.servo_endstop_angle[1]);

	printf("----Extruder offset:%d\n", pa.ext_count);
	for (i = 0; i < pa.ext_count; i++) {
		printf("ext[%d] offset x:%f, y:%f, z:%f\n", i, 
                           pa.ext_offset[X_AXIS][i], 
						   pa.ext_offset[Y_AXIS][i], 
                           pa.ext_offset[Z_AXIS][i]);
	}

    printf("---HBP: S=1, 0\n");
	printf("HBP Available \r\nM908 S%d\r\n", pa.has_bed);
	printf("Dangerous Thermistor temperature: %d\n", pa.dangerousThermistor);
	printf("Dangerous Thermocouple temperature: %d\n", pa.dangerousThermocouple);

    printf("---Auto current adjust: S=1, 0\n");
	printf("Enable auto current adjust: \r\nM911 S%d\r\n", pa.auto_current);

    printf("---Slow down: S=1, 0\n");
	printf("Auto slow down enable: \r\nM912 S%d\r\n", pa.slow_down);
	printf("slow down percent: \r\nM915 S%d\r\n", pa.slowdown_percent);

    printf("---Machine type: S=0, 1, 2\n");
	printf("Machine type setting \r\nM913 S%d\r\n", pa.machine_type);
    switch (pa.machine_type) {
        case MACHINE_XYZ:
            printf("             xyz\n");
			strcpy(machine_type, "XYZ");
            break;
        case MACHINE_DELTA:
            printf("             delta\n");
			strcpy(machine_type, "DELTA");
            break;
        case MACHINE_COREXY:
            printf("             corexy\n");
			strcpy(machine_type, "COREXY");
            break; 
        default:
            break;
    }
    printf("---Delta Machine diagonal_rod=%f,radius=%f,available_radius=%f,z_home_pos=%f,segments_per_second=%f\n", 
                              pa.diagonal_rod, pa.radius, pa.delta_print_radius, pa.z_home_pos, pa.segments_per_second);

	printf("---Delta deploy start location:(%f, %f, %f), end location:(%f, %f, %f)\n", 
			pa.delta_deploy_start_location[0],	pa.delta_deploy_start_location[1], pa.delta_deploy_start_location[2],
			pa.delta_deploy_end_location[0], 	pa.delta_deploy_end_location[1], pa.delta_deploy_end_location[2] );  
	printf("---Delta retract start location:(%f, %f, %f), end location:(%f, %f, %f)\n", 
    				pa.delta_retract_start_location[0], pa.delta_retract_start_location[1], pa.delta_retract_start_location[2],
					pa.delta_retract_end_location[0],   pa.delta_retract_end_location[1],	pa.delta_retract_end_location[2] );

    printf("max_heat_pwm_hotend:%d, max_heat_pwm_bed=%d\n", 
                        pa.max_heat_pwm_hotend, pa.max_heat_pwm_bed);
    printf("Auto Level enable:%d, device type:%d,  endstop extend:%d, retract:%d, endsoffset:%f,%f,%f, pt1:%f,%f pt2:%f,%f, pt3:%f,%f\n", 
					pa.autoLeveling, pa.probeDeviceType, pa.servo_endstop_angle[0], pa.servo_endstop_angle[1],
					pa.endstopOffset[X_AXIS], pa.endstopOffset[Y_AXIS],	pa.endstopOffset[Z_AXIS],
					pa.probePoint1[X_AXIS],	pa.probePoint1[Y_AXIS], 
					pa.probePoint2[X_AXIS], pa.probePoint2[Y_AXIS],
					pa.probePoint3[X_AXIS], pa.probePoint3[Y_AXIS]);
    printf("Auto Level  Z_RAISE_BEFORE_PROBING:%f, Z_RAISE_BETWEEN_PROBING:%f,Grid left:%f, right:%f,  front:%f, back:%f, point:%d \n",
			        pa.zRaiseBeforeProbing, pa.zRaiseBetweenProbing,
                    pa.probeLeftPos, pa.probeRightPos, pa.probeFrontPos, pa.probeBackPos, pa.probeGridPoints);
    printf("thermocouple_max6675_cnnection:%d, thermocouple_ad597_cnnection:%d\n", pa.thermocouple_max6675_cnnection, pa.thermocouple_ad597_cnnection);


	if (gcode_send_response_remote == NULL) {
			printf("dump only in local\n");
			return;
	}
	memset(send_buf, 0, sizeof(send_buf));
	sprintf(send_buf, "Machine type:%s \n"
				  "bbp1_extend_func :%d\n"	
				  "Delta Machine diagonal_rod=%f,radius=%f,available radius=%f,z_home_pos=%f,segments_per_second=%f\n"
				  "Delta Radius adjustment: %f,%f,%f\n"
				  "Delta Diagonal rod adjustment: %f,%f,%f\n"
				  "Delta EndStop x,y,z: %f,%f,%f\n"
				  "Delta deploy start location:(%f, %f, %f), end location:(%f, %f, %f)\n" 
				  "Delta retract start location:(%f, %f, %f), end location:(%f, %f, %f)\n"
				  "Volume: X%d Y%d Z%d \n"	
				  "Has heated Bed: %d\n"
				  "Dangerous Thermistor temperature: %d\n"
			      "Dangerous Thermocouple temperature: %d\n"	
				  "Max heat pwm, hotend:%d, bed:%d\n"
				  "Axis Max speed X:%f Y:%f Z:%f E:%f \n"
				  "Axis direction invert X:%d Y:%d Z:%d E:%d\n"
				  "pa.measure_ext1:%d \n"
				  "pa.measure_ext2:%d \n"
				  "pa.measure_ext3:%d \n"
				  "Extruder num:%d\n"
				    ,machine_type
				    ,pa.bbp1_extend_func
					,pa.diagonal_rod, pa.radius, pa.delta_print_radius, pa.z_home_pos, pa.segments_per_second
					,pa.radius_adj[0], pa.radius_adj[1], pa.radius_adj[2]
					,pa.diagonal_rod_adj[0], pa.diagonal_rod_adj[1], pa.diagonal_rod_adj[2]
					,pa.endstop_adj[0], pa.endstop_adj[1], pa.endstop_adj[2] 
					,pa.delta_deploy_start_location[0],	pa.delta_deploy_start_location[1], pa.delta_deploy_start_location[2],
						pa.delta_deploy_end_location[0], pa.delta_deploy_end_location[1], pa.delta_deploy_end_location[2] 
    				,pa.delta_retract_start_location[0], pa.delta_retract_start_location[1], pa.delta_retract_start_location[2],
						pa.delta_retract_end_location[0], pa.delta_retract_end_location[1],	pa.delta_retract_end_location[2]
					,(int)pa.x_max_length, (int)pa.y_max_length, (int)pa.z_max_length
					,pa.has_bed
	 				,pa.dangerousThermistor
	 				,pa.dangerousThermocouple
					,pa.max_heat_pwm_hotend,	pa.max_heat_pwm_bed
				    ,pa.max_feedrate[0], pa.max_feedrate[1], pa.max_feedrate[2], pa.max_feedrate[3]
            		,pa.invert_x_dir, pa.invert_y_dir, pa.invert_z_dir, pa.invert_e_dir
    				,pa.measure_ext1, pa.measure_ext2, pa.measure_ext3
					,pa.ext_count
			);
    gcode_send_response_remote(send_buf);



	memset(send_buf, 0, sizeof(send_buf));
	strcpy(send_buf, "extruder offset:\n");
	for (i = 0; i < pa.ext_count; i++) {
		sprintf(send_buf + strlen(send_buf), "\r ext%d x:%f, y:%f, z:%f\n"  
							,i, pa.ext_offset[X_AXIS][i], pa.ext_offset[Y_AXIS][i], pa.ext_offset[Z_AXIS][i]);
	}
	sprintf(send_buf + strlen(send_buf), 
			"Motor steps_per_unit X:%f Y:%f Z:%f E:%f\n"
			"Homing feedrate X:%f Y:%f Z:%f E:%f\n"
			"Stepper current X:%d Y:%d Z:%d E0:%d E1:%d\n"
			"Dynamic current set:%d\n"
			"Stepper mircostep X:%d Y:%d Z:%d E0:%d E1:%d\n"
			"Retract length:%f, feedrate:%f\n"
			"Retract recover length:%f, feedrate:%f\n"
			"Maximum Acceleration X:%ld Y:%ld Z:%ld E:%ld\n"
			"Acceleration: %f\n"
			"Advanced variables \n \rmax XY jerk:%f\n \rmax Z jerk:%f\n \r max E jerk:%f\n\r min feedrate:%f, min travel feedrate:%f\n"
					,pa.axis_steps_per_unit[0], pa.axis_steps_per_unit[1], pa.axis_steps_per_unit[2], pa.axis_steps_per_unit[3]
            		,pa.homing_feedrate[0], pa.homing_feedrate[1], pa.homing_feedrate[2], pa.homing_feedrate[3]
					,pa.axis_current[0], pa.axis_current[1], pa.axis_current[2], pa.axis_current[3], pa.axis_current[4]
					,pa.auto_current
		            ,pa.axis_ustep[0], pa.axis_ustep[1], pa.axis_ustep[2], pa.axis_ustep[3], pa.axis_ustep[4]
            		,pa.retract_length, pa.retract_feedrate
            		,pa.retract_recover_length, pa.retract_recover_feedrate
		            ,pa.max_acceleration_units_per_sq_second[0], pa.max_acceleration_units_per_sq_second[1], 
							pa.max_acceleration_units_per_sq_second[2], pa.max_acceleration_units_per_sq_second[3]
            		,pa.move_acceleration
					,pa.max_xy_jerk, pa.max_z_jerk, pa.max_e_jerk, pa.minimumfeedrate, pa.mintravelfeedrate
			);
    gcode_send_response_remote(send_buf);

	memset(send_buf, 0, sizeof(send_buf));
	sprintf(send_buf, 
			"Auto Level enable:%d, device type:%d, down rate:%f\n"
			"endstop extend:%d, retract:%d, raiseBeforeProbing:%f, raiseBetweenProbing:%f\n"
			"device offset:%f,%f,%f\n"
			"Grid points:%d, left:%f, right:%f,  front:%f, back:%f\n"
            "Enstop invert X:%d, Y:%d, Z:%d, Autolevel:%d\n"
			"BBP1S DUAL XY mode:%d\n"
					,pa.autoLeveling, pa.probeDeviceType, pa.autolevel_down_rate
					,pa.servo_endstop_angle[0], pa.servo_endstop_angle[1], pa.zRaiseBeforeProbing, pa.zRaiseBetweenProbing
					,pa.endstopOffset[X_AXIS], pa.endstopOffset[Y_AXIS], pa.endstopOffset[Z_AXIS]
					,pa.probeGridPoints, pa.probeLeftPos, pa.probeRightPos, pa.probeFrontPos, pa.probeBackPos
                    ,pa.x_endstop_invert, pa.y_endstop_invert, pa.z_endstop_invert, pa.autolevel_endstop_invert
                    ,pa.bbp1s_dual_xy_mode
			);
    gcode_send_response_remote(send_buf);
}

