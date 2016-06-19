/*
 * Unicorn 3D Printer Firmware
 * parameters.h
*/
#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include <stdbool.h>
#include "common.h"

#define FW_VERSION  "0.96"
#define NUM_AXIS             (4)
#define DEFAULT_FAN_LEVEL    (50)
#define DEFAULT_FAN_MAX_LEVEL    (255)
/*------------------------------------------------------------------------
 * Machine Type 
 * 0 -> xyz ;
 * 1 -> delta; 
 * 2 -> corexy
 * ps: use ultimaker as default machine profile 
 *-----------------------------------------------------------------------*/
#define MACHINE_XYZ           (0)
#define MACHINE_DELTA         (1)
#define MACHINE_COREXY        (2)

#define PROBE_DEVICE_PROXIMIRY  (0)
#define PROBE_DEVICE_SERVO		(1)
#define PROBE_DEVICE_Z_PIN		(2)
#define PROBE_DEVICE_FSR		(3)

#define BBP1_EXTEND_FUNC_DUAL_Z  (1)
#define BBP1_EXTEND_FUNC_DUAL_EXTRUDER (2)

#define ULTIMAKER
#define MACHINE_TYPE          MACHINE_XYZ

//#define DELTA
//#define MACHINE_TYPE          MACHINE_DELTA

//#define COREXY
//#define MACHINE_TYPE          MACHINE_COREXY
/*------------------------------------------------------------------------
 * Firmware function selection
 * 1 -> enable
 * 0 -> disable
 *-----------------------------------------------------------------------*/
/* Enable retract support */
#define RETRACT

/* Enable Auto stepper motor current adjustment */
#ifdef DELTA
#define AUTO_CURRENT_ADJUST   (0)
#else
#define AUTO_CURRENT_ADJUST   (1)
#endif

/* Enable Auto level support */
#define AUTO_LEVELING       (1)
#define AUTO_LEVELING_GRID  (1)

/* Enable Servo support */
#define SERVO               (1)

/* If defined the movements slow down when the look ahead buffer is only half full */
#define SLOWDOWN             (1)
/*------------------------------------------------------------------------
 * Delta Settings
 *-----------------------------------------------------------------------*/
/*
 * Make delta curves from many straight lines (linear interpolation)
 * This is a trade-off between visible corners (not enough segments)
 * and processor overload.
 */
#define DELTA_SEGMENTS_PER_SECOND  (100)

/*
 * Center-to-center distance of the holes in the diagonal push rods
 * mm
 */
#define DELTA_DIAGONAL_ROD         (220)

/*
 * Horizontal offset from middle of printer to smooth rod center
 */
#define DELTA_SMOOTH_ROD_OFFSET    (136.5)

/*
 *  Horizontal offset of the universal joints on the end effector
 */
#define DELTA_EFFECTOR_OFFSET      (20) 

/*
 *  Horizontal offset of the universal joints on the end carriages
 */
#define DELTA_CARRIAGE_OFFSET      (12.0)

/*
 * Effective horizontal distance bridged by diagonal push rods
 * when effector is centered
 */
#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET - DELTA_EFFECTOR_OFFSET - DELTA_CARRIAGE_OFFSET)

/*
 * Print surface diameter/2 minus unreachable space
 * avoid collisions with vertical towers
 */
#define DELTA_PRINTABLE_RADUIS    (80.0)

#define SIN_60  (0.8660254037844386)
#define COS_60  (0.5)
#define DELTA_TOWER1_X (-SIN_60 * DELTA_RADIUS)
#define DELTA_TOWER1_Y (-COS_60 * DELTA_RADIUS)

#define DELTA_TOWER2_X ( SIN_60 * DELTA_RADIUS)
#define DELTA_TOWER2_Y ( COS_60 * DELTA_RADIUS)

#define DELTA_TOWER3_X (0.0)
#define DELTA_TOWER3_Y (DELTA_RADIUS)

#define DELTA_DIAGONAL_ROD_2 (DELTA_DIAGONAL_ROD * DELTA_DIAGONAL_ROD)

/*------------------------------------------------------------------------
 * Bed Auto leveling
 * There are 2 different ways to pick the x and y locations to probe:
 * 1. "grid" mode
 *   Probe every point in a rectangular grid
 *   You must specify the rectangle, and the density of simple points
 *   This mode is preferred because there are more measurements.
 *   It used to be call accurate_bed_leveling but grid is more descriptive
 * 2. "3-point" mode
 *   Probe 3 arbitrary points on the bed (that aren't colinear)
 *   You must specify the X & Y coordinates of all 3 points
 *-----------------------------------------------------------------------*/
#ifdef AUTO_LEVELING
    
//#ifdef AUTO_LEVELING_GRID
/* Set the rectangle in which to probe */
#define LEFT_PROBE_BED_POSITION    (15)
#define RIGHT_PROBE_BED_POSITION   (170)
#define BACK_PROBE_BED_POSITION    (180)
#define FRONT_PROBE_BED_POSITION   (20)
/* Set the number of grid points per dimension */
#define AUTO_LEVELING_GRID_POINTS  (2)
//#else
/* 
 * 3 arbitrary points. 
 * A simple cross-product is used to esimate the plane of the print bed
 */
#define ABL_PROBE_PT_1_X           (15)
#define ABL_PROBE_PT_1_Y           (180)
#define ABL_PROBE_PT_2_X           (15)
#define ABL_PROBE_PT_2_Y           (20)
#define ABL_PROBE_PT_3_X           (170)
#define ABL_PROBE_PT_3_Y           (20)
//#endif

/* Offsets to the probe relative to the extruder tip */
#define X_PROBE_OFFSET_FROM_EXTRUDER  (-25)
#define Y_PROBE_OFFSET_FROM_EXTRUDER  (-29)
#define Z_PROBE_OFFSET_FROM_EXTRUDER  (-12.35)

/* Raise Z before homing (G28) for probe clearance */
#define Z_RAISE_BEFORE_HOMING         (4)

/* X and Y axis travel speed between  probes, mm/min */
#define XY_TRAVEL_SPEED               (8000.0)

/* How much the extruder will be raised before traveling to the first point */
#define Z_RAISE_BEFORE_PROBING        (15)

/* How much the extruder will be raised between traveling next probing point */
#define Z_RAISE_BETWEEN_PROBING       (5)

#endif

/*------------------------------------------------------------------------
 * Servo support
 *-----------------------------------------------------------------------*/
#ifdef SERVO
/* Number of servos */
#define NUM_SERVO            (1)

/* Servo endstop */
#define SERVO_ENDSTOP        (1)
#define SERVO_ENDSTOP_ANGLE  {90, 0}
#endif

/*------------------------------------------------------------------------
 * Calibration variable
 *-----------------------------------------------------------------------*/
/* 
 * X, Y, Z, E steps per unit 
 */
#ifdef ULTIMAKER
/* ultimaker 1 : 32 microstep, 3mm */
#define AXIS_STEP_PER_UNIT {157.4804, 157.4804, 200 * 32 / 3, 1520 / 5 }
#endif

#ifdef DELTA
/* Kossel Mini : 32 microstep, 1.75mm */
#define AXIS_STEP_PER_UNIT {200.0, 200.0, 200.0, 298 }  //478
#endif

#ifdef COREXY
/* H-Bot : 16 microstep, 1.75mm */
#define AXIS_STEP_PER_UNIT {78.7402, 78.7402, 200 * 16 / 8, 1302 / 5 }  
#endif

/* 
 * Max Length of each axis
 */
/* Ultimaker 1 */
#ifdef ULTIMAKER
#define X_MAX_LENGTH	200
#define Y_MAX_LENGTH 	200
#define Z_MAX_LENGTH 	200
#endif

#ifdef COREXY
#define X_MAX_LENGTH	180
#define Y_MAX_LENGTH 	180
#define Z_MAX_LENGTH 	180
#endif

/* Top and center of the Cartesian print volume */
#define MANUAL_X_HOME_POS (0)
#define MANUAL_Y_HOME_POS (0)
#define MANUAL_Z_HOME_POS (251.5)  //251.5

#ifdef DELTA
/* Travel limit after homing */
#define X_MAX_POS  DELTA_PRINTABLE_RADUIS 
#define X_MIN_POS -DELTA_PRINTABLE_RADUIS 
#define Y_MAX_POS  DELTA_PRINTABLE_RADUIS 
#define Y_MIN_POS -DELTA_PRINTABLE_RADUIS 
#define Z_MAX_POS  MANUAL_Z_HOME_POS
#define Z_MIN_POS  0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)
#endif

/* 
 * Inverting axis direction
 */
/* Ultimaker 1 */
#ifdef ULTIMAKER
#define INVERT_X_DIR 	true
#define INVERT_Y_DIR 	false
#define INVERT_Z_DIR 	true
#define INVERT_E_DIR 	false
#endif

#ifdef COREXY
#define INVERT_X_DIR 	true
#define INVERT_Y_DIR 	false
#define INVERT_Z_DIR 	true
#define INVERT_E_DIR 	false
#endif

#ifdef DELTA
#define INVERT_X_DIR 	true
#define INVERT_Y_DIR 	true
#define INVERT_Z_DIR 	true
#define INVERT_E_DIR 	true
#endif

/*
 * Axis default current
 */
#ifdef DELTA
#define AXIS_CURRENT {500, 500, 500, 450, 450, 450, 500}
#else
#define AXIS_CURRENT {800, 800, 450, 450, 450, 450, 500}
#endif
/* 
 * Axis default microstep
 */
#define AXIS_USTEP {32, 32, 32, 32, 32, 32, 32}
#define AXIS_RELATIVE_MODES {false, false, false, false, false}

/* 
 * Disables axis when it's not being used.
 * Not used!
 */
#define DISABLE_X_EN 	false
#define DISABLE_Y_EN 	false
#define DISABLE_Z_EN 	false
#define DISABLE_E_EN 	false

/*------------------------------------------------------------------------
 * Endstop Settings
 *-----------------------------------------------------------------------*/
/* If your axes move in one direction ONLY when the endstops are triggered, 
 * set [XYZ]_ENDSTOP_INVERT to true here
 */
#define X_ENDSTOP_INVERT 	false
#define Y_ENDSTOP_INVERT 	false
#define Z_ENDSTOP_INVERT 	false

/* If true, axis won't move to coordinates less than zero. */
#define MIN_SOFTWARE_ENDSTOPS true; 

/* If true, axis won't move to coordinates greater than the defined lengths below. */
#define MAX_SOFTWARE_ENDSTOPS true;  

/* 
 * ENDSTOP INPUT ACTIV:	
 *  1 --> Active
 * -1 --> NO ENDSTOP 
 */
#define X_MIN_ACTIV      1
#define X_MAX_ACTIV      1

#define Y_MIN_ACTIV      1
#define Y_MAX_ACTIV      1

#define Z_MIN_ACTIV      1
#define Z_MAX_ACTIV      1

/* 
 * Homing Setting
 * Sets direction of endstops when homing; 
 *  1 -> homing to max endstop
 * -1 -> homing to min endstop
 */
#define X_HOME_DIR      -1
#define Y_HOME_DIR      -1
#define Z_HOME_DIR      -1

/* Set additional home offset (M206) */
#define HOMING_OFFSET {0, 0, 0}

/*------------------------------------------------------------------------
 * Temperature Setting
 * Not used!
 *-----------------------------------------------------------------------*/
/*
 * Thermistor convert with table:
 * 1 is 100k thermistor B57540G0104F000
 * 2 is 200k thermistor
 * 3 is mendel-parts thermistor
 * 4 is 10k thermistor
 * 5 is ParCan supplied 104GT-2 100K 
 * 6 is EPCOS 100k B57560G1104F
 * 7 is 100k Honeywell thermistor 135-104LAG-J01
 *
* Thermistor convert with formular
 * 11 EPCOS 100K Thermistor (B57540G0104F000)
 * 13 EPCOS 10K Thermistor (B57550G103J); RS 484-0149
 * 14 RRRF 10K Thermistor
 * 15 RRRF 100K Thermistor; RS 198-961
 * 16 EPCOS 100K Thermistor (B57560G1104F)
 * 17 Honeywell 100K Thermistor (135-104LAG-J01)
 * 
 * AD595
 * 50 get Temperatur with AD595
 */
#define THERMISTORHEATER 11
#define THERMISTORBED 11

#define MAX_DANGEROUS_CELSIUS  280
#define MAX_THERMOCOUPLE_DANGEROUS_CELSIUS  1100

/*------------------------------------------------------------------------
 * Movement Settings
 *-----------------------------------------------------------------------*/
#ifdef DELTA
#define MAX_FEEDRATE        {500, 500, 500, 25}   // (mm/sec)   
#else
#define MAX_FEEDRATE        {500, 500, 5, 25}     // (mm/sec)   
#endif

#ifdef DELTA
#define HOMING_FEEDRATE     {3000, 3000, 3000, 0}  // (mm/min)
#else
#define HOMING_FEEDRATE     {3000, 3000, 240, 0}  // (mm/min)
#endif
/*------------------------------------------------------------------------
 * Acceleration Settings
 *-----------------------------------------------------------------------*/
/* X, Y, Z, E maximum start speed for accelerated moves. 
 */
/* Axis Normal acceleration mm/s^2 for printing */
#define ACCELERATION         4000

/* Extruder Normal acceleration mm/s^2 for retracting */
#define RETRACT_ACCELERATION 3000

/* The speed change that does not require acceleration */
#ifdef DELTA
#define MAX_XY_JERK          60.0
#define MAX_Z_JERK           60.0
#define MAX_E_JERK           10.0
#else
#define MAX_XY_JERK          100.0
#define MAX_Z_JERK           2.0
#define MAX_E_JERK           10.0
#endif

/* X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts */
#ifdef DELTA
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND {9000, 9000, 9000, 10000}
#else
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND {9000, 9000, 100, 10000}
#endif

/* For the retract (negative Extruder) move this maxiumum Limit of Feedrate is used
 * The next positive Extruder move use also this Limit, 
 * then for the next (second after retract) move the original Maximum (MAX_FEEDRATE) Limit is used
 */
#define MAX_RETRACT_FEEDRATE 100    // (mm/sec)

/* Minimum planner junction speed. 
 * Sets the default minimum speed the planner plans for at the end of the buffer and all stops. 
 * This should not be much greater than zero and should only be changed
 * if unwanted behavior is observed on a user's machine when running at very slow speeds.
 */
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/sec)

/* minimum feedrate */
#define DEFAULT_MINIMUMFEEDRATE       20.0     
//lkj #define DEFAULT_MINTRAVELFEEDRATE     40.0
#define DEFAULT_MINTRAVELFEEDRATE     5.0

/* Mininum time in microseconds that a movement needs to take if the buffer is emptied */
#define DEFAULT_MINSEGMENTTIME        20000
/*------------------------------------------------------------------------
 * Retract Settings
 *-----------------------------------------------------------------------*/
/* Minnum extruded mm to accept a automatic gcode retraction attempt */
#define MIN_RETRACT 0.1

/* default retract length, positive mm */
#ifdef DELTA
#define RETRACT_LENGTH 1.5
#else
#define RETRACT_LENGTH 2 //3
#endif

/* default feedrate for retracting (mm/s) */
#define RETRACT_FEEDRATE 25

/* default retract Z-lift */
#define RETRACT_ZLIFT 0

/* default additional recover lenght (mm, added to retract length when recovering */
#define RETRACT_RECOVER_LENGTH 0

/* default feedrate for recovering from retraction (mm/s) */
#define RETRACT_RECOVER_FEEDRATE 20
/*------------------------------------------------------------------------
 * Extruder Settings
 *-----------------------------------------------------------------------*/
/* Max allowed extruders */
#define MAX_EXTRUDER   (5)

/* The number of extruders */
#define EXTRUDERS      (1)
/*------------------------------------------------------------------------
 * parameter_t
 *-----------------------------------------------------------------------*/
typedef struct {
    /* Version */
    unsigned long chk_sum;		          /* 16 bit CRC checksum */
    char version[4];			          /* Settings Version like F001 */
    unsigned long pru_checksum;		

    /* Planner Parameter */
    float max_feedrate[NUM_AXIS];
    float homing_feedrate[NUM_AXIS];
    float add_homing[3];
    float axis_steps_per_unit[NUM_AXIS + 2]; //X, Y, Z, E0,E1,E2
    unsigned long max_acceleration_units_per_sq_second[NUM_AXIS];

    float max_xy_jerk;
    float max_z_jerk;
    float max_e_jerk;
    float minimumfeedrate;
    float mintravelfeedrate;
    float minsegmenttime;
    float move_acceleration;       
    
    /* Retract */
    float retract_length;
    float retract_feedrate;
    float retract_zlift;
    float retract_recover_length;
    float retract_recover_feedrate;
    float retract_acceleration; 

    /* Software Endstops */
    unsigned char min_software_endstops;
    unsigned char max_software_endstops;

    /* Homing direction */
    signed short x_home_dir;
    signed short y_home_dir;
    signed short z_home_dir;

    /* Invert endstop signal */
    volatile unsigned char x_endstop_invert;
    volatile unsigned char y_endstop_invert;
    volatile unsigned char z_endstop_invert;

    /* Digital Input for Endstop switch aktiv */
    volatile signed short x_min_endstop_aktiv;
    volatile signed short x_max_endstop_aktiv;
    volatile signed short y_min_endstop_aktiv;
    volatile signed short y_max_endstop_aktiv;
    volatile signed short z_min_endstop_aktiv;
    volatile signed short z_max_endstop_aktiv;

    /* Invert axis direction */
    volatile unsigned char invert_x_dir;
    volatile unsigned char invert_y_dir;
    volatile unsigned char invert_z_dir;
    volatile unsigned char invert_e_dir;

    /* maximum Printing area */
    unsigned short x_max_length;
    unsigned short y_max_length;
    unsigned short z_max_length;

    /* Disable axis when not used */
    unsigned char disable_x_en;
    unsigned char disable_y_en;
    unsigned char disable_z_en;
    unsigned char disable_e_en;

    /* Motor Settings */
    unsigned int  axis_current[MAX_AXIS];
    unsigned int  axis_ustep[MAX_AXIS];

    /* Extruder */
	unsigned int ext_count;
	float ext_offset[NUM_AXIS][MAX_EXTRUDER];

    /* HBP 
     * 1 -> enable hbp
     * 0 -> disable hbp
     **/
	unsigned int has_bed;

    /* Auto current adjust
     * 1 -> enable auto current adjustment
     * 0 -> disable auto current adjustment
     */
    unsigned char auto_current;

    /* Auto slow down
     * 1 -> enable auto slow down
     * 0 -> disable auto slow down
     */
    unsigned char slow_down;
    
    /* machine type : 
     * 0 -> xyz ;
     * 1 -> delta; 
     * 2 -> h-bot/corexy
     */
    unsigned char machine_type;
    
    /*
     * delta machine parameter
     */
    float radius;
    float diagonal_rod;
    float z_home_pos;
    float segments_per_second;

    float endstop_adjust[3];

    /* Servo endstop */
    unsigned int servo_endstop_angle[2]; //0 extend, 1 retract
	int slowdown_percent;

	unsigned char bbp1_extend_func;
	unsigned char thermocouple_max6675_cnnection; //thermocouple max6675 on board
	unsigned char thermocouple_ad597_cnnection;  //extend interface ad597
	unsigned char max_heat_pwm_hotend;
	unsigned char max_heat_pwm_bed;

	unsigned char autoLeveling; //0 for disable,  1 for enable ----! 1 for 3 point, 2 for grid --!
	unsigned char probeDeviceType; //Proximity:0, Servo:1 
	float endstopOffset[3];
	float zRaiseBeforeProbing;
	float zRaiseBetweenProbing;
	float probePoint1[2];
	float probePoint2[2];
	float probePoint3[2];

	float probeLeftPos;
	float probeRightPos;
	float probeBackPos;
	float probeFrontPos;
	unsigned int probeGridPoints;
	unsigned int dangerousThermistor;
	unsigned int dangerousThermocouple;

	/*"1","THERMISTOR_1",	"2","THERMISTOR_2",	"3","Thermocouple_MAX6675",	"4","Thermocouple_AD597" */
	unsigned int measure_ext1;
	unsigned int measure_ext2;
	unsigned int measure_ext3;

	//for delta autolevel
    float delta_deploy_start_location[3];
    float delta_deploy_end_location[3];
    float delta_retract_start_location[3];
    float delta_retract_end_location[3];
  	float radius_adj[3];
  	float diagonal_rod_adj[3];
  	float endstop_adj[3];
  	float delta_print_radius;

    unsigned char bbp1s_dual_xy_mode;
    unsigned char autolevel_endstop_invert;
    float autolevel_down_rate; //autolevel down speed
} parameter_t;

extern parameter_t pa;

#if defined (__cplusplus)
extern "C" {
#endif

extern int  parameter_init(char *dev);
extern void parameter_exit(void);

extern void parameter_restore_default(void);

extern int parameter_load_from_eeprom(void);
extern int parameter_save_to_eeprom(void);

extern int parameter_save_to_sd(void);

extern void parameter_dump(int (*gcode_send_response_remote)(char *));

#if defined (__cplusplus)
}
#endif
#endif
