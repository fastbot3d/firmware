/*
 * Unicorn 3D Printer Firmware
 * parameters.h
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
#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include <stdbool.h>

#define COREXY         (1)

#define NUM_AXIS       (4)
#define MAX_EXTRUDER   (1)

#define FLASH_VERSION  "F02"

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

/*--------------------------------
 * Calibration variable
 *-------------------------------*/
/* X, Y, Z, E steps per unit */
#ifdef COREXY
#define AXIS_STEP_PER_UNIT {78.7402, 78.7402, 200 * 16 / 8, 760 / 5}
#else
#define AXIS_STEP_PER_UNIT {78.7402, 78.7402, 200 * 16 / 3, 760 / 5}
#endif

#define AXIS_CURRENT {128, 128, 128, 128, 128}
#define AXIS_USTEP {16, 16, 16, 16, 16}

/*----------------------------------
 * MOVEMENT SETTINGS
 *---------------------------------*/
#define MAX_FEEDRATE        {500, 500, 5, 25}     // (mm/sec)    
#define HOMING_FEEDRATE     {3000, 3000, 120}     // (mm/min) !!

/*-----------------------------------
 * Acceleration settings
 *----------------------------------*/
/* X, Y, Z, E maximum start speed for accelerated moves. 
 * E default values are good for skeinforge 40+, for older versions raise them a lot.
 */
/* Axis Normal acceleration mm/s^2 */
#define ACCELERATION         4000

/* Extruder Normal acceleration mm/s^2 */
#define RETRACT_ACCELERATION 3000

#define MAX_XY_JERK          20.0
#define MAX_Z_JERK           0.4
#define MAX_E_JERK           5.0

/* X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts */
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND {9000, 9000, 100, 10000}

/* For the retract (negative Extruder) move this maxiumum Limit of Feedrate is used
 * The next positive Extruder move use also this Limit, 
 * then for the next (second after retract) move the original Maximum (_MAX_FEEDRATE) Limit is used
 */
#define MAX_RETRACT_FEEDRATE 100    //mm/sec

/* Minimum planner junction speed. 
 * Sets the default minimum speed the planner plans for at the end of the buffer and all stops. 
 * This should not be much greater than zero and should only be changed
 * if unwanted behavior is observed on a user's machine when running at very slow speeds.
 */
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/sec)

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

/* If defined the movements slow down when the look ahead buffer is only half full */
#define SLOWDOWN

/*--------------------------------
 * Retract Settings
 *-------------------------------*/
#define RETRACT

#ifdef RETRACT
/* Minnum extruded mm to accept a automatic gcode retraction attempt */
#define MIN_RETRACT 0.1
/* default retract length, positive mm */
#define RETRACT_LENGTH 3
/* default feedrate for retracting (mm/s) */
#define RETRACT_FEEDRATE 25
/* default retract Z-lift */
#define RETRACT_ZLIFT 0
/* default additional recover lenght (mm, added to retract length when recovering */
#define RETRACT_RECOVER_LENGTH 0
/* default feedrate for recovering from retraction (mm/s) */
#define RETRACT_RECOVER_FEEDRATE 20
#endif
/*--------------------------------
 * Endstop Settings
 *-------------------------------*/
/* If your axes move in one direction ONLY when the endstops are triggered, 
 * set [XYZ]_ENDSTOP_INVERT to true here
 */
#define X_ENDSTOP_INVERT 	false
#define Y_ENDSTOP_INVERT 	false
#define Z_ENDSTOP_INVERT 	false

/* If true, axis won't move to coordinates less than zero. */
#define MIN_SOFTWARE_ENDSTOPS false; 

/* If true, axis won't move to coordinates greater than the defined lengths below. */
#define MAX_SOFTWARE_ENDSTOPS true;  

/* 
 * ENDSTOP INPUT ACTIV:	
 *                      1 --> Active
 *                     -1 --> NO ENDSTOP 
 */
#define X_MIN_ACTIV      1
#define X_MAX_ACTIV      1

#define Y_MIN_ACTIV      1
#define Y_MAX_ACTIV      1

#define Z_MIN_ACTIV      1
#define Z_MAX_ACTIV      1

/* 
 * Disables axis when it's not being used.
 */
#define DISABLE_X_EN 	false
#define DISABLE_Y_EN 	false
#define DISABLE_Z_EN 	true
#define DISABLE_E_EN 	false

/* 
 * Inverting axis direction
 */
#define INVERT_X_DIR 	false
#define INVERT_Y_DIR 	true
#define INVERT_Z_DIR 	true
#define INVERT_E_DIR 	false

/* 
 * HOMING SETTINGS:
 * Sets direction of endstops when homing; 1=MAX, -1=MIN
 */
#define X_HOME_DIR      -1
#define Y_HOME_DIR      -1
#define Z_HOME_DIR      -1

/* 
 * Max Length for Prusa Mendel, check the ways of your axis and set this Values
 */
#define X_MAX_LENGTH	200
#define Y_MAX_LENGTH 	200
#define Z_MAX_LENGTH 	200

/* Set additional home offset (M206) */
#define HOMING_OFFSET {0, 0, 0}

typedef struct {
    /* Version and Chksum, dont change this place of this parameters */
    unsigned short chk_sum;		/* 16 bit CRC checksum */
    char version[4];			/* Settings Version like F001 */

    /* Planner Parameter */
    float max_feedrate[NUM_AXIS];
    float homing_feedrate[3];
    float add_homing[3];
    float axis_steps_per_unit[NUM_AXIS];
    unsigned long max_acceleration_units_per_sq_second[NUM_AXIS];
    float minimumfeedrate;
    float retract_acceleration; /* mm/s^2  filament pull-pack and push-forward  
                                 * while standing still in the other axis M204 TXXXX
                                 */
    float max_xy_jerk;          /* speed than can be stopped at once, if i understand correctly. */
    float max_z_jerk;
    float max_e_jerk;
    float mintravelfeedrate;
    float move_acceleration;       

    /* Software Endstops YES / NO */
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
    signed short x_max_length;
    signed short y_max_length;
    signed short z_max_length;

    /* Disable axis when not used */
    unsigned char disable_x_en;
    unsigned char disable_y_en;
    unsigned char disable_z_en;
    unsigned char disable_e_en;

    /* Motor Settings */
    unsigned char axis_current[5];
    unsigned char axis_ustep[5];
} parameter_t;

extern parameter_t pa;

#if defined (__cplusplus)
extern "C" {
#endif

extern int  parameter_init(void);
extern void parameter_exit(void);

extern void parameter_load_from_eeprom(void);
extern void parameter_save_to_eeprom(void);

extern void parameter_save_to_sd(void);

extern void parameter_dump(void);

#if defined (__cplusplus)
}
#endif
#endif
