/*
 * Unicorn 3D Printer Firmware
 * gcode.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#include "common.h"
#include "fan.h"
#include "led.h"
#include "heater.h"
#include "motion.h"
#include "stepper.h"
#include "planner.h"
#include "sdcard.h"
#include "parameter.h"
#include "unicorn.h"

#define BUFFER_SIZE  (8192)

static bool stop = false;

float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float offset[3] = {0.0, 0.0, 0.0};

char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

/* 100 -> original
 * 200 -> Factor 2
 * 50  -> Factor 0.5
 */
volatile signed short feedmultiply = 100; 

/* 0 --> Exteruder 1 
 * 1 --> Extruder 2
 */
unsigned char active_extruder = 0;		

signed short feedrate = 1800;


#ifdef RETRACT
static bool retracted = false;
static float retract_length = RETRACT_LENGTH;
static float retract_feedrate = RETRACT_FEEDRATE;
static float retract_recover_length = RETRACT_RECOVER_LENGTH;
static float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
#endif

/*
 * gcode process result
 */
enum process_reply_t {
    NO_REPLY,
    SEND_REPLY,
};

/*
 * gcode parser
 */
typedef struct {
	char     buffer[BUFFER_SIZE];
    int      comment : 1;
	char     *pos;
} parser_t;
static parser_t parser;

/*
 * gcode string processor
 */
static int32_t get_int(char chr)
{
    char *ptr = strchr(parser.pos, chr);
    return ptr ? strtol(ptr + 1, NULL, 10) : 0;
}

static uint32_t get_uint(char chr)
{
	char *ptr = strchr(parser.pos, chr);
	return ptr ? strtoul(ptr + 1, NULL, 10) : 0;
}

static float get_float(char chr)
{
	char *ptr = strchr(parser.pos, chr);
	return ptr ? strtod(ptr + 1, NULL) : 0;
}

static uint32_t get_bool(char chr)
{
	return get_int(chr) ? 1 : 0;
}

static const char* get_str(char chr)
{
	char *ptr = strchr(parser.pos, chr);
	return ptr ? ptr + 1 : NULL;
}

static int has_code(char chr)
{
	return strchr(parser.pos, chr) != NULL;
}

static uint8_t get_command(void)
{
	return parser.pos[0];
}

static char* trim_line(char* line)
{
	int i;
	char command_chars[] = {'G','M','T'};
	
	while (*line) {
		for (i = 0; i < sizeof(command_chars); i++) {
			if (*line == command_chars[i]) {
				return line;
            }
		}

		line++;
	}
	return line;
}

static void get_coordinates(void)
{
    int i = 0;        
    short next_feedrate;
    
    for (i = 0; i < NUM_AXIS; i++) {
        if (has_code(axis_codes[i])) {
            destination[i] = get_float(axis_codes[i]);
        } else {
            destination[i] = current_position[i];
        }
    }

    if (has_code('F')) {
        next_feedrate = get_int('F');
        if (next_feedrate > 0) {
            feedrate = next_feedrate;
        }
    }
}

static void get_arc_coordinates(void)
{
    get_coordinates();
    
    if (has_code('I')) {
        offset[0] = get_float('I');
    } else {
        offset[0] = 0.0f;
    }

    if (has_code('J')) {
        offset[1] = get_float('J');
    } else {
        offset[1] = 0.0f;
    }
}

//test
int counter = 0;
static void prepare_move(void)
{
    int i = 0;
    long help_feedrate = 0;

    if (pa.min_software_endstops) { 
        if (destination[X_AXIS] < 0) {
            destination[X_AXIS] = 0.0;
        }
        if (destination[Y_AXIS] < 0) {
            destination[Y_AXIS] = 0.0;
        }
        if (destination[Z_AXIS] < 0) {
            destination[Z_AXIS] = 0.0;
        }
    }
        
    if (pa.max_software_endstops) {
        if (destination[X_AXIS] > pa.x_max_length) {
            destination[X_AXIS] = pa.x_max_length;
        }
        if (destination[Y_AXIS] > pa.y_max_length) {
            destination[Y_AXIS] = pa.y_max_length;
        }
        if (destination[Z_AXIS] > pa.z_max_length) {
            destination[Z_AXIS] = pa.z_max_length;
        }
    }
    
    if (destination[E_AXIS] > current_position[E_AXIS]) {
        help_feedrate = ((long)feedrate * (long)feedmultiply);
    } else {
        help_feedrate = ((long)feedrate * (long)100);
    }

#if 0 
	fprintf(stderr, "[%5d] POS -> X%3.2f Y%3.2f Z%3.2f E%3.2f F%d\n\r",
            ++counter,
            destination[0],
            destination[1],
            destination[2],
            destination[3],
            (int)feedrate);
#endif 

    plan_buffer_line(destination[X_AXIS], 
                     destination[Y_AXIS], 
                     destination[Z_AXIS],
                     destination[E_AXIS],
                     help_feedrate / 6000.0,
                     active_extruder);

    for (i = 0; i < NUM_AXIS; i++) {
        current_position[i] = destination[i];
    }
}

static void prepare_arc_move(char isclockwise)
{
    float r;
    long help_feedrate = 0;
    unsigned char i = 0;

    r = hypot(offset[X_AXIS], offset[Y_AXIS]);
    
    if (destination[E_AXIS] > current_position[E_AXIS]) {
        help_feedrate = ((long)feedrate * (long)feedmultiply);
    } else {
        help_feedrate = ((long)feedrate * (long)100);
    }

    /* Trace the arc */
    mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, 
           help_feedrate / 6000.0, r, isclockwise, active_extruder);

	/* As far as the parser is concerned, the position is now == target. In reality the
	 * motion control system might still be processing the action and the real tool position
	 * in any intermediate location.
     */
    for (i = 0; i < NUM_AXIS; i++) {
        current_position[i] = destination[i];
    }
}

#ifdef RETRACT
static void retract(bool retracting)
{
    if (retracting && !retracted) {
        destination[X_AXIS] = current_position[X_AXIS]; 
        destination[Y_AXIS] = current_position[Y_AXIS]; 
        destination[Z_AXIS] = current_position[Z_AXIS];
        destination[E_AXIS] = current_position[E_AXIS];

        current_position[E_AXIS] += retract_length;
        plan_set_e_position(current_position[E_AXIS]);

        float feedrate_old = feedrate;
        feedrate = retract_feedrate * 60;
        prepare_move();

        retracted = true;
        feedrate = feedrate_old;
    } else if (!retracting && retracted){
        destination[X_AXIS] = current_position[X_AXIS]; 
        destination[Y_AXIS] = current_position[Y_AXIS]; 
        destination[Z_AXIS] = current_position[Z_AXIS];
        destination[E_AXIS] = current_position[E_AXIS];

        current_position[E_AXIS] -= retract_length;
        current_position[E_AXIS] -= retract_recover_length;
        plan_set_e_position(current_position[E_AXIS]);

        float feedrate_old = feedrate;
        feedrate = retract_recover_feedrate * 60;
        prepare_move();

        retracted = false;
        feedrate = feedrate_old;
    }
}
#endif

static void homing_axis(void) 
{
    uint8_t reverse = 0; 

    if (has_code(axis_codes[X_AXIS])) {
        if (pa.x_home_dir == 1) {
            reverse = 1;
        } else {
            reverse = 0;
        }
        stepper_homing_axis(X_AXIS, reverse);
    }

    if (has_code(axis_codes[Y_AXIS])) {
        if (pa.x_home_dir == 1) {
            reverse = 1;
        } else {
            reverse = 0;
        }
        stepper_homing_axis(Y_AXIS, reverse);
    }
    
    if (has_code(axis_codes[Z_AXIS])) {
        if (pa.x_home_dir == 1) {
            reverse = 1;
        } else {
            reverse = 0;
        }
        stepper_homing_axis(Z_AXIS, reverse);
    }

    if (has_code(axis_codes[X_AXIS])) {
        stepper_wait_for_lmsw(X_AXIS);
    }

    if (has_code(axis_codes[Y_AXIS])) {
        stepper_wait_for_lmsw(Y_AXIS);
    }

    if (has_code(axis_codes[Z_AXIS])) {
        stepper_wait_for_lmsw(Z_AXIS);
    }
}
/*
 * -----------------------------------------------------------------
 * Gcode command processor
 * Implemented Codes
 * look here for descriptions of gcodes: 
 * http://linuxcnc.org/handbook/gcode/g-code.html
 * http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 * -----------------------------------------------------------------
 * G0   -> G1
 * G1   - Coordinated Movement X Y Z E
 * G2   - CW ARC
 * G3   - CCW ARC
 * G4   - Dwell S<seconds> or P<milliseconds>
 * G10  - retract filament according to setting of M207
 * G11  - retract recover according to setting of M208
 * G28  - Home all Axis
 * G90  - Use Absolute Coordinates
 * G91  - Use Relative Coordinates
 * G92  - Set current position to coordinates given
 * 
 * RepRap M Codes
 * M104 - Set extruder target temp
 * M105 - Read current temp
 * M106 - Fan 1 on
 * M107 - Fan 1 off
 * M109 - Wait for extruder current temp to reach target temp.
 * M114 - Display current position
 * 
 * Custom M Codes
 * M20  - List SD card
 * M21  - Init SD card
 * M22  - Release SD card
 * M23  - Select SD file (M23 filename.g)
 * M24  - Start/resume SD print
 * M25  - Pause SD print
 * M26  - Set SD position in bytes (M26 S12345)
 * M27  - Report SD print status
 * M28  - Start SD write (M28 filename.g)
 * M29  - Stop SD write
 *        <filename> - Delete file on sd card
 * M42  - Set output on free pins (not implemented)
 * M44  - Boot From ROM (load bootloader for uploading firmware)
 * M80  - Turn on Power Supply (not implemented)
 * M81  - Turn off Power Supply (not implemented)
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 * M84  - Disable steppers until next move, 
 *		  or use S<seconds> to specify an inactivity timeout, 
 *        after which the steppers will be disabled.	S0 to disable the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92  - Set axis_steps_per_unit - same syntax as G92
 * M93  - Send axis_steps_per_unit
 * M115 - Capabilities string
 * M119 - Show Endstop State 
 * M140 - Set bed target temp
 * M176 - Fan 2 on
 * M177 - Fan 2 off
 * M190 - Wait for bed current temp to reach target temp.
 * M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 * M202 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 * M203 - Set temperture monitor to Sx
 * M204 - Set default acceleration: S normal moves T filament only moves 
 *        (M204 S3000 T7000) in mm/sec^2
 * M205 - advanced settings:	minimum travel speed S=while printing T=travel only,  
 *        X=maximum xy jerk, Z=maximum Z jerk
 * M206 - set additional homing offset
 * M207 - set homing feedrate mm/min (M207 X1500 Y1500 Z120)
 * 
 * M220 - set speed factor override percentage S=factor in percent 
 * M221 - set extruder multiply factor S100 --> original Extrude Speed 
 *
 * Note: M301, M303, M304 applies to currently selected extruder.	Use T0 or T1 to select.
 * M301 - Set Heater parameters P, I, D, S (slope), B (y-intercept), W (maximum pwm)
 * M303 - PID relay autotune S<temperature> sets the target temperature. 
 *        (default target temperature = 150C)
 * M304 - Calculate slope and y-intercept for HEATER_DUTY_FOR_SETPOINT formula.
 *        Caution - this can take 30 minutes to complete and will heat the hotend 
 *				   to 200 degrees.
 * 
 * M400 - Finish all moves
 *
 * M510 - Invert axis, 0=false, 1=true (M510 X0 Y0 Z0 E1)
 * M520 - Set maximum print area (M520 X200 Y200 Z150)
 * M521 - Disable axis when unused (M520 X0 Y0 Z1 E0)
 * M522 - Use software endstops I=min, A=max, 0=false, 1=true (M522 I0 A1)
 * M523 - Enable min endstop input 1=true, -1=false (M523 X1 Y1 Z1)
 * M524 - Enable max endstop input 1=true, -1=false (M524 X-1 Y-1 Z-1)
 * M525 - Set homing direction 1=+, -1=- (M525 X-1 Y-1 Z-1)
 * M526 - Invert endstop inputs 0=false, 1=true (M526 X0 Y0 Z0)
 *  
 * Note: M530, M531 applies to currently selected extruder.  Use T0 or T1 to select.
 * M530 - Set heater sensor (thermocouple) type B (bed) E (extruder) (M530 E11 B11)
 * M531 - Set heater PWM mode 0=false, 1=true (M531 E1)
 *  
 * M350 - Set microstepping steps (M350 X16 Y16 Z16 E16 B16)
 * M906 - Set motor current (mA) (M906 X1000 Y1000 Z1000 E1000 B1000) or set all (M906 S1000)
 * M907 - Set motor current (raw) (M907 X128 Y128 Z128 E128 B128) or set all (M907 S128)
 * 
 * M500 - stores paramters in EEPROM
 * M501 - reads parameters from EEPROM 
 *        (if you need to reset them after you changed them temporarily).
 * M502 - reverts to the default "factory settings". 
 *        You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print settings
 * M505 - Save Parameters to SD-Card
 */
static int gcode_process_g(int value) 
{
    switch (value) 
    { 
        case 0:
        case 1:
            /* G0-G1: Coordinated Movement X Y Z E */
            get_coordinates();
            prepare_move();
            break;
        case 2:
            /* G2: CW ARC */
            get_arc_coordinates();
            prepare_arc_move(1);
            break;
        case 3:
            /* G3: CCW ARC */
            get_arc_coordinates();
            prepare_arc_move(0);
            break;
        case 4:
            /* G4: Dwell S<seconds> or P<milliseconds> */
            break;
#ifdef RETRACT
        case 10:
            /* Retract */
            retract(true);
            break;
        case 11:
            /* Retract recover */
            retract(false);
            break;
#endif
        case 21:
            break;
        case 28:
            /* G28: Home all axis one at a time */
            homing_axis();
            break;
        case 90:
            /* G90: Use Absolute Coordinates */
            break;
        case 91:
            /* G91: Use Relative Coordinates */
            break;
        case 92:
        {
            /* G92: Set current position to coordinates given */
            if (!has_code(axis_codes[E_AXIS])) {
                stepper_sync();
            }

            int i;
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(axis_codes[i])) {
                    current_position[i] = get_float(axis_codes[i]);
                }
            }
            plan_set_position(current_position[X_AXIS], 
                              current_position[Y_AXIS],
                              current_position[Z_AXIS],
                              current_position[E_AXIS]);
            break;
        }
        default:
            return NO_REPLY;
    }

    return SEND_REPLY;
}

static int gcode_process_m(int value)
{
    int i;
    int idx = 0; 
    int level = 0;
    double setpoint = 0.0;
    channel_tag heater, fan;
    pid_settings pid;

    switch (value)
    {
        case 20:
            /* M20: list sd files */
            sdcard_list_files();
            return NO_REPLY;
        case 21:
            /* M21: init sd card */
            sdcard_mount();
            break;
        case 22:
            /* M22: release sd card */
            sdcard_unmount();
            break;
        case 23:
            /* M23: select sd file */
            sdcard_select_file(get_str(' '));
            break;
        case 24:
            /* M24: start/resume sd print */
            sdcard_replay_start();
            break;
        case 25:
            /* M25: pause sd print */
            sdcard_replay_pause();
            break;
        case 26:
            /* M26: set sd position in bytes */
            if (has_code('S')) {
                sdcard_set_position(get_uint('S'));
            }
            break;
        case 27:
            /* M27: report sd print status */
            sdcard_print_status();
            return NO_REPLY;
        case 28:
            /* M28: begin write to sd file */
            sdcard_capture_start(get_str(' '));
            break;
        case 29:
            /* M29: stop writing sd file */
            sdcard_capture_stop();
            break;
        case 44:
            /* M44: Load bootloader */
            break;
        case 82:
            /* M82: Set E codes absolute */
            break;
        case 83:
            /* M83: Set E codes relative while in Absolute Coordinates (G90) mode */
            break;
        case 84:
            /* M84: Disable steppers until next move */
            //TODO:

            break;
        case 85:
            /* M85: Set inactivity shudown timer with parameter S<seconds> */
            break;
        case 92:
            /* M92: Set axis_steps_per_unit */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(axis_codes[i])) {
                    pa.axis_steps_per_unit[i] = get_float(axis_codes[i]);
                    axis_steps_per_sqr_second[i] = pa.max_acceleration_units_per_sq_second[i]
                                                   * pa.axis_steps_per_unit[i];
                }
            }
            break;
        case 93:
            /* M93: Send current axis_steps_per_unit */
			fprintf(stderr, "X:%d Y:%d Z:%d E:%d",
                    (int)pa.axis_steps_per_unit[0],
                    (int)pa.axis_steps_per_unit[1],
                    (int)pa.axis_steps_per_unit[2],
                    (int)pa.axis_steps_per_unit[3]);
            break;
        case 104: 
            /* M104: Set extruder target temp */
#if 0
            if (has_code('S')) {
                int idx = 0; 
                if (has_code('T')) {
                    idx = get_int('T');
                } else {
                    if (has_code('P')) {
                        idx = get_int('P');
                    }
                }

                heater = heater_lookup_by_index(idx);
                if (heater) {
                    heater_set_setpoint(heater, get_uint('S'));
                    heater_enable(heater, 1);
                }
            }
#else
            if (has_code('S')) {
                heater = heater_lookup_by_name("heater_ext");
                if (heater) {
                    heater_set_setpoint(heater, get_uint('S'));
                    heater_enable(heater, 1);
                }
            }
#endif
            break;
        case 105: 
            /* M105: Read extruder current temp */
            if (has_code('T')) {
                idx = get_int('T');
            } else {
                if (has_code('P')) {
                    idx = get_int('P');
                }
            }

            heater = heater_lookup_by_index(idx);
            if (heater) {
                //FIXME:
                heater_get_setpoint(heater, &setpoint);
            }

            break;
        case 106:
            /* M106: Fan 1 on */
            if (has_code('S')) {
                level = constrain(get_uint('S'), 0, 255);   
                fan = fan_lookup_by_name("fan_ext");
                level = level * 100 / 255;
                level = level / 3;  //FIXME: Attention Here!!!!
                if (level >= 100) {
                    level = 99;
                }
                fan_enable(fan);
                fan_set_level(fan, level);
            }
            break;
        case 107:
            /* M107: Fan 1 off */
            fan = fan_lookup_by_name("fan_ext");
            fan_disable(fan);
            break;
        case 109:
            /* M109: Wait for extruder heater to reach target */
            if (has_code('S')) {
                heater = heater_lookup_by_name("heater_ext");
                if (heater) {
                    heater_set_setpoint(heater, get_uint('S'));
                    heater_enable(heater, 1);
                    
                    printf("Start perpare heating\n");
                    while (!stop) { 
                        if (heater_temp_reached(heater)) {
                            break;
                        } else {
                            sleep(1);
                        }
                    } 
                    printf("perpare heating done\n");
                }
            }

            break;
        case 110:
            break;
        case 114:
            /* M114: Display current position */
			printf("X:%f Y:%f Z:%f E:%f ",
                    current_position[0],
                    current_position[1],
                    current_position[2],
                    current_position[3]);
            break;
        case 115:
            /* M115: Capabilities string */
            printf("Unicorn 3D printer firmware, version 1.0, Machine type: Pandala box 1\n");
            break;
        case 119:
            /* M119: show endstop state */
            break;
        case 140:
            /* M140: Set bed temperature */
            if (has_code('S')) {
                heater = heater_lookup_by_name("heater_bed");
                if (heater) {
                    heater_set_setpoint(heater, get_uint('S'));
                    heater_enable(heater, 1);
                }
            }
            break;
        case 176:
            /* M176: Fan 2 on */
            if (has_code('S')) {
                level = constrain(get_uint('S'), 0, 255);   
                level = level * 100 / 255;
                fan = fan_lookup_by_name("fan_sys");
                fan_set_level(fan, level);
                fan_enable(fan);
            }
            break; 
        case 177:
            /* M177: Fan2 off */
            fan = fan_lookup_by_name("fan_sys");
            fan_disable(fan);
            break;     
        case 190:
            /* M190: Wait for bed heater to reach target temperature. */
            // TODO:
            break;
        case 201: 
            /* M201: Set maximum acceleration in units/s^2 for print move (M201 X1000 Y1000) */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(axis_codes[i])) {
                    float acc = get_float(axis_codes[i]);
                    pa.max_acceleration_units_per_sq_second[i] = acc;
                    axis_steps_per_sqr_second[i] = acc * pa.axis_steps_per_unit[i];
                }
            }
            break;
        case 202:
            /* M202: max feedrate mm/sec */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(axis_codes[i])) {
                    pa.max_feedrate[i] = get_float(axis_codes[i]);
                }
            }
            break;
        case 203: 
            /* M203: Temperature monitor */
            break; 
        case 204:
            /* M204: Acceleration S normal moves T filmanent only moves */
            if (has_code('S')) {
                pa.move_acceleration = get_float('S');
            }
            if (has_code('T')) {
                pa.retract_acceleration = get_float('T');
            }
            break;
        case 205:
            /* M205: Advanced settings:
             *       minimum travel speed S = while printing, T=travel only,
             *       B=minimum segment time
             *       X=maximum xy jerk
             *       Z=maximum z jerk
             *       E=max E jerk
             */
            if (has_code('S')) {
                pa.minimumfeedrate = get_float('S'); 
            }
            if (has_code('T')) {
                pa.mintravelfeedrate = get_float('T');
            }
            if (has_code('X')) {
                pa.max_xy_jerk = get_float('X');
            }
            if (has_code('Z')) {
                pa.max_z_jerk = get_float('Z');
            }
            if (has_code('E')) {
                pa.max_e_jerk = get_float('E');
            }
            break;
        case 206:
            /* M206: additional homing offset */
            for (i = 0; i < 3; i++) {
                if (has_code(axis_codes[i])) {
                    pa.add_homing[i] = get_float(axis_codes[i]);
                }
            }
            break;
        case 207:
            /* M207: Homing Feedrate mm/min Xnnn Ynnn Znnn */
            for (i = 0; i < 3; i++) {
                if (has_code(axis_codes[i])) {
                    pa.homing_feedrate[i] = get_float(axis_codes[i]);
                }
            }
            break;
        case 220:
            /* M220: S<factor in percent> -> set speed factor override percentage */
            if (has_code('S')) {
                feedmultiply = constrain(get_uint('S'), 20, 200);
            }
            break;
        case 221:
            /* M221: S<factor in percent> -> set extrude factor override percentage */
            if (has_code('S')) {
                //extrudemultiply = constrain(get_uint('S'), 40, 200);
            }
            break;
        case 301:
            /* M301: Set extruder heater PID parameters */
            heater = heater_lookup_by_name("heater_ext");
            if (heater) {
                heater_get_pid_values(heater, &pid);

                if (has_code('P')) { 
                    pid.P = get_uint('P');
                }
                if (has_code('I')) { 
                    pid.I = get_uint('I');
                }
                if (has_code('D')) { 
                    pid.D = get_uint('D');
                }
                if (has_code('S')) { 
                    pid.FF_factor = get_uint('S');
                }
                if (has_code('B')) { 
                    pid.FF_offset = get_uint('B');
                }
                if (has_code('W')) { 

                }
                heater_set_pid_values(heater, &pid);
            }
            break;
        case 303:
            /* M303: PID relay autotune S<temperature>, set the target temperature */
            // TODO:
            break;
        case 304:
            /* M304: Evaluate heater performance */
            break;
        case 400:
            /* M400: finish all moves */
            break;
        case 350:
            /* M350: Set microstepping mode 
             * M350 X[value] Y[value] Z[value] E[value] B[value]
             * M350 S[value] set all motors
             *       1 -> full step
             *       2 -> 1/2 step
             *       4 -> 1/4 step
             *       16-> 1/16 step
             */
            break;
        case 500:
            /* M500: Store parameters in EEEPROM */
            parameter_save_to_eeprom();
            break;
        case 501:
            /* M501: Read parameters from EEPROM,
             *       If you need to reset them after you changed them temporarily.
             */
            parameter_load_from_eeprom();
            break;
        case 502:
            /*
             * M502: Reverts to the default "factory settings".
             *       You still need to store them in EEPROM afterwards if you want to.
             */
            parameter_init();
            break;
        case 503:
            /* M503: show settings */
            parameter_dump();
            break;
        case 505:
            /* M505: */
            parameter_save_to_sd();
            break;
        case 510:
            /* M510: Axis invert */
            if (has_code('X')) {
                pa.invert_x_dir = get_bool('X');
            }
            if (has_code('Y')) {
                pa.invert_y_dir = get_bool('Y');
            }
            if (has_code('X')) {
                pa.invert_z_dir = get_bool('Z');
            }
            if (has_code('X')) {
                pa.invert_e_dir = get_bool('E');
            }

            break;
        case 520:
            /* M520: Maximum Area unit */
            if (has_code('X')) {
                pa.x_max_length = get_int('X');
            }
            if (has_code('Y')) {
                pa.y_max_length = get_int('Y');
            }
            if (has_code('Z')) {
                pa.z_max_length = get_int('Z');
            }
            break;
        case 521:
            /* M521: Disable axis when unused */
            if (has_code('X')) {
                pa.disable_x_en = get_bool('X');
            }
            if (has_code('Y')) {
                pa.disable_y_en = get_bool('Y');
            }
            if (has_code('Z')) {
                pa.disable_z_en = get_bool('Z');
            }
            if (has_code('E')) {
                pa.disable_e_en = get_bool('E');
            }
            break;
        case 522:
            /* M522: Software Endstop */
            if (has_code('I')) {
                pa.min_software_endstops = get_bool('I');
            }
            if (has_code('A')) {
                pa.max_software_endstops = get_bool('A');
            }
            break;
        case 523:
            /* M523: Min Endstop */
            if (has_code('X')) {
                pa.x_min_endstop_aktiv = get_int('X') == 1 ? 1 : -1;
            }
            if (has_code('Y')) {
                pa.y_min_endstop_aktiv = get_int('Y') == 1 ? 1 : -1;
            }
            if (has_code('Z')) {
                pa.z_min_endstop_aktiv = get_int('Z') == 1 ? 1 : -1;
            }
            break;
        case 524:
            /* M524: Max Endstop */
            if (has_code('X')) {
                pa.x_max_endstop_aktiv = get_int('X') == 1 ? 1 : -1;
            }
            if (has_code('Y')) {
                pa.y_max_endstop_aktiv = get_int('Y') == 1 ? 1 : -1;
            }
            if (has_code('Z')) {
                pa.z_max_endstop_aktiv = get_int('Z') == 1 ? 1 : -1;
            }
            break;
        case 525:
            /* M525: Homing Direction */
            if (has_code('X')) {
                pa.x_home_dir = get_int('X') == 1 ? 1 : -1;
            }
            if (has_code('Y')) {
                pa.y_home_dir = get_int('Y') == 1 ? 1 : -1;
            }
            if (has_code('Z')) {
                pa.z_home_dir = get_int('Z') == 1 ? 1 : -1;
            }
            break;
        case 526:
            /* M526: Endstop Invert */
            if (has_code('X')) {
                pa.x_endstop_invert = get_bool('X');
            }
            if (has_code('Y')) {
                pa.y_endstop_invert = get_bool('Y');
            }
            if (has_code('Z')) {
                pa.z_endstop_invert = get_bool('Z');
            }
            break;
        case 530:
            /* M530: Heater Sensor */
            break;
        case 531:
            /* M531: Heater PWM */
            break;
        case 906:
            /* M906: set motor current value in mA using axis codes 
             * M906 X[mA] Y[mA] Z[mA] E[mA] B[mA]
             * M906 S[mA] Set all motors current
             */
            
            break;
        case 907:
            /* M907: Set motor current value (0-255) using axis codes
             * M906 X[value] Y[value] Z[value] E[value] B[value]
             * M906 S[value] Set all motors current
             */
            break;
        default:
            return NO_REPLY;
    }

    return SEND_REPLY;
}

static int gcode_process_t(void)
{
    return SEND_REPLY;
}

int gcode_process_line(void)
{
    int val;
    parser.pos = trim_line(parser.buffer);

    switch (get_command()) {
    case 'G':
        val = get_int('G');
        if (gcode_process_g(val) == NO_REPLY) {
            fprintf(stderr, "gcode_process_m NO_REPLY\n");
        }
        break;

    case 'M':
        val = get_int('M');
        if (gcode_process_m(val) == NO_REPLY) {
            fprintf(stderr, "gcode_process_m NO_REPLY\n");
        }
        break;

    case 'T':
        gcode_process_t();
        break;

    default:
        return NO_REPLY;
    }
    return SEND_REPLY;
}

void *gcode_get_line_from_file(FILE *fp)
{
    return fgets((char *)&parser.buffer, sizeof(parser.buffer), fp);
}

void gcode_set_feed(int multiply)
{
    if (multiply >= 20 && multiply <= 400) {
        feedmultiply = multiply;
        printf("!!!! multiply %d\n", feedmultiply);
    }
}

/*
 * Init gcode parse
 */
int gcode_init(void)
{
    memset(&parser, 0, sizeof(parser_t));
    return 0;
}
/*
 * Delete gcode parser
 */
void gcode_exit(void)
{
    fprintf(stderr, "gcode_exit called.\n");
}

void gcode_stop(void)
{
    if (!stop) {
        stop = true;
    }
}

