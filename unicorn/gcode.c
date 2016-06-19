/*
 * Unicorn 3D Printer Firmware
 * gcode.c
 * G-Code Interpreter
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "common.h"
#include "eeprom.h"
#include "parameter.h"
#include "fan.h"
#include "temp.h"
#include "heater.h"
#include "stepper_pruss.h"

#ifdef AUTO_LEVELING
#include "vector.h"
#ifdef AUTO_LEVELING_GRID
#include "qr_solve.h"
#endif
#endif

#ifdef SERVO
#include "servo.h"
#endif

#include "motion.h"
#include "stepper.h"
#include "planner.h"
#include "sdcard.h"
#include "unicorn.h"
#include "gcode.h"

#include "util/Pause.h"

#define BUFFER_SIZE  (256)

static bool stop = false;
static bool thread_started = false;
static pthread_t gcode_thread;
static pthread_t emerg_gcode_thread;
static pthread_t mcode_thread;

static float min_probe_x = 0; 
static float min_probe_y = 0; 
static float max_probe_x = 0; 
static float max_probe_y = 0; 


float delta[3] = {0.0, 0.0, 0.0};

static float bed_level[100][100];
float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float offset[3] = {0.0, 0.0, 0.0};
static  float delta_tower1_x, delta_tower1_y;
static  float delta_tower2_x, delta_tower2_y;
static  float delta_tower3_x, delta_tower3_y;
static float delta_diagonal_rod_2_tower1 = 0;
static float delta_diagonal_rod_2_tower2 = 0;
static float delta_diagonal_rod_2_tower3 = 0;
//static float endstop_adj[3]={0,0,0};
//static float tower_adj[6]= {0,0,0,0,0,0};

/* Determines Absolute or Relative Coordinates */
static bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
static bool relative_mode = false;

char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

volatile signed short feedmultiply = 100; 

/* 0 --> Exteruder 1 
 * 1 --> Extruder 2
 */
volatile unsigned char active_extruder = 0;		
static uint8_t tmp_extruder = 0;

float feedrate = 1800.0;

int extrudemultiply = 100; //100->1 200->2
int extruder_multiply[MAX_EXTRUDER] = {100,100,100};


#ifdef RETRACT
//static bool auto_retract_enbaled = false;
static bool retracted[MAX_EXTRUDER] = 	   {false, false, false};
static bool retracted_swap[MAX_EXTRUDER] = {false, false, false};

#endif

#define X_HOME_RETRACT_MM 3
#define Y_HOME_RETRACT_MM 3
#define Z_HOME_RETRACT_MM 4
static const float home_retract_mm[3] = 
			{ X_HOME_RETRACT_MM, Y_HOME_RETRACT_MM, Z_HOME_RETRACT_MM };     
static float safe_servo_retract_distance = 5;
static void adjust_delta(float cartesian[3]);
static void reset_bed_level();
static void engage_z_probe(void);
static void retract_z_probe(void);
/*
 * gcode process result
 */
enum process_reply_t {
    NO_REPLY,
    SEND_REPLY,
};

static void dock_sled(bool dock, int offset);
static void delta_min_z_pin_dock(bool);

/*
 * gcode parser
 */
typedef struct {
	char     buffer[BUFFER_SIZE];
    int      comment : 1;
	char     *pos;
    FILE     *gcode_fp;
    int      fd_rd;        /* socket/fifo rd fd */
    int      fd_wr;        /* socket/fifo wr fd */
    int      fd_rd_emerg;  /* socket/fifo wr emergency fd */
} parser_t;
static parser_t parser;
//volatile 
matrix_t plan_bed_level_matrix = {
	.matrix = {
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0 
	},
};

static int rootfs_is_mmcblk0()
{
	struct stat st;
	system("cat /proc/cmdline | grep 'root=/dev/mmcblk0'>/tmp/rootfs.t");
	if (stat("/tmp/rootfs.t", &st) == 0 && st.st_size == 0){ 
			return 0;
	} else 
			return 1;
}

static int scan_dir(const char *dirname, char *suffix)
{
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de; 
    dir = opendir(dirname);
    if(dir == NULL)
        return -1; 
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/'; 
    while((de = readdir(dir))) {
        if (de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' ||
                 (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        if ((de->d_name[0] == '.') || strstr(de->d_name, "LOST.DIR"))
            continue;
		#if 0
        if(de->d_type == DT_DIR) {
            //printf("scan dir:%s\n", de->d_name);
            strcpy(filename, de->d_name);
            scan_dir(devname, suffix);
        }
		#endif
        char * p_gcode = strrchr(de->d_name, '.');
        if ((p_gcode !=NULL) && *(p_gcode+1) == 'g' && *(p_gcode+2) == 'c' && *(p_gcode+3) == 'o' 
                && *(p_gcode+4) == 'd' && *(p_gcode+5) == 'e' && *(p_gcode+6) == '\0') {
            struct stat file_stat;
            strcpy(filename, de->d_name);
            stat(devname, &file_stat);
			char tmp[1024]={0}; 
			sprintf(tmp, "%s %lld\n", devname, (long long)file_stat.st_size); 
			gcode_send_response_remote(tmp);
			printf("found gcode:%s, size:%lld\n", devname, (long long)file_stat.st_size);
        }
        #if 0
        char * p_gcode = strstr(de->d_name, suffix);
        if (p_gcode != NULL) {
            struct stat file_stat;
            strcpy(filename, de->d_name);
            int size = stat(devname, &file_stat);
            printf("found gcode:%s, size:%lld\n", devname, (long long)file_stat.st_size);
        }
        #endif
    }
    closedir(dir);
    return 0;
}

static bool is_android_system()
{
	struct stat sb;
	if (stat("/system/build.prop", &sb) == -1) {
   		return false; 
	}
	return true; 
}

//1 , success
//0 , failed
static int androidPathIsMounted(char * arg, char *retMountPath)
{
    char devPath[1024] = {0};
    char mountPath[1024] = {0};
    char mountType[1024] = {0};
    char mountOption[1024] = {0};
    int arg1,arg2;
    int found = 0;

    FILE *fp = fopen("/proc/mounts", "r");

    if( fp == NULL ){
        printf("error, failed to open /proc/mounts\n");
        return found;
    }

    while( !feof(fp) ) {
        // /dev/sda4 /disk1 ext4 rw,relatime,data=ordered 0 0
        fscanf(fp, "%s %s %s %s %d %d\n", devPath, mountPath, mountType, mountOption, &arg1, &arg2);
        //printf("%s, %s, %s, %s, %d, %d \n", devPath, mountPath, mountType, mountOption, arg1, arg2);
        if(strstr(mountPath, arg)){
            found = 1;
            strcpy(retMountPath, mountPath);
            break;
        }
    }

    fclose(fp);

    return found;
}

//1 , success
//0 , failed
static int deviceIsMounted(char * arg, char *retMountPath)
{
    char devPath[1024] = {0};
    char mountPath[1024] = {0};
    char mountType[1024] = {0};
    char mountOption[1024] = {0};
    int arg1,arg2;
    int found = 0;

    FILE *fp = fopen("/proc/mounts", "r");

    if( fp == NULL ){
        printf("error, failed to open /proc/mounts\n");
        return found;
    }

    while( !feof(fp) ) {
        // /dev/sda4 /disk1 ext4 rw,relatime,data=ordered 0 0
        fscanf(fp, "%s %s %s %s %d %d\n", devPath, mountPath, mountType, mountOption, &arg1, &arg2);
        //printf("%s, %s, %s, %s, %d, %d \n", devPath, mountPath, mountType, mountOption, arg1, arg2);
        if(strstr(devPath, arg)){
            found = 1;
            strcpy(retMountPath, mountPath);
            break;
        }
    }

    fclose(fp);

    return found;
}


/*
 * gcode string processor
 */
static int32_t get_int(char *line, char chr)
{
    char *ptr = strchr(line, chr);
    return ptr ? strtol(ptr + 1, NULL, 10) : 0;
}

static uint32_t get_uint(char *line, char chr)
{
	char *ptr = strchr(line, chr);
	return ptr ? strtoul(ptr + 1, NULL, 10) : 0;
}

static float get_float(char *line, char chr)
{
	char *ptr = strchr(line, chr);
	return ptr ? strtod(ptr + 1, NULL) : 0;
}

static uint32_t get_bool(char *line, char chr)
{
	return get_int(line, chr) ? 1 : 0;
}

static const char* get_str(char *line, char chr)
{
	char *ptr = strchr(line, chr);
	return ptr ? ptr + 1 : NULL;
}

static int has_code(char *line, char chr)
{
	return strchr(line, chr) != NULL;
}

static void remove_str(char *line, char *del)
{
    char *ptr = strstr(line, del);
    if (ptr)
        memset(ptr, 0, strlen(del));
}

static uint8_t get_command(char *line)
{
    int i;
    char command_chars[] = {'G','M','T'};

    while ( *line && (*line !='\n') ) {
        for (i = 0; i < sizeof(command_chars); i++) {
            if (*line == command_chars[i]) {
                return *line;
            }
        }
        line++; 
    }
    return 0;
}

#if 0
static char* trim_line(char* line)
{
	int i;
	char command_chars[] = {'G','M','T'};
	
	while (*line && (*line !='\n')) {
		for (i = 0; i < sizeof(command_chars); i++) {
			if (*line == command_chars[i]) {
				return line;
            }
		}

		line++;
	}
	return line;
}
#endif

static void get_coordinates(char *line)
{
    int i = 0;        
    float next_feedrate;
    
    for (i = 0; i < NUM_AXIS; i++) {
        if (has_code(line, axis_codes[i])) {
            if (relative_mode || axis_relative_modes[i]) {
                destination[i] = current_position[i] + get_float(line, axis_codes[i]);
            } else {
                destination[i] = get_float(line, axis_codes[i]);
            }
        } else {
            destination[i] = current_position[i];
        }
    }

    if (has_code(line, 'F')) {
        next_feedrate = get_float(line, 'F');
        if (next_feedrate > 0) {
            feedrate = next_feedrate;
        }
    }
}

static void get_arc_coordinates(char *line)
{
    get_coordinates(line);
    
    if (has_code(line, 'I')) {
        offset[0] = get_float(line, 'I');
    } else {
        offset[0] = 0.0f;
    }

    if (has_code(line, 'J')) {
        offset[1] = get_float(line, 'J');
    } else {
        offset[1] = 0.0f;
    }
}

void calculate_delta(float cartesian[4])
{
#if 0   //lkj
    /* Front left tower */
    float delta_tower1_x = -SIN_60 * pa.radius;
    float delta_tower1_y = -COS_60 * pa.radius;
    
    /* Front right tower */
    float delta_tower2_x =  SIN_60 * pa.radius;
    float delta_tower2_y = -COS_60 * pa.radius;

    /* Back middle tower */
    float delta_tower3_x = 0.0;
    float delta_tower3_y = pa.radius;

    float delta_diagonal_rod_2 = powf(pa.diagonal_rod, 2);
#endif

    delta[X_AXIS] = sqrt(delta_diagonal_rod_2_tower1 
                         - powf((delta_tower1_x - cartesian[X_AXIS]), 2)
                         - powf((delta_tower1_y - cartesian[Y_AXIS]), 2)
                         ) + cartesian[Z_AXIS];

    delta[Y_AXIS] = sqrt(delta_diagonal_rod_2_tower2 
                         - powf((delta_tower2_x - cartesian[X_AXIS]), 2)
                         - powf((delta_tower2_y - cartesian[Y_AXIS]), 2)
                         ) + cartesian[Z_AXIS];

    delta[Z_AXIS] = sqrt(delta_diagonal_rod_2_tower3 
                         - powf((delta_tower3_x - cartesian[X_AXIS]), 2)
                         - powf((delta_tower3_y - cartesian[Y_AXIS]), 2)
                         ) + cartesian[Z_AXIS];
}


static void prepare_move_raw(void)
{
#if 0 
    float help_feedrate = 0;

    if (destination[E_AXIS] > current_position[E_AXIS]) {
        /* print speed */
        help_feedrate = (feedrate * feedmultiply);
    } else {
        /* travel speed */
        help_feedrate = (feedrate * 100);
    }


        /* Delta */
		int i;
        float difference[NUM_AXIS];
        for (i = 0; i < NUM_AXIS; i++) {
            difference[i] = destination[i] - current_position[i];
        }
        float cartesian_mm = sqrt(powf(difference[X_AXIS], 2) +
                                  powf(difference[Y_AXIS], 2) +
                                  powf(difference[Z_AXIS], 2));

        if (cartesian_mm < 0.000001) {
            cartesian_mm = abs(difference[E_AXIS]);
        }

        if (cartesian_mm < 0.000001) {
            printf("cartesian_mm < 0.000001!!!!!!!!!!\n");
            return;
        }

        float seconds = 6000 * cartesian_mm / help_feedrate;
        int steps = max(1, (int)(pa.segments_per_second * seconds));

        int s;
        for (s = 1; s <= steps; s++) {
            float fraction = (float)(s) / (float)(steps);     
            for (i = 0; i < NUM_AXIS; i++) {
                destination[i] = current_position[i] + difference[i] * fraction;
            }

            calculate_delta(destination);
            plan_buffer_line(delta[X_AXIS], 
                             delta[Y_AXIS], 
                             delta[Z_AXIS],
                             destination[E_AXIS], 
                             help_feedrate / 6000.0,
                             active_extruder);
        }
#else 
    int i;

    calculate_delta(destination);
    plan_buffer_line(delta[X_AXIS],
                     delta[Y_AXIS],
                     delta[Z_AXIS],
                     destination[E_AXIS],
                     feedrate * feedmultiply / 60 / 100,
                     active_extruder);
#endif

    for (i = 0; i < NUM_AXIS; i++) {
        current_position[i] = destination[i];
    }

	/*
	plan_set_position(current_position[X_AXIS],
			current_position[Y_AXIS],
			current_position[Z_AXIS],
			current_position[E_AXIS]);
	stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
			lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
			lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
			lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
	*/
}

int counter = 0;
static void prepare_move(void)
{
    int i = 0;
    float help_feedrate = 0;

#if 1
    if (destination[E_AXIS] > current_position[E_AXIS]) {
        /* print speed */
        help_feedrate = (feedrate * feedmultiply);
    } else {
        /* travel speed */
        help_feedrate = (feedrate * 100);
    }

#else
    /* Do not use feedmultiply for E or Z only moves */
    if ((current_position[X_AXIS] == destination[X_AXIS]) 
            && current_position[Y_AXIS] == destination[Y_AXIS]) {
        help_feedrate = ((long)feedrate * (long)100);
    } else {
        help_feedrate = ((long)feedrate * (long)feedmultiply);
    }
#endif

    if (pa.machine_type == MACHINE_DELTA) {
		float distance_from_center = destination[X_AXIS] * destination[X_AXIS]  + destination[Y_AXIS] * destination[Y_AXIS];
		if (distance_from_center > pa.delta_print_radius * pa.delta_print_radius) {
			if ((abs(destination[X_AXIS]) != 0) && (abs(destination[Y_AXIS]) != 0)) { 
				printf("lkj debug, delta out of range !!!!\n");
				float scale = fabs(destination[X_AXIS] / destination[Y_AXIS]);
				int x_direction = 1, y_direction = 1; 
				if (destination[X_AXIS] < 0)
					x_direction = -1; 
				if (destination[Y_AXIS] < 0)
					y_direction = -1; 
				destination[Y_AXIS] = y_direction * pa.delta_print_radius / sqrt(1+scale * scale);
				destination[X_AXIS] = x_direction * fabs(destination[Y_AXIS]) * scale;
			}
		}

        if (pa.min_software_endstops) {
            if (destination[X_AXIS] < -pa.delta_print_radius) {
                destination[X_AXIS] =  -pa.delta_print_radius;
            }
            if (destination[Y_AXIS] <  -pa.delta_print_radius) {
                destination[Y_AXIS] = -pa.delta_print_radius;
            }
        }
            
        if (pa.max_software_endstops) {
            if (destination[X_AXIS] > pa.delta_print_radius) {
                destination[X_AXIS] = pa.delta_print_radius;
            }
            if (destination[Y_AXIS] > pa.delta_print_radius) {
                destination[Y_AXIS] = pa.delta_print_radius;
            }
        }

        /* Delta */
        float difference[NUM_AXIS];
        for (i = 0; i < NUM_AXIS; i++) {
            difference[i] = destination[i] - current_position[i];
        }
        float cartesian_mm = sqrt(powf(difference[X_AXIS], 2) +
                                  powf(difference[Y_AXIS], 2) +
                                  powf(difference[Z_AXIS], 2));

        if (cartesian_mm < 0.000001) {
            cartesian_mm = abs(difference[E_AXIS]);
        }

        if (cartesian_mm < 0.000001) {
            printf("cartesian_mm < 0.000001!!!!!!!!!!\n");
            return;
        }

        float seconds = 6000 * cartesian_mm / help_feedrate;
        int steps = max(1, (int)(pa.segments_per_second * seconds));
		
		if (abs(difference[Z_AXIS]) >= 10.0) {
        	steps = 1;
		}

        int s;
        for (s = 1; s <= steps; s++) {
            float fraction = (float)(s) / (float)(steps);     
            for (i = 0; i < NUM_AXIS; i++) {
                destination[i] = current_position[i] + difference[i] * fraction;
            }

            calculate_delta(destination);
			if (pa.autoLeveling) {
				adjust_delta(destination);
			}
#if 0
         PLAN_DBG("[%5d] POS -> X%3.2f Y%3.2f Z%3.2f E%3.2f F%d\n\r",
                    ++counter,
                    delta[0] * 200,
                    delta[1] * 200,
                    delta[2] * 200,
                    delta[3],
                    (int)feedrate);
#endif        
            plan_buffer_line(delta[X_AXIS], 
                             delta[Y_AXIS], 
                             delta[Z_AXIS],
                             destination[E_AXIS], 
                             help_feedrate / 6000.0,
                             active_extruder);
        }
    } else {
        /* xyz & corexy,  min endstop  in PRU 
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
        }*/
            
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
        
#if 0
        float difference[NUM_AXIS];
        for (i = 0; i < NUM_AXIS; i++) {
            difference[i] = destination[i] - current_position[i];
        }
        float cartesian_mm = sqrt(powf(difference[X_AXIS], 2) +
                                  powf(difference[Y_AXIS], 2) +
                                  powf(difference[Z_AXIS], 2));
        float max_move_xy_mm = 65535 / max(pa.axis_steps_per_unit[X_AXIS], pa.axis_steps_per_unit[Y_AXIS]);
        float max_move_z_mm = 65535  / pa.axis_steps_per_unit[Z_AXIS];

        //printf("diff_x:%f, diff_y:%f, diff_z:%f, cartesian_mm:%f\n", 
         //       difference[X_AXIS], difference[Y_AXIS], difference[Z_AXIS], cartesian_mm);
        //printf("max_move_xy_mm:%f,max_move_z_mm:%f \n", max_move_xy_mm, max_move_z_mm);

        //if (((difference[X_AXIS] == difference[Y_AXIS] && difference[Y_AXIS] == 0.0f) && fabsf(difference[Z_AXIS]) > max_move_z_mm)
        if ((fabsf(difference[Z_AXIS]) > max_move_z_mm)
            || (cartesian_mm > max_move_xy_mm )) { 
                float max_move = max_move_xy_mm;
                //if ((difference[X_AXIS] == difference[Y_AXIS] && difference[Y_AXIS] == 0.0f) && fabsf(difference[Z_AXIS]) > max_move_z_mm){
                if (fabsf(difference[Z_AXIS]) > max_move_z_mm){
                    max_move = max_move_z_mm;
                } 
                int segments = ceilf(cartesian_mm / max_move);
                //printf("segments:%d, cartesian_mm:%f, max_move:%f \n", segments, cartesian_mm, max_move);
                float fraction = 0.0f;
                while (segments >0) {
                    printf("segments:%d\n", segments);
                    if (segments == 1){
                        fraction = 1;
                    } else {
                        fraction += max_move / cartesian_mm;
                    }
                    for (i = 0; i < NUM_AXIS; i++) {
                        destination[i] = current_position[i] + difference[i] * fraction;
                        //printf("destination[%d]:%f, fraction:%f\n", i, destination[i], fraction);
                    }
                    plan_buffer_line(destination[X_AXIS], 
                                     destination[Y_AXIS], 
                                     destination[Z_AXIS],
                                     destination[E_AXIS], 
                                     help_feedrate / 6000.0,
                                     active_extruder);
                    segments--;
                }
         } else {
            plan_buffer_line(destination[X_AXIS], 
                    destination[Y_AXIS], 
                    destination[Z_AXIS],
                    destination[E_AXIS],
                    help_feedrate / 6000.0,
                    active_extruder);
        }
#endif
#if 1

        PLAN_DBG("[%5d] POS -> X%3.2f Y%3.2f Z%3.2f E%3.2f F%f\n\r",
                    ++counter,
                    destination[0],
                    destination[1],
                    destination[2],
                    destination[3],
                    feedrate);

        plan_buffer_line(destination[X_AXIS], 
                         destination[Y_AXIS], 
                         destination[Z_AXIS],
                         destination[E_AXIS],
                         help_feedrate / 6000.0,
                         active_extruder);
#endif
    }

    /* Save destination as current position */
    for (i = 0; i < NUM_AXIS; i++) {
        current_position[i] = destination[i];
    }

}

static void prepare_arc_move(char isclockwise)
{
    float r;
    float help_feedrate = 0;
    unsigned char i = 0;

    r = hypot(offset[X_AXIS], offset[Y_AXIS]);
    
    if (destination[E_AXIS] > current_position[E_AXIS]) {
        help_feedrate = (feedrate * feedmultiply);
    } else {
        help_feedrate = (feedrate * 100);
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
static void retract(bool retracting, bool swapretract)
{
    if (retracting && !retracted[active_extruder]) {
        destination[X_AXIS] = current_position[X_AXIS]; 
        destination[Y_AXIS] = current_position[Y_AXIS]; 
        destination[Z_AXIS] = current_position[Z_AXIS];
        destination[E_AXIS] = current_position[E_AXIS];

		if (swapretract) {
			current_position[E_AXIS] += pa.retract_length; //pa.retract_length_swap;
		} else {
			current_position[E_AXIS] += pa.retract_length;
		}
        plan_set_e_position(current_position[E_AXIS]);

        float feedrate_old = feedrate;
		retracted[active_extruder]=true;
#if 0
        //Please FIXME!!!!!
        if (pa.retract_feedrate > MAX_RETRACT_FEEDRATE) {
            pa.retract_feedrate = MAX_RETRACT_FEEDRATE;
        }
        feedrate = pa.retract_feedrate * 60;
#endif
        feedrate = pa.retract_feedrate * 60; // mm/s to mm/min

        prepare_move();

		feedrate = feedrate_old;
	} else if(!retracting && retracted[active_extruder]) {
        destination[X_AXIS] = current_position[X_AXIS]; 
        destination[Y_AXIS] = current_position[Y_AXIS]; 
        destination[Z_AXIS] = current_position[Z_AXIS];
        destination[E_AXIS] = current_position[E_AXIS];

		if (swapretract) {
			current_position[E_AXIS] -= (pa.retract_length + pa.retract_recover_length); //(pa.retract_length_swap + pa.retract_recover_length_swap);
		} else {
			current_position[E_AXIS] -= (pa.retract_length + pa.retract_recover_length);
		}
		plan_set_e_position(current_position[E_AXIS]);


		float feedrate_old = feedrate;
		retracted[active_extruder]=false;
#if 0
        if (pa.retract_recover_feedrate > MAX_RETRACT_FEEDRATE) {
            pa.retract_recover_feedrate = MAX_RETRACT_FEEDRATE;
        }
        feedrate = pa.retract_recover_feedrate * 60;
#endif
        feedrate = pa.retract_recover_feedrate * 60;

		prepare_move();

		feedrate = feedrate_old;
	}
}
#endif

static void home_axis(int axis) {
    unsigned int max_length = 0;

    if (axis==X_AXIS || axis==Y_AXIS || axis==Z_AXIS) {
        float feedrate_bak = feedrate; 
           
        if (axis==X_AXIS) {
            max_length = pa.x_max_length;
        } else if (axis==Y_AXIS) {
            max_length = pa.y_max_length;
        } else if (axis==Z_AXIS) {
            max_length = pa.z_max_length;
        }

        current_position[axis] = 0;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

        if (axis==Z_AXIS) {
            if ((pa.machine_type != MACHINE_DELTA) && pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
                destination[Z_AXIS] = current_position[Z_AXIS] + pa.zRaiseBeforeProbing;
                plan_buffer_line(current_position[X_AXIS], 
                        current_position[Y_AXIS],
                        destination[Z_AXIS],
                        current_position[E_AXIS],
                        feedrate / 60.0,
                        active_extruder);
                stepper_sync();
                engage_z_probe();
            }
            if ((pa.machine_type != MACHINE_DELTA) && pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
                dock_sled(false, 0);
        }

     destination[axis] = -1.5 * max_length;
     feedrate = pa.homing_feedrate[axis];
	 //if (axis == Z_AXIS && (pa.autoLeveling && (pa.machine_type != MACHINE_DELTA))) {
    // 	feedrate = pa.homing_feedrate[axis]/4;
	 //}
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
     stepper_sync();

     if ((pa.machine_type != MACHINE_DELTA) && pa.autoLeveling && axis == Z_AXIS) {
         stepper_wait_for_autoLevel();
     } else {
         stepper_wait_for_lmsw(axis);
     }

 
     if (axis == Z_AXIS) { 
         if (pa.autoLeveling && (pa.machine_type != MACHINE_DELTA)) {
             stepper_sync();
             current_position[Z_AXIS] = fabs(pa.endstopOffset[Z_AXIS]);  
             plan_set_position(current_position[X_AXIS],  current_position[Y_AXIS],
                                        current_position[Z_AXIS], current_position[E_AXIS]);
             stepper_set_position(lround(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
                                 lround(current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
                                 lround(current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
                                 lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));

              //printf("debug autolevel, home z pos:%f\n", stepper_get_position_mm(Z_AXIS));

             if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
                 destination[Z_AXIS] = current_position[Z_AXIS] + safe_servo_retract_distance;// home_retract_mm[Z_AXIS];
                 plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],
                                     destination[Z_AXIS], current_position[E_AXIS],
                                     feedrate / 60.0,  active_extruder);
                 stepper_sync();

                 current_position[Z_AXIS] = destination[Z_AXIS]; //stepper_get_position_mm(Z_AXIS);
             }

             if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
                 retract_z_probe();
             if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
                 dock_sled(true, 0);
         }
     }

     destination[axis] = current_position[axis];
     feedrate = feedrate_bak ; 
    }
}

static void homing_axis_plan_buffer(char *line)
{   
    int i = 0;
    bool home_all_axis = false;
    bool has_x = false, has_y = false, has_z = false;

    if (pa.autoLeveling) {
		matrix_set_to_identity(&plan_bed_level_matrix);
		reset_bed_level();
debug_matrix(&plan_bed_level_matrix);
	}

    has_x = has_code(line, axis_codes[X_AXIS]);
    has_y = has_code(line, axis_codes[Y_AXIS]);
    has_z = has_code(line, axis_codes[Z_AXIS]);
    home_all_axis = !(has_x || has_y || has_z);


    if ((pa.machine_type == MACHINE_XYZ) || (pa.machine_type == MACHINE_COREXY)) {
        for(i=0; i < NUM_AXIS; i++) {
            destination[i] = current_position[i];
        }
        if (home_all_axis || has_x) {
            home_axis(X_AXIS);
        }
        if (home_all_axis || has_y) {
            home_axis(Y_AXIS);
        }
        if (home_all_axis || has_z) {
            home_axis(Z_AXIS);
        }

        current_position[E_AXIS] = 0.0;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], 
				current_position[Z_AXIS], current_position[E_AXIS]);
		stepper_set_position(lround(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
				lround(current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
				lround(current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
				lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
		//feedrate = saved_feedrate;
		//feedmultiply = saved_feedmultiply;
	} else if (pa.machine_type == MACHINE_DELTA){
		current_position[X_AXIS] = 0;
		current_position[Y_AXIS] = 0;
		current_position[Z_AXIS] = 0;
		current_position[E_AXIS] = 0;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

		destination[X_AXIS] = 100 * pa.z_home_pos;
		destination[Y_AXIS] = 100 * pa.z_home_pos;
		destination[Z_AXIS] = 100 * pa.z_home_pos;
		destination[E_AXIS] = 0;
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], pa.homing_feedrate[X_AXIS]/60, active_extruder);
		stepper_sync();

         stepper_wait_for_lmsw(X_AXIS);
         stepper_wait_for_lmsw(Y_AXIS);
         stepper_wait_for_lmsw(Z_AXIS);

		current_position[X_AXIS] = 0;
		current_position[Y_AXIS] = 0;
		current_position[Z_AXIS] = pa.z_home_pos;
		current_position[E_AXIS] = 0;
		//home_axis(X_AXIS);
		//home_axis(Y_AXIS);
		//home_axis(Z_AXIS);

		calculate_delta(current_position);

		printf("delta[X_AXIS]: %f\n", delta[X_AXIS]);
		printf("delta[Y_AXIS]: %f\n", delta[Y_AXIS]);
		printf("delta[Z_AXIS]: %f\n", delta[Z_AXIS]);

		delta[X_AXIS] = delta[X_AXIS] + pa.endstop_adj[0];
		delta[Y_AXIS] = delta[Y_AXIS] + pa.endstop_adj[1];
		delta[Z_AXIS] = delta[Z_AXIS] + pa.endstop_adj[2];
		printf("add endstop, delta:%f,%f,%f\n", delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS]);
		plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

		/* Set the current position for PRU */
		stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
				lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
				lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
				lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
	}

#if 0
        if (has_code(line, axis_codes[Z_AXIS])) {
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
				printf("lkj home zRaiseBeforeProbing: %f \n", pa.zRaiseBeforeProbing);
				destination[Z_AXIS] = current_position[Z_AXIS] + pa.zRaiseBeforeProbing;
				plan_buffer_line(current_position[X_AXIS], 
						current_position[Y_AXIS],
						destination[Z_AXIS],
						current_position[E_AXIS],
						feedrate / 60.0,
						active_extruder);
				stepper_sync();
				engage_z_probe();
			}
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
				dock_sled(false, 0);
			stepper_homing_axis(Z_AXIS);
        }

        if (has_code(line, axis_codes[X_AXIS])) {
  			stepper_wait_for_lmsw(X_AXIS);
            current_position[X_AXIS] = 0.0;
        }

        if (has_code(line, axis_codes[Y_AXIS])) {
            stepper_wait_for_lmsw(Y_AXIS);
            current_position[Y_AXIS] = 0.0;
        }

        if (has_code(line, axis_codes[Z_AXIS])) {
			if (pa.autoLeveling) {
				stepper_sync();
				stepper_wait_for_autoLevel();
            	current_position[Z_AXIS] = fabs(pa.endstopOffset[Z_AXIS]);  
				plan_set_position(current_position[X_AXIS],
								  current_position[Y_AXIS],
								  current_position[Z_AXIS],
								  current_position[E_AXIS]);
				stepper_set_position(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS],
						current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS],
						current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS],
						current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]);

				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
					printf("lkj home z 1 %f \n", home_retract_mm[Z_AXIS]);
					destination[Z_AXIS] = current_position[Z_AXIS] + home_retract_mm[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], 
									 current_position[Y_AXIS],
									 destination[Z_AXIS],
									 current_position[E_AXIS],
									 feedrate / 60.0,
									 active_extruder);
					stepper_sync();
				}
					
				current_position[Z_AXIS] = stepper_get_position_mm(Z_AXIS);
				plan_set_position(current_position[X_AXIS],
								  current_position[Y_AXIS],
								  current_position[Z_AXIS],
								  current_position[E_AXIS]);
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
					retract_z_probe();
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
					dock_sled(true, 0);
			} else {
            	stepper_wait_for_lmsw(Z_AXIS);
            	current_position[Z_AXIS] = 0.0;
			}
        }

        if (has_code(line, axis_codes[X_AXIS])) {
            stepper_homing_axis(X_AXIS);
        }

        if (has_code(line, axis_codes[Y_AXIS])) {
            stepper_homing_axis(Y_AXIS);
        }

        current_position[E_AXIS] = 0.0;

        plan_set_position(current_position[X_AXIS], 
                          current_position[Y_AXIS],
                          current_position[Z_AXIS],
                          current_position[E_AXIS]);

		stepper_set_position(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS],
							current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS],
							current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS],
				 			current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]);
#endif
}

#if 0
static void homing_axis(char *line) 
{

	if (pa.autoLeveling) {
		matrix_set_to_identity(&plan_bed_level_matrix);
	}

	if (pa.machine_type == MACHINE_COREXY) {
		        /* xyz & corexy */
        if (has_code(line, axis_codes[X_AXIS])) {
            stepper_homing_axis(X_AXIS);
        }

        if (has_code(line, axis_codes[Y_AXIS])) {
            stepper_homing_axis(Y_AXIS);
        }
        
        if (has_code(line, axis_codes[Z_AXIS])) {
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
				destination[Z_AXIS] = current_position[Z_AXIS] + pa.zRaiseBeforeProbing;
				plan_buffer_line(current_position[X_AXIS], 
						current_position[Y_AXIS],
						destination[Z_AXIS],
						current_position[E_AXIS],
						feedrate / 60.0,
						active_extruder);
				stepper_sync();
				engage_z_probe();
			}
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
				dock_sled(false, 0);
			stepper_homing_axis(Z_AXIS);
        }

        if (has_code(line, axis_codes[X_AXIS])) {
            stepper_wait_for_lmsw(X_AXIS);
            current_position[X_AXIS] = 0.0;
        }

        if (has_code(line, axis_codes[Y_AXIS])) {
            stepper_wait_for_lmsw(Y_AXIS);
            current_position[Y_AXIS] = 0.0;
        }

        if (has_code(line, axis_codes[Z_AXIS])) {
			if (pa.autoLeveling) {
				stepper_sync();
				stepper_wait_for_autoLevel();
				//current_position[Z_AXIS] = -pa.endstopOffset[Z_AXIS];  
				current_position[Z_AXIS] = fabs(pa.endstopOffset[Z_AXIS]);  
				plan_set_position(current_position[X_AXIS],
								  current_position[Y_AXIS],
								  current_position[Z_AXIS],
								  current_position[E_AXIS]);
				stepper_set_position(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS],
						current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS],
						current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS],
						current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]);

				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
					destination[Z_AXIS] = current_position[Z_AXIS] + home_retract_mm[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], 
									 current_position[Y_AXIS],
									 destination[Z_AXIS],
									 current_position[E_AXIS],
									 feedrate / 60.0,
									 active_extruder);
					stepper_sync();
				}
					
				current_position[Z_AXIS] = stepper_get_position_mm(Z_AXIS);
				plan_set_position(current_position[X_AXIS],
								  current_position[Y_AXIS],
								  current_position[Z_AXIS],
								  current_position[E_AXIS]);
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
					retract_z_probe();
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
					dock_sled(true, 0);
			} else {
            	stepper_wait_for_lmsw(Z_AXIS);
            	current_position[Z_AXIS] = 0.0;
			}
        }

        current_position[E_AXIS] = 0.0;

        plan_set_position(current_position[X_AXIS], 
                          current_position[Y_AXIS],
                          current_position[Z_AXIS],
                          current_position[E_AXIS]);

		stepper_set_position(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS],
							current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS],
							current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS],
				 			current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]);
	} else if (pa.machine_type == MACHINE_DELTA) {
        /* Delta */
        /* Move all carriages up together until all endstops are hit */
        current_position[X_AXIS] = 0.0;
        current_position[Y_AXIS] = 0.0;
        current_position[Z_AXIS] = 0.0;
        current_position[E_AXIS] = 0.0;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
                          current_position[Z_AXIS], current_position[E_AXIS]);

        destination[X_AXIS] = 0.0;
        destination[Y_AXIS] = 0.0;
        destination[Z_AXIS] = pa.z_home_pos;
	
        /* Take care of back off and rehome now we are all at the top */
        stepper_homing_axis(X_AXIS); //special suitiation, already set Y Z AXIS in stepper_pruss.c  
        //stepper_homing_axis(Y_AXIS);
        //stepper_homing_axis(Z_AXIS);

		if (pa.autoLeveling) {
        	destination[Z_AXIS] = pa.z_home_pos - fabs(pa.endstopOffset[Z_AXIS]);  
		}

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        current_position[Z_AXIS] = destination[Z_AXIS];

        calculate_delta(current_position);

        printf("delta[X_AXIS]: %f\n", delta[X_AXIS]);
        printf("delta[Y_AXIS]: %f\n", delta[Y_AXIS]);
        printf("delta[Z_AXIS]: %f\n", delta[Z_AXIS]);
        plan_set_position(delta[X_AXIS], 
                          delta[Y_AXIS], 
                          delta[Z_AXIS], 
                          current_position[E_AXIS]);

        /* Set the current position for PRU */
        stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
                             lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
                             lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
                             lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
    } else {
        /* xyz & corexy */
        if (has_code(line, axis_codes[X_AXIS])) {
            stepper_homing_axis(X_AXIS);
        }

        if (has_code(line, axis_codes[Y_AXIS])) {
            stepper_homing_axis(Y_AXIS);
        }
        
        if (has_code(line, axis_codes[Z_AXIS])) {
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
				destination[Z_AXIS] = current_position[Z_AXIS] + pa.zRaiseBeforeProbing;
				plan_buffer_line(current_position[X_AXIS], 
						current_position[Y_AXIS],
						destination[Z_AXIS],
						current_position[E_AXIS],
						feedrate / 60.0,
						active_extruder);
				stepper_sync();
				engage_z_probe();
			}
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
				dock_sled(false, 0);
			stepper_homing_axis(Z_AXIS);
        }

        if (has_code(line, axis_codes[X_AXIS])) {
  			stepper_wait_for_lmsw(X_AXIS);
            current_position[X_AXIS] = 0.0;
        }

        if (has_code(line, axis_codes[Y_AXIS])) {
            stepper_wait_for_lmsw(Y_AXIS);
            current_position[Y_AXIS] = 0.0;
        }

        if (has_code(line, axis_codes[Z_AXIS])) {
			if (pa.autoLeveling) {
				stepper_sync();
				stepper_wait_for_autoLevel();
            	current_position[Z_AXIS] = fabs(pa.endstopOffset[Z_AXIS]);  
				plan_set_position(current_position[X_AXIS],
								  current_position[Y_AXIS],
								  current_position[Z_AXIS],
								  current_position[E_AXIS]);
				stepper_set_position(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS],
						current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS],
						current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS],
						current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]);

				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
					destination[Z_AXIS] = current_position[Z_AXIS] + home_retract_mm[Z_AXIS];
					plan_buffer_line(current_position[X_AXIS], 
									 current_position[Y_AXIS],
									 destination[Z_AXIS],
									 current_position[E_AXIS],
									 feedrate / 60.0,
									 active_extruder);
					stepper_sync();
				}
					
				current_position[Z_AXIS] = stepper_get_position_mm(Z_AXIS);
				plan_set_position(current_position[X_AXIS],
								  current_position[Y_AXIS],
								  current_position[Z_AXIS],
								  current_position[E_AXIS]);
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
					retract_z_probe();
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
					dock_sled(true, 0);
			} else {
            	stepper_wait_for_lmsw(Z_AXIS);
            	current_position[Z_AXIS] = 0.0;
			}
        }

        current_position[E_AXIS] = 0.0;

        plan_set_position(current_position[X_AXIS], 
                          current_position[Y_AXIS],
                          current_position[Z_AXIS],
                          current_position[E_AXIS]);

		stepper_set_position(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS],
							current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS],
							current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS],
				 			current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]);
    }
}
#endif

static void engage_z_probe(void)
{
#ifdef SERVO
    channel_tag servo; 
    servo = servo_lookup_by_name("servo_0");
    if (servo) {
        servo_set_angle(servo, pa.servo_endstop_angle[0]);
        servo_enable(servo);
        /* disable the servo after engage the z probe */ 
        sleep(1); //FIXME
        servo_disable(servo);
    }
#else


#endif
}

static void retract_z_probe(void)
{
#ifdef SERVO
    channel_tag servo; 
    servo = servo_lookup_by_name("servo_0");
    if (servo) {
        servo_set_angle(servo, pa.servo_endstop_angle[1]);
        servo_enable(servo);
        /* disable the servo after engage the z probe */ 
        sleep(2); //FIXME
        servo_disable(servo);
    }
#else


#endif
}

#ifdef AUTO_LEVELING
static void run_z_probe(void)
{
    float position_z = -10.0;
    float oldFeedRate = feedrate;

    if (pa.machine_type == MACHINE_DELTA) {
        feedrate = pa.autolevel_down_rate;

      float start_z = current_position[Z_AXIS];
      long start_steps = stepper_get_position(Z_AXIS);
  
      destination[Z_AXIS] = -200;
      prepare_move_raw();
      stepper_sync();
      long stop_steps = stepper_get_position(Z_AXIS);
      float mm = start_z - (float)(start_steps - stop_steps) / pa.axis_steps_per_unit[Z_AXIS];
 	  //printf("run_z_probe resulrt: current z position:%fmm, start_z position:%f, start_steps:%ld, stop_steps:%ld \n", mm, start_z, start_steps, stop_steps); 
	  printf("lkj start_step:%ld, stop_steps:%ld \n", start_steps, stop_steps); 
 	  printf("lkj current_z :%fmm, new_current_z:%fmm \n",start_z, mm);
      current_position[Z_AXIS] = mm;
      calculate_delta(current_position);
      plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
	  stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
			  lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
			  lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
			  lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
	} else {
        feedrate = pa.autolevel_down_rate;

        /* move down until you find the bed */
        plan_buffer_line(current_position[X_AXIS], 
                         current_position[Y_AXIS],
                         position_z,
                         current_position[E_AXIS],
                         feedrate / 60.0,
                         active_extruder);
        stepper_sync();
        /* let the planner know where we are right now
         * as it is not where we said to go
         */
        position_z = stepper_get_position_mm(Z_AXIS);

        current_position[Z_AXIS] = position_z;
        plan_set_position(current_position[X_AXIS],
                          current_position[Y_AXIS],
                          current_position[Z_AXIS],
                          current_position[E_AXIS]);
		stepper_set_position(lround(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
				lround(current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
				lround(current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
				lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));

		/* move up the retract distance */
        position_z += pa.zRaiseBetweenProbing; //home_retract_mm[Z_AXIS];  
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],
                         position_z,
                         current_position[E_AXIS],
                         feedrate / 60.0,
                         active_extruder);
        stepper_sync();

//		if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY) {
			feedrate = pa.autolevel_down_rate/4.5f;
			position_z -= 2 *pa.zRaiseBetweenProbing; // home_retract_mm[Z_AXIS];  
			plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],
					position_z,
					current_position[E_AXIS],
					feedrate / 60.0,
					active_extruder);
			stepper_sync();
//		}

        current_position[Z_AXIS] = stepper_get_position_mm(Z_AXIS);
		//printf("debug autolevel, run_z_probe :%f\n", current_position[Z_AXIS]);
        plan_set_position(current_position[X_AXIS],
                          current_position[Y_AXIS],
                          current_position[Z_AXIS],
                          current_position[E_AXIS]);
		stepper_set_position(lround(current_position[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
				lround(current_position[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
				lround(current_position[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
				lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));

		if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {  //for safe retract distance
        	current_position[Z_AXIS] += safe_servo_retract_distance; 
			plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],
					current_position[Z_AXIS],
					current_position[E_AXIS],
					feedrate / 60.0,
					active_extruder);
			stepper_sync();
		}

#if 0
        /* move back down slowly to find bed */
        //feedrate = pa.homing_feedrate[Z_AXIS] / 4;
        position_z -= home_retract_mm[Z_AXIS] * 2;  //FIXME
        plan_buffer_line(current_position[X_AXIS], 
                         current_position[Y_AXIS],
                         position_z,
                         current_position[E_AXIS],
                         feedrate / 60,
                         active_extruder);
        stepper_sync();

        /* make sure the planner knows where we are as it
         * may be a bit different than we last said to move to 
         *
         */
        current_position[Z_AXIS] = stepper_get_position_mm(Z_AXIS);
        plan_set_position(current_position[X_AXIS],
                          current_position[Y_AXIS],
                          current_position[Z_AXIS],
                          current_position[E_AXIS]);
#endif
    }

    feedrate = oldFeedRate;
}

static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    feedrate = 8000; //XY_TRAVEL_SPEED;
	//printf("do_blocking_move_to x:%f, y:%f, z:%f \n", x, y, z);
	//printf("do_blocking_move_to current x:%f, y:%f, z:%f \n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

	if (pa.machine_type == MACHINE_DELTA) {
		destination[X_AXIS] = x; 
		destination[Y_AXIS] = y; 
		destination[Z_AXIS] = z; 
		prepare_move_raw();
	} else { // cartesian
		current_position[X_AXIS] = x; 
		current_position[Y_AXIS] = y; 
		current_position[Z_AXIS] = z; 
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
	}
    stepper_sync();

    feedrate = oldFeedRate;
}

/*
 * Probe bed height at position (x, y), return the measured z value
 */
static float probe_bed_height(float x, float y, float z_before)
{
    float measured_z;

	//printf("probe bed height current x:%f, y:%f, z:%f \n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
	//printf("probe bed height start x:%f, y:%f, z_before:%f \n", x, y, z_before);

	do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
	do_blocking_move_to(x + pa.endstopOffset[X_AXIS], y + pa.endstopOffset[Y_AXIS], current_position[Z_AXIS]);
    
    /* Engage z probe */
	if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
    	engage_z_probe(); 

    run_z_probe();

    measured_z = current_position[Z_AXIS];
    //printf("debug autolevel, after run_z_prboe, z pos:%f\n", stepper_get_position_mm(Z_AXIS));
    
    /* Retract z probe */
	if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
    	retract_z_probe();
    
    /* respond current position to host */

    return measured_z;
}

static void single_z_probe(void) 
{
    float old_feedrate = feedrate;
    int old_feedmultiply = feedmultiply;
    feedmultiply = 100;

    /* Engage z probe */
	if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
    	engage_z_probe(); 
    
    stepper_sync();

    feedrate = pa.autolevel_down_rate;

    run_z_probe();

    /* Retract z probe */
	if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
     	retract_z_probe();

    feedrate = old_feedrate;
    feedmultiply = old_feedmultiply;
}


#ifdef AUTO_LEVELING
void plan_get_position(vector_t *v, matrix_t level_matrix)
{
    matrix_t transpos_matrix;
    v->x = stepper_get_position_mm(X_AXIS);
    v->y = stepper_get_position_mm(Y_AXIS); 
    v->z = stepper_get_position_mm(Z_AXIS);
    
    matrix_transpos(level_matrix, &transpos_matrix); 
    vector_apply_rotation(v, transpos_matrix);
}
#endif
static void set_delta_constants()
{
#if 0
  max_length[Z_AXIS] = pa.z_home_pos - Z_MIN_POS;
  base_max_pos[Z_AXIS]  = pa.z_home_pos;
  base_home_pos[Z_AXIS] = pa.z_home_pos;
#endif
  
  
  // Effective X/Y positions of the three vertical towers.
  /*
  delta_tower1_x = (-SIN_60 * delta_radius) + tower_adj[0]; // front left tower + xa
  delta_tower1_y = (-COS_60 * delta_radius) - tower_adj[0] ;
  delta_tower2_x = -(-SIN_60 * delta_radius) + tower_adj[1]; // front right tower + xb
  delta_tower2_y = (-COS_60 * delta_radius) + tower_adj[1]; // 
  delta_tower3_x = tower_adj[2] ; // back middle tower + xc
  delta_tower3_y = -2 * (-COS_60 * delta_radius);  
  */

  delta_tower1_x = -SIN_60 * (pa.radius + pa.radius_adj[0]); // front left tower + xa
  delta_tower1_y = -COS_60 * (pa.radius + pa.radius_adj[0]) ;
  delta_tower2_x = SIN_60  * (pa.radius + pa.radius_adj[1]); // front right tower + xb
  delta_tower2_y = -COS_60 * (pa.radius + pa.radius_adj[1]); // 
  delta_tower3_x = 0.0; 
  delta_tower3_y = pa.radius + pa.radius_adj[2];  
  
  delta_diagonal_rod_2_tower1 = pow(pa.diagonal_rod + pa.diagonal_rod_adj[0], 2);
  delta_diagonal_rod_2_tower2 = pow(pa.diagonal_rod + pa.diagonal_rod_adj[1], 2);
  delta_diagonal_rod_2_tower3 = pow(pa.diagonal_rod + pa.diagonal_rod_adj[2], 2);
#if 0
  delta_tower1_x = (pa.radius + pa.tower_adj[3]) * cos((210 + pa.tower_adj[0]) * M_PI/180); // front left tower
  delta_tower1_y = (pa.radius + pa.tower_adj[3]) * sin((210 + pa.tower_adj[0]) * M_PI/180); 
  delta_tower2_x = (pa.radius + pa.tower_adj[4]) * cos((330 + pa.tower_adj[1]) * M_PI/180); // front right tower
  delta_tower2_y = (pa.radius + pa.tower_adj[4]) * sin((330 + pa.tower_adj[1]) * M_PI/180); 
  delta_tower3_x = (pa.radius + pa.tower_adj[5]) * cos((90 + pa.tower_adj[2]) * M_PI/180);  // back middle tower
  delta_tower3_y = (pa.radius + pa.tower_adj[5]) * sin((90 + pa.tower_adj[2]) * M_PI/180); 
#endif
}


#ifdef AUTO_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_t plane_normal;
    vector_t corrected_position;

    plane_normal.x = -plane_equation_coefficients[0];
    plane_normal.y = -plane_equation_coefficients[1];
    plane_normal.z = 1;

    plan_bed_level_matrix = matrix_create_look_at(plane_normal);
    /* if (!plan_bed_level_matrix) {
        printf("[gcode]: matrix create look at err\n");
    } */
    plan_get_position(&corrected_position, plan_bed_level_matrix);

    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;


	plan_set_position_no_delta_autolevel(current_position[X_AXIS],
			current_position[Y_AXIS],
			current_position[Z_AXIS],
			current_position[E_AXIS]);
}
#else
static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3)
{
    vector_t pt1, pt2, pt3;
    vector_t from_2_to_1;
    vector_t from_2_to_3;
    matrix_set_to_identity(&plan_bed_level_matrix);
    
    pt1.x = pa.probePoint1[X_AXIS];//ABL_PROBE_PT_1_X;
    pt1.y = pa.probePoint1[Y_AXIS];//ABL_PROBE_PT_1_Y;
    pt1.z = z_at_pt_1;

    pt2.x = pa.probePoint2[X_AXIS];//ABL_PROBE_PT_2_X;
    pt2.y = pa.probePoint2[Y_AXIS];//ABL_PROBE_PT_2_Y;
    pt2.z = z_at_pt_2;

    pt3.x = pa.probePoint3[X_AXIS];//ABL_PROBE_PT_3_X;
    pt3.y = pa.probePoint3[Y_AXIS];//ABL_PROBE_PT_3_Y;
    pt3.z = z_at_pt_3;

    from_2_to_1.x = pt1.x - pt2.x; 
    from_2_to_1.y = pt1.y - pt2.y; 
    from_2_to_1.z = pt1.z - pt2.z; 

    from_2_to_3.x = pt3.x - pt2.x; 
    from_2_to_3.y = pt3.y - pt2.y; 
    from_2_to_3.z = pt3.z - pt2.z; 

    vector_normalize(&from_2_to_1);
    vector_normalize(&from_2_to_3);

    vector_t plane_normal;
    vector_cross(from_2_to_1, from_2_to_3, &plane_normal);
    vector_normalize(&plane_normal);
    plane_normal.z = abs(plane_normal.z);
    
    plan_bed_level_matrix = matrix_create_look_at(plane_normal);
     
	vector_t corrected_position;
    plan_get_position(&corrected_position, plan_bed_level_matrix);

    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;
    
    plan_set_position(current_position[X_AXIS],
                      current_position[Y_AXIS],
                      current_position[Z_AXIS],
                      current_position[E_AXIS]);

}
#endif

static void delta_min_z_pin_dock(bool dock)
{
	float saved_feedrate = feedrate;
	if (dock) {
		feedrate = pa.homing_feedrate[X_AXIS];
		//destination[Z_AXIS] = 50;
		//prepare_move_raw();

		destination[X_AXIS] = pa.delta_retract_start_location[X_AXIS];
		destination[Y_AXIS] = pa.delta_retract_start_location[Y_AXIS];
		destination[Z_AXIS] = pa.delta_retract_start_location[Z_AXIS];
		prepare_move_raw();
		stepper_sync();

		// Move the nozzle below the print surface to push the probe up.
		feedrate = pa.homing_feedrate[Z_AXIS]/10;
		destination[X_AXIS] = pa.delta_retract_end_location[X_AXIS];
		destination[Y_AXIS] = pa.delta_retract_end_location[Y_AXIS];
		destination[Z_AXIS] = pa.delta_retract_end_location[Z_AXIS];
		prepare_move_raw();
		stepper_sync();

		feedrate = pa.homing_feedrate[Z_AXIS];
		destination[X_AXIS] = pa.delta_retract_start_location[X_AXIS];
		destination[Y_AXIS] = pa.delta_retract_start_location[Y_AXIS];
		destination[Z_AXIS] = pa.delta_retract_start_location[Z_AXIS];
		prepare_move_raw();

		stepper_sync();
	} else {
		feedrate =pa.homing_feedrate[X_AXIS];
		destination[X_AXIS] = pa.delta_deploy_start_location[X_AXIS];
		destination[Y_AXIS] = pa.delta_deploy_start_location[Y_AXIS];
		destination[Z_AXIS] = pa.delta_deploy_start_location[Z_AXIS];
		prepare_move_raw();
		stepper_sync();

		feedrate = pa.homing_feedrate[X_AXIS]/10;
		destination[X_AXIS] = pa.delta_deploy_end_location[X_AXIS];
		destination[Y_AXIS] = pa.delta_deploy_end_location[Y_AXIS];
		destination[Z_AXIS] = pa.delta_deploy_end_location[Z_AXIS];
		prepare_move_raw();
		stepper_sync();

		feedrate = pa.homing_feedrate[X_AXIS];
		destination[X_AXIS] = pa.delta_deploy_start_location[X_AXIS];
		destination[Y_AXIS] = pa.delta_deploy_start_location[Y_AXIS];
		destination[Z_AXIS] = pa.delta_deploy_start_location[Z_AXIS];
		prepare_move_raw();

		stepper_sync();
	}

	feedrate = saved_feedrate;
}

static void dock_sled(bool dock, int offset)
{
 //int z_loc;
  if (dock) {
   // turn off magnet
	stepper_autoLevel_gpio_turn(false);
 } else {
   // turn on magnet
	stepper_autoLevel_gpio_turn(true);
 }

#if 0
 if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
   LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
   SERIAL_ECHO_START;
   SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
   return;
 }

 if (dock) {
   do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset,
                       current_position[Y_AXIS],
                       current_position[Z_AXIS]);
   // turn off magnet
   digitalWrite(SERVO0_PIN, LOW);
 } else {
   if (current_position[Z_AXIS] < (Z_RAISE_BEFORE_PROBING + 5))
     z_loc = Z_RAISE_BEFORE_PROBING;
   else 
     z_loc = current_position[Z_AXIS];
   do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset,
                       Y_PROBE_OFFSET_FROM_EXTRUDER, z_loc);
   // turn on magnet
   digitalWrite(SERVO0_PIN, HIGH);
 }
#endif
}


static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
  if (bed_level[x][y] != 0.0) {
    return;  // Don't overwrite good values.
  }
  float a = 2*bed_level[x+xdir][y] - bed_level[x+xdir*2][y];  // Left to right.
  float b = 2*bed_level[x][y+ydir] - bed_level[x][y+ydir*2];  // Front to back.
  float c = 2*bed_level[x+xdir][y+ydir] - bed_level[x+xdir*2][y+ydir*2];  // Diagonal.
  float median = c;  // Median is robust (ignores outliers).
  if (a < b) { 
    if (b < c) median = b; 
    if (c < a) median = a; 
  } else {  // b <= a
    if (c < b) median = b; 
    if (a < c) median = a; 
  }
  bed_level[x][y] = median;
}

// Fill in the unprobed points (corners of circular print surface)
// using linear extrapolation, away from the center.
static void extrapolate_unprobed_bed_level() {
  int x, y;
  int half = (pa.probeGridPoints-1)/2;
  for (y = 0; y <= half; y++) {
    for (x = 0; x <= half; x++) {
      if (x + y < 3) continue;
      extrapolate_one_point(half-x, half-y, x>1?+1:0, y>1?+1:0);
      extrapolate_one_point(half+x, half-y, x>1?-1:0, y>1?+1:0);
      extrapolate_one_point(half-x, half+y, x>1?+1:0, y>1?-1:0);
      extrapolate_one_point(half+x, half+y, x>1?-1:0, y>1?-1:0);
    }
  }
}

static void print_bed_level(float (*level)[100]) {
 int x, y;

  char buf[750] = {0};

  memset(buf, 0, sizeof(buf));
  for (y = 0; y < pa.probeGridPoints; y++) {
    for (x = 0; x < pa.probeGridPoints; x++) {
      printf("%f ", level[x][y]);
	  sprintf(buf + strlen(buf), "%f ", level[x][y]);
    }
	sprintf(buf + strlen(buf), " \n");
  	gcode_send_response_remote((char *)&buf);
    memset(buf, 0, sizeof(buf));
    printf("\n");
  }

}

// Reset calibration results to zero.
static void reset_bed_level()  //for delta
{ 
 int x, y;
  for (y = 0; y < pa.probeGridPoints; y++) {
    for (x = 0; x < pa.probeGridPoints; x++) {
      bed_level[x][y] = 0.0;
    }
  }
}

// Adjust print surface height by linear interpolation over the bed_level array.
static void adjust_delta(float cartesian[3])
{
  int half = (pa.probeGridPoints- 1) / 2;
  float x_num = (pa.probeRightPos - pa.probeLeftPos) / (pa.probeGridPoints- 1);
  float y_num = (pa.probeBackPos - pa.probeFrontPos) / (pa.probeGridPoints- 1);
  float grid_x = max(0.001-half, min(half-0.001, cartesian[X_AXIS] / x_num));
  float grid_y = max(0.001-half, min(half-0.001, cartesian[Y_AXIS] / y_num));
  int floor_x = floor(grid_x);
  int floor_y = floor(grid_y);
  float ratio_x = grid_x - floor_x;
  float ratio_y = grid_y - floor_y;
  float z1 = bed_level[floor_x+half][floor_y+half];
  float z2 = bed_level[floor_x+half][floor_y+half+1];
  float z3 = bed_level[floor_x+half+1][floor_y+half];
  float z4 = bed_level[floor_x+half+1][floor_y+half+1];
  float left = (1-ratio_y)*z1 + ratio_y*z2;
  float right = (1-ratio_y)*z3 + ratio_y*z4;
  float offset = (1-ratio_x)*left + ratio_x*right;

  //printf("grid_x:%f, grid_y:%f, half-0.01:%f\n", grid_x, grid_y, half-0.01);
  //printf("z1:%f, z2:%f, z3:%f,z4:%f\n", z1, z2, z3, z4);
  //printf("cartesian x:%f, y:%f, z:%f\n", cartesian[X_AXIS], cartesian[Y_AXIS], cartesian[Z_AXIS]);
  //printf("delta  x:%f, y:%f, z:%f, offset:%f\n", delta[X_AXIS], delta[Y_AXIS],delta[Z_AXIS], offset);
  delta[X_AXIS] += offset;
  delta[Y_AXIS] += offset;
  delta[Z_AXIS] += offset;
}

static void auto_bed_leveling(bool mult_point)
{
    COMM_DBG("Auto bed leveling\n");
    
    /* Prevent user from running G29 without first homing in X and Y 
     * Check limit swith state, if not hit, do nothing!
     */
    if (!stepper_check_lmsw(X_AXIS) 
        || !stepper_check_lmsw(Y_AXIS)) {
        printf("[gcode]: Abort G29, Please homing X and Y first\n");
        return;
    }

	if (pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
		dock_sled(false, 0);
	if (pa.probeDeviceType == PROBE_DEVICE_Z_PIN)
		delta_min_z_pin_dock(false);

    stepper_sync();
    
    /* Make sure the bed level rotation matrix is identity */
    matrix_set_to_identity(&plan_bed_level_matrix);
	reset_bed_level();

	if (pa.machine_type != MACHINE_DELTA) {
		vector_t uncorrected_position; 
		plan_get_position(&uncorrected_position, plan_bed_level_matrix);

		current_position[X_AXIS] = uncorrected_position.x;
		current_position[Y_AXIS] = uncorrected_position.y;
		current_position[Z_AXIS] = uncorrected_position.z;

		plan_set_position(current_position[X_AXIS],
				current_position[Y_AXIS],
				current_position[Z_AXIS],
				current_position[E_AXIS]);
	}
    //printf("debug autolevel, before G29 z pos:%f\n", stepper_get_position_mm(Z_AXIS));

    feedrate = pa.homing_feedrate[Z_AXIS];
    //printf("lkj before G29. x:%f, y:%f, z:%f\n", current_position[X_AXIS],
     //                 current_position[Y_AXIS],
       //               current_position[Z_AXIS]);
debug_matrix(&plan_bed_level_matrix);

#ifdef AUTO_LEVELING_GRID
	if (pa.machine_type == MACHINE_DELTA && pa.autoLeveling) {
#if 1

		 bool zig = (pa.probeGridPoints & 1) ? true : false;
  		float x_num = (pa.probeRightPos - pa.probeLeftPos) / (pa.probeGridPoints- 1);
  		float y_num = (pa.probeBackPos - pa.probeFrontPos) / (pa.probeGridPoints- 1);
         int probePointCounter = 0;
		 int yCount = 0, xCount = 0; 
  		//float x_stay = 0;
  		//float y_stay = 0;
         for (yCount=0; yCount < pa.probeGridPoints; yCount++) {
           float yProbe = pa.probeFrontPos + y_num * yCount;
           int xStart, xStop, xInc;
           //if (yCount % 2) {
           if (zig) {
             xStart = 0;
             xStop = pa.probeGridPoints;
             xInc = 1;
           } else {
             xStart = pa.probeGridPoints - 1;
             xStop = -1;
             xInc = -1;
           }

			zig = !zig;
           for (xCount=xStart; xCount != xStop; xCount += xInc) {
             float xProbe = pa.probeLeftPos + x_num * xCount;

             // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
             //float distance_from_center = sqrt(xProbe*xProbe + yProbe*yProbe);
             float distance_from_center = sqrt((xProbe + pa.endstopOffset[X_AXIS]) * (xProbe + pa.endstopOffset[X_AXIS])  + 
										 (yProbe + pa.endstopOffset[Y_AXIS]) * (yProbe + pa.endstopOffset[Y_AXIS]));
            float  device_distance_from_center = sqrt(xProbe * xProbe + yProbe * yProbe);
             if ((distance_from_center >  pa.delta_print_radius) || (device_distance_from_center > pa.delta_print_radius)) {
						printf("skip porint, x:%f, y:%f\n", xProbe, yProbe);
						int x=xCount, y=yCount;
#if 1
						if ((x-1)>0 && (x-2)>0 && bed_level[x-1][y] != 0 && bed_level[x-2][y] != 0) {
							float p1 = bed_level[x-1][y];
							float p2 = bed_level[x-2][y];
							printf("up point  p1:%f, p2:%f\n", p1, p2);
							float s = p2/p1;
							bed_level[x][y] = p1 / s;
						} else if ((y-1)>0 && (y-2)>0 && bed_level[x][y-1] != 0 && bed_level[x][y-2] != 0) {
							float p1 = bed_level[x][y-1];
							float p2 = bed_level[x][y-2];
							printf("left point p1:%f, p2:%f\n", p1, p2);
							float s = p2/p1;
							bed_level[x][y] = p1 / s;
						} else if (((x+1)<pa.probeGridPoints) && ((x+2)<pa.probeGridPoints) && bed_level[x+1][y] != 0 && bed_level[x+2][y] != 0){
							float p1 = bed_level[x+1][y];
							float p2 = bed_level[x+2][y];
							printf("down point  p1:%f, p2:%f\n", p1, p2);
							float s = p2/p1;
							bed_level[x][y] = p1 / s;
						} else if ((y+1)<pa.probeGridPoints && (y+2)<pa.probeGridPoints && bed_level[x][y+1] != 0 && bed_level[x][y+2] != 0){
							float p1 = bed_level[x][y+1];
							float p2 = bed_level[x][y+2];
							printf("right point  p1:%f, p2:%f\n", p1, p2);
							float s = p2/p1;
							bed_level[x][y] = p1 / s;
						} else {
							printf("no near point\n");
						}
							#endif
						continue;
			 }

			 float z_before = probePointCounter == 0 ? pa.zRaiseBeforeProbing : current_position[Z_AXIS] + pa.zRaiseBetweenProbing;
			 float measured_z1 = probe_bed_height(xProbe, yProbe, z_before);
			 if (probePointCounter == 0) {
				 z_before = current_position[Z_AXIS] + pa.zRaiseBetweenProbing;
			 } 	

			float measured_z2 = measured_z1;
			float measured_z3 = measured_z1;
			if (mult_point) {
				measured_z2 = probe_bed_height(xProbe, yProbe, z_before);
				measured_z3 = probe_bed_height(xProbe, yProbe, z_before);
			}

			float measured_z = (measured_z1 + measured_z2 + measured_z3) /3.0;
			printf("measured_z1-3:%f %f %f\n", measured_z1, measured_z2, measured_z3);

			 printf("[%d][%d]=xProbe:%f,yProbe:%f,measured_z+offset:%f\n", xCount, yCount, xProbe, yProbe, measured_z + pa.endstopOffset[Z_AXIS]);
             bed_level[xCount][yCount] = measured_z + pa.endstopOffset[Z_AXIS];

             probePointCounter++;
           }
         }
		extrapolate_unprobed_bed_level(); //for delta
		print_bed_level(bed_level);
    	stepper_sync();

		if (pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
			dock_sled(true, 0);
		if (pa.probeDeviceType == PROBE_DEVICE_Z_PIN)
			delta_min_z_pin_dock(true);
		return;
#endif
	}


    /* Grid mode Auto bed leveling : probe at the points of a lattice grid */
    //int x_grid_spacing = (pa.probeRightPos - pa.probeLeftPos) / (pa.probeGridPoints -1);
					//	(RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION)
                     //    / (AUTO_LEVELING_GRID_POINTS - 1);
                                       

    //int y_grid_spacing = (pa.probeBackPos - pa.probeFrontPos) / (pa.probeGridPoints - 1);
    					// (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) 
                        // / (AUTO_LEVELING_GRID_POINTS - 1);
	//printf("x spacing:%d, y spacing:%d\n", x_grid_spacing, y_grid_spacing);
    /* 
     * Solve the plane equation ax + by + d = z
     * A is the matrix with rows [x, y, 1] for all the probed points
     * B is the vector of the Z positions
     * The normal vector to the plane is formed by the coefficient of the plane
     * equation in the standard form, which is Vx*x + Vy*y + Vz*z + d = 0;
     * So Vx = -a, Vy = -b, Vz = 1, we want the vector facing towards positive Z.
     */
    /* "A" matrix of the linear system of equations */
    double eqn_a_matrix[pa.probeGridPoints* pa.probeGridPoints* 3];
    /* "B" vector of Z points */
    double eqn_b_matrix[pa.probeGridPoints* pa.probeGridPoints];
	bool zig = (pa.probeGridPoints & 1) ? true : false;
  	float x_num = (pa.probeRightPos - pa.probeLeftPos) / (pa.probeGridPoints- 1);
  	float y_num = (pa.probeBackPos - pa.probeFrontPos) / (pa.probeGridPoints- 1);
    int probePointCounter = 0;
    int yCount = 0, xCount = 0; 
	for (yCount=0; yCount < pa.probeGridPoints; yCount++) {
		float yProbe = pa.probeFrontPos + y_num * yCount;
		int xStart, xStop, xInc;
		if (zig) {
			xStart = 0;
			xStop = pa.probeGridPoints;
			xInc = 1;
		} else {
			xStart = pa.probeGridPoints - 1;
			xStop = -1;
			xInc = -1;
		}

		zig = !zig;
		for (xCount=xStart; xCount != xStop; xCount += xInc) {
			float xProbe = pa.probeLeftPos + x_num * xCount;
			float z_before; 
            if (probePointCounter == 0) {
					z_before = pa.zRaiseBeforeProbing;
			} else {
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO) {
					z_before = current_position[Z_AXIS] + pa.zRaiseBetweenProbing; //zRaiseBeforeProbing;
				} else {
					z_before = current_position[Z_AXIS] + pa.zRaiseBetweenProbing; //home_retract_mm[Z_AXIS];
				}
            }

			float measured_z1 = probe_bed_height(xProbe, yProbe, z_before);

			float measured_z2 = measured_z1;
			float measured_z3 = measured_z1;
			if (mult_point) {
				measured_z2 = probe_bed_height(xProbe, yProbe, z_before);
				measured_z3 = probe_bed_height(xProbe, yProbe, z_before);
			}
			float measured_z = (measured_z1 + measured_z2 + measured_z3) /3.0;
			
			printf("measured_z1-3:%f %f %f\n", measured_z1, measured_z2, measured_z3);
			printf("xProbe:%f,yProbe:%f,measured_z:%f\n", xProbe, yProbe, measured_z);
			eqn_b_matrix[probePointCounter] = measured_z;
			eqn_a_matrix[probePointCounter+ 0 * pa.probeGridPoints * pa.probeGridPoints] = xProbe;
			eqn_a_matrix[probePointCounter+ 1 * pa.probeGridPoints * pa.probeGridPoints] = yProbe;
			eqn_a_matrix[probePointCounter+ 2 * pa.probeGridPoints * pa.probeGridPoints] = 1;
			probePointCounter++;
		}
	}

	/* Solve lsq problem */
	double *plane_equation_coefficients = qr_solve(pa.probeGridPoints * pa.probeGridPoints, 3, 	eqn_a_matrix, eqn_b_matrix); 
	set_bed_level_equation_lsq(plane_equation_coefficients);

	free(plane_equation_coefficients);
#else
    /* "3-point" mode Auto bed leveling : probe 3 arbitrary points */
    /* probe point 1 */
    float z_at_pt_1 = probe_bed_height(pa.probePoint1[X_AXIS], //ABL_PROBE_PT_1_X, 
                                       pa.probePoint1[Y_AXIS], //ABL_PROBE_PT_1_Y,
                                       pa.zRaiseBeforeProbing);

    /* probe point 2 */
    float z_at_pt_2 = probe_bed_height(pa.probePoint2[X_AXIS], //ABL_PROBE_PT_2_X, 
                                       pa.probePoint2[Y_AXIS], //ABL_PROBE_PT_2_Y,
                                       current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBING);

    /* probe point 3 */
    float z_at_pt_3 = probe_bed_height(pa.probePoint3[X_AXIS], //ABL_PROBE_PT_3_X, 
                                       pa.probePoint3[Y_AXIS], //ABL_PROBE_PT_3_Y,
                                       current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBING);

    set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
#endif
    
    stepper_sync();

	if (pa.machine_type != MACHINE_DELTA) {
		/* 
		 * Correct the z height difference from z-probe position and hotend tip 
		 * The Z height on homing is measured by Z-Probe, but the probe is quite far from
		 * the hotend. When the bed is uneven, this height must be corrected.
		 */
		/* Get the real z, since the auto leveling is already correcting the plane */
		float real_z = (stepper_get_position(Z_AXIS)) / pa.axis_steps_per_unit[Z_AXIS]; 
		float x = current_position[X_AXIS] - pa.endstopOffset[X_AXIS];// X_PROBE_OFFSET_FROM_EXTRUDER;
		float y = current_position[Y_AXIS] - pa.endstopOffset[Y_AXIS];// Y_PROBE_OFFSET_FROM_EXTRUDER;
		//float x = current_position[X_AXIS] + pa.endstopOffset[X_AXIS];// X_PROBE_OFFSET_FROM_EXTRUDER;
		//float y = current_position[Y_AXIS] + pa.endstopOffset[Y_AXIS];// Y_PROBE_OFFSET_FROM_EXTRUDER;
		float z = current_position[Z_AXIS];
		//printf("lkj after 1 G29. x:%f, y:%f, z:%f, real_z:%f, z_steps:%d\n", current_position[X_AXIS],
		//                    current_position[Y_AXIS],
		//                  current_position[Z_AXIS], real_z, (stepper_get_position(Z_AXIS)));

		//printf("lkj after 3 G29. z:%f, real_z:%f, current_position[Z_AXIS]:%f\n", z, real_z, current_position[Z_AXIS]);
		/* Apply the correction sending the probe offset */
		apply_rotation_xyz(plan_bed_level_matrix, &x, &y, &z);  

		//printf("lkj after 4 G29. z:%f, real_z:%f, current_position[Z_AXIS]:%f\n", z, real_z, current_position[Z_AXIS]);

		/* The difference is added to current position and send to planner */
		current_position[Z_AXIS] = z - real_z + fabs(pa.endstopOffset[Z_AXIS]);
printf("lkj G29 new x:%f, y:%f, z:%f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
		plan_set_position_no_delta_autolevel(current_position[X_AXIS],
				current_position[Y_AXIS],
				current_position[Z_AXIS],
				current_position[E_AXIS]);
		debug_matrix(&plan_bed_level_matrix);
	}

	if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
		dock_sled(true, 0);
	if (pa.probeDeviceType == PROBE_DEVICE_Z_PIN)
		delta_min_z_pin_dock(true);
}
#endif
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
 * 
 * G29  - Detailed Z-Probe, probes the bed at 3 or more points. : TODO
 *        Will failed if you haven't homed yet.
 * G30  - Single Z Probe, probes bed at current XY location. : TODO
 *
 * G90  - Use Absolute Coordinates
 * G91  - Use Relative Coordinates
 * G92  - Set current position to coordinates given
 * 
 * RepRap M Codes
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors power
 *
 * M104 - Set extruder target temp
 * M105 - Read current temp
 *
 * M106 - Fan 1 on
 * M107 - Fan 1 off
 *
 * M109 - Wait for extruder current temp to reach target temp.
 *
 * M110 - Set Current Line Number  TODO
 *
 * M111 - Set Debug level.
 *
 * M114 - Display current position in mm
 * M115 - Capabilities string
 * M119 - Show Endstop State : Min [X/Y/Z] [hited/open-chain]
 * M140 - Set bed target temp
 * M150 - Set Led light output, R<0~255>, G<0~255>, B<0~255>
 *                              Fan_4,    Fan_5,    Fan_6
 * M190 - Wait for bed current temp to reach target temp.
 * 
 * Custom M Codes
 * SD Card support:
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
 * M30  - Delete file on sd card
 *
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 *
 * M84  - Disable steppers until next move, 
 *        M84 -> blocking quit
 *        M84  S0 -> quit immediately
 *
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. 
 *        To disable set zero (default) : TODO
 *
 * M92  - Set axis_steps_per_unit - same syntax as G92
 * M93  - Send axis_steps_per_unit
 *
 *
 * M176 - Fan 3 on
 * M177 - Fan 3 off
 *
 * M200 - Set filament diameter ans set E axis units to cubic millimeters. 
 *        use S0 to set back to millimeters. : TODO
 *
 * M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000 Z100 E2000)
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 * M204 - Set default acceleration: S normal moves T filament only moves 
 *        (M204 S3000 T7000) in mm/sec^2
 * M205 - advanced settings:	minimum travel speed S=while printing T=travel only,  
 *        X=maximum xy jerk, Z=maximum Z jerk
 * M206 - set additional homing offset
 * 
 * M207 - Set retract length S[positive mm] F[feedrate mm/s] Z[addition zlift/hop],
 *        stays in mm regardless of M200 setting.
 * M208 - Set recover/unretract length S[positive mm] F[feedrate mm/s].
 * M209 - Enable automatic retract detect if the slicer did not support G10/G11, TODO
 *      - every normal extrude-only move will be classified as retract depending on the direction.
 *
 * M210 - Set Homing Feedrate mm/min Xnnn Ynnn Znnn
 *
 * M218 - Set hotend offset in mm: T<extruder num> X<offset_on_x> Y<offset_on_y> TODO
 *
 * M220 - set speed factor override percentage S=factor in percent 
 * M221 - set extruder multiply factor S100 --> original Extrude Speed   TODO
 *
 * M280 - Set servo position absolute. P: servo index, S: angle or microseconds.  TODO
 *
 * Note: M301, M303, M304 applies to currently selected extruder.	Use T0 or T1 to select.
 * M301 - Set Heater parameters P, I, D, S (slope), B (y-intercept), W (maximum pwm)
 * M303 - PID relay autotune S<temperature> sets the target temperature. 
 *        (default target temperature = 150C)  TODO
 * M304 - Set Heater parameters P, I, D, S (slope), B (y-intercept), W (maximum pwm)
 * 
 * M350 - Set microstepping steps (M350 X16 Y16 Z16 E16 B16) TODO
 * M355 - Trun case fan on/off, S1, enable fan_3, S0, disable fan_3.
 * M400 - Finish all moves
 * M401 - Lower Z-probe TODO
 * M402 - Raise Z-probe TODO
 * M404 - Filament width TODO
 * M405 - Filament sensor on TODO
 * M406 - Filament sensor off TODO
 * M407 - Display filament diameter TODO
 *
 * M500 - stores paramters in EEPROM
 * M501 - reads parameters from EEPROM 
 *        (if you need to reset them after you changed them temporarily).
 * M502 - reverts to the default "factory settings". 
 *        You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print settings currently in memory : TODO
 * M504 - upgrade pru.bin firmware, define by fastbot
 * M505 - Save Parameters to SD-Card TODO
 *
 * M510 - Invert axis, 0=false, 1=true (M510 X0 Y0 Z0 E1)
 * M520 - Set maximum print area (M520 X200 Y200 Z150)
 *
 * M521 - Disable axis when unused (M520 X0 Y0 Z1 E0)
 * M522 - Use software endstops I=min, A=max, 0=false, 1=true (M522 I0 A1)
 * M523 - Enable min endstop input 1=true, -1=false (M523 X1 Y1 Z1)
 * M524 - Enable max endstop input 1=true, -1=false (M524 X-1 Y-1 Z-1)
 * M525 - Set homing direction 1=+, -1=- (M525 X-1 Y-1 Z-1)
 * M526 - Invert endstop inputs 0=false, 1=true (M526 X0 Y0 Z0)
 * 
 * M600 - Printing pause
 * M601 - Printing resume
 *
 * M665 - Set delta configurations
 * M666 - Set delta endstop adjustment
 *
 * M906 - Set motor current (mA) 
 *        (M906 X1000 Y1000 Z1000 E1000 B1000) or set all (M906 S1000)
 * M907 - Set motor current (raw) 
 *        (M907 X128 Y128 Z128 E128 B128) or set all (M907 S128)
 *
 * M908 - Set feedrate multiply (M908 S200)
 * M909 - has heat bed platform
 *        S1 -> Yes, S0 -> No
 * M910 - finish sending parameters
 * M911 - Whether enable auto current adjust or not, 
 *        S1 -> enable, S0 -> disable
 * M912 - Whether enable auto slow down or not, 
 *        S1 -> enable, S0 -> disable
 * M913 - Machine type: 
 *        S0 -> xyz, 
 *        S1 -> delta, 
 *        S2 -> corexy
 * M914 - Set servo endstop angle, M914 S0 E90
 */
static int gcode_process_g(char *line, int value, bool send_ok) 
{
    switch (value) 
    { 
        case 0:
        case 1:
            /* G0-G1: Coordinated Movement X Y Z E */
            get_coordinates(line);

#ifdef RETRACT
#if 0  //FIXME
            if (auto_retract_enbaled) {
                if (!(has_code(line,axis_codes[X_AXIS]) 
                      || has_code(line,axis_codes[Y_AXIS])
                      || has_code(line,axis_codes[Z_AXIS]))
                    && has_code(line,axis_codes[E_AXIS])) {
                    float echange = destination[E_AXIS] - current_position[E_AXIS];

                    if ((echange < -MIN_RETRACT && !retracted) 
                        || (echange > MIN_RETRACT && retracted)) {
                        /* Hide the slicer generated retract/recove from caculation */
                        current_position[E_AXIS] = destination[E_AXIS];
                        plan_set_e_position(current_position[E_AXIS]);
                        retract(!retracted);
                        break;
                    }
                }
            }
#endif
#endif
            prepare_move();
            break;
        case 2:
            /* G2: CW ARC */
            get_arc_coordinates(line);
            prepare_arc_move(1);
            break;
        case 3:
            /* G3: CCW ARC */
            get_arc_coordinates(line);
            prepare_arc_move(0);
            break;
        case 4:
            /* G4: Dwell S<seconds> or P<milliseconds> */
            break;
#ifdef RETRACT
        case 10:
            /* G10: Retract */
			if (pa.ext_count > 1) {
				int tx = 0;
				if (has_code(line, 'S')) {
					tx = get_int(line, 'S') == 1;
				}
				retracted_swap[active_extruder]= tx; // checks for swap retract argument
				retract(true, retracted_swap[active_extruder]);
			} else {
				retract(true, false); 
			}
            break;
        case 11:
            /* G11: Retract recover */
			if (pa.ext_count > 1) {
				retract(false, retracted_swap[active_extruder]);
			} else {
				retract(false, false);
			}
            break;
#endif
        case 28:
            /* G28: Home all axis one at a time */
    #if 0
            homing_axis(line);
    #else 
            homing_axis_plan_buffer(line);
    #endif
            gcode_set_feed(100);
			gcode_set_extruder_feed(100);
            break;

#ifdef AUTO_LEVELING
		case 29:
			{
				if (! pa.autoLeveling) {
					break;
				}

				if (has_code(line, 'S')) {
					int s_val = -1;
					s_val = get_int(line, 'S');
					if (s_val == 1) { //save eeprom
						struct stat st;
						if (stat(AUTO_LEVEL_BIN, &st) != 0) {
							auto_bed_leveling(true);
							if (pa.machine_type == MACHINE_DELTA) {
								save_autolevel((float *)bed_level, sizeof(bed_level));
							} else {
								save_autolevel((float *)plan_bed_level_matrix.matrix, sizeof(plan_bed_level_matrix.matrix));
							}
						} else {
							if (pa.machine_type == MACHINE_DELTA) {
								load_autolevel((float *)bed_level, sizeof(bed_level));
							} else {
								matrix_set_to_identity(&plan_bed_level_matrix);
								if (pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
									dock_sled(false, 0);
								stepper_sync();

								//float measured_z1 = probe_bed_height(140, 180 
								//							, current_position[Z_AXIS] + pa.zRaiseBeforeProbing);
								float measured_z1 =	probe_bed_height(fabs(pa.endstopOffset[X_AXIS]) + 1, 
												fabs(pa.endstopOffset[Y_AXIS]) + 1, current_position[Z_AXIS] + pa.zRaiseBeforeProbing);
								float measured_z2 =	probe_bed_height(fabs(pa.endstopOffset[X_AXIS]) + 1, 
												fabs(pa.endstopOffset[Y_AXIS]) + 1, current_position[Z_AXIS] + pa.zRaiseBeforeProbing);
								float measured_z3 =	probe_bed_height(fabs(pa.endstopOffset[X_AXIS]) + 1, 
												fabs(pa.endstopOffset[Y_AXIS]) + 1, current_position[Z_AXIS] + pa.zRaiseBeforeProbing);
								float measured_z = (measured_z1 + measured_z2 + measured_z3) /3.0;
    							current_position[Z_AXIS] = measured_z;

								printf("lkj 0 new x:%f, y:%f, z:%f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
								load_autolevel((float *)plan_bed_level_matrix.matrix, 
										sizeof(plan_bed_level_matrix.matrix));
								plan_set_position_no_delta_autolevel(current_position[X_AXIS],
										current_position[Y_AXIS],
										current_position[Z_AXIS],
										current_position[E_AXIS]);

								float real_z = (stepper_get_position(Z_AXIS)) / pa.axis_steps_per_unit[Z_AXIS]; 
								float x = current_position[X_AXIS] - pa.endstopOffset[X_AXIS];
								float y = current_position[Y_AXIS] - pa.endstopOffset[Y_AXIS];
								float z = current_position[Z_AXIS];
								apply_rotation_xyz(plan_bed_level_matrix, &x, &y, &z);  
								current_position[Z_AXIS] = z - real_z + fabs(pa.endstopOffset[Z_AXIS]);
								printf("lkj new x:%f, y:%f, z:%f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
								plan_set_position_no_delta_autolevel(current_position[X_AXIS],
										current_position[Y_AXIS],
										current_position[Z_AXIS],
										current_position[E_AXIS]);

								if (pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
									dock_sled(true, 0);
							}
						}
					}
				} else {
					auto_bed_leveling(false);
				}
			}
			break;

        case 30:
            /* G30: Single Z-Probe */
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
            	single_z_probe();
            break;
#endif
		case 31: // dock the sled
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
				dock_sled(true, 0);
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_Z_PIN)
				delta_min_z_pin_dock(true);
			break;

		case 32: // undock the sled
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY)
				dock_sled(false, 0);
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_Z_PIN)
				delta_min_z_pin_dock(false);
			break; 

        case 90:
            /* G90: Use Absolute Coordinates */
            relative_mode = false;
            break;
        case 91:
            /* G91: Use Relative Coordinates */
            relative_mode = true;
            break;
        case 92:
            /* G92: Set current position to coordinates given */
            if (!has_code(line, axis_codes[E_AXIS])) {
                stepper_sync();
            }

            int i;
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line, axis_codes[i])) {
					if(i == E_AXIS) {
						current_position[i] = get_float(line, axis_codes[i]);
						plan_set_e_position(current_position[E_AXIS]);
					} else if (i == X_AXIS || i == Y_AXIS || i == Z_AXIS) {
                    	current_position[i] = get_float(line, axis_codes[i]);
						if (pa.machine_type == MACHINE_DELTA) {
							calculate_delta(current_position);
							plan_set_position(delta[X_AXIS],
									delta[Y_AXIS],
									delta[Z_AXIS],
									current_position[E_AXIS]);

							stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
									lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
									lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
									lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
						} else {
							plan_set_position(current_position[X_AXIS], 
									current_position[Y_AXIS],
									current_position[Z_AXIS],
									current_position[E_AXIS]);
						}
					}
                }
            }
            break;
        default:
            printf("[Warning: not support G%d command!\n", value);
            break;
    }

    if (unicorn_get_mode() == FW_MODE_REMOTE && send_ok) {
        gcode_send_response_remote("ok\n");
    }

    return SEND_REPLY;
}

static unsigned int mcode_index = 0;
static char mountPath[1024] = {0};
static int gcode_process_m(char *line, int value, bool send_ok)
{
    int i = 0 ;
    int idx = 0;
    int mode = 1;
    int level = 0;
    int current = 0;
    int multiply = 0;
    uint8_t limited = 0;
    double temp_ext[MAX_EXTRUDER] = {0.0};
    double temp_bed = 0.0;            
    double setpoint_ext[MAX_EXTRUDER] = {0.0};
    double setpoint_bed = 0.0;            
    int temp_ext_reached = 0;
    int temp_bed_reached = 0;
    char buf[150] = {0};
	int buf_pos = 0;
	float pos_x, pos_y, pos_z, pos_e;

    channel_tag heater, fan;//,fan2;
#ifdef SERVO
    channel_tag servo;
    uint32_t angle = 0;
#endif
    pid_settings pid;

    switch (value)
    {
        case 17:
            /* M17: Enable all stepper motors */
            stepper_enable_drivers();
            break;

        case 18:
            /* M18: Disable all stepper motors power */
            stepper_disable_drivers();
            break;

        case 20:
            /* M20: list sd files */
			if (send_ok){
	            gcode_send_response_remote("ok\n");
			}
            gcode_send_response_remote("Begin file list\n");
            printf("start scan :%s\n", mountPath);
			scan_dir(mountPath, ".gcode");
            sleep(1);
            gcode_send_response_remote("End file list\n");

            return SEND_REPLY;
        case 21:
            /* M21: init sd card */
   			{
			//korea client	char *external_dev_1[]={"sda", "sda1", "sda2"};
				//char *external_dev_2[]={"mmcblk0", "mmcblk0p1", "sda", "sda1"};
				char *external_dev_1[]={"mmcblk1", "mmcblk1p1", "sda", "sda1", "sda2"};
				char *external_dev_2[]={"mmcblk0", "mmcblk0p1"};
				char *external_dev_android[]={"sdcard1", "usb1"};
				int dev_size = sizeof(external_dev_1)/sizeof(external_dev_1[0]);
				int i=0, ret=0;
				int mmc0_is_rootfs = rootfs_is_mmcblk0();

				if (is_android_system()) {
					memset(mountPath, 0, sizeof(mountPath));
					for (i=0; i<dev_size; i++) {
						ret = androidPathIsMounted(external_dev_android[i], mountPath);
						if( ret ) {
							COMM_DBG("mountPath=%s\n", mountPath);
							break;
						}
					}
				} else {
					memset(mountPath, 0, sizeof(mountPath));
					for (i=0; i<dev_size; i++) {
						#if 1
						if (mmc0_is_rootfs) {
							ret = deviceIsMounted(external_dev_1[i], mountPath);
						} else {
							ret = deviceIsMounted(external_dev_2[i], mountPath);
						}
						#else
						ret = deviceIsMounted(external_dev_1[i], mountPath);
						#endif
						if( ret ) { 
							COMM_DBG("mountPath=%s\n", mountPath);
							break;
						}
					}
				}
				if (send_ok){
        	    	gcode_send_response_remote("ok\n");
				}
                if (mountPath[0] == 0) {
                 	gcode_send_response_remote("SD init fail\n"); 
                } else {
                    char tmp[1024]={0}; 
                    sprintf(tmp, "SD card ok:%s\n", mountPath); 
					gcode_send_response_remote(tmp);
                }
            }
            return SEND_REPLY;
        case 22:
            /* M22: release sd card */
			if (send_ok){
				gcode_send_response_remote("ok\n");
			}
            gcode_send_response_remote("SD init fail\n");

            return SEND_REPLY;
        case 23:
            /* M23: select sd file */
            break;
        case 24:
            /* M24: start/resume sd print */
            break;
        case 25:
            /* M25: pause sd print */
            break;
        case 26:
            /* M26: set sd position in bytes */
            break;
        case 27:
            /* M27: report sd print status */
            break;
        case 28:
            /* M28: begin write to sd file */
            break;
        case 29:
            /* M29: stop writing sd file */
            break;
        case 30:
            /* M30: Delete a file on the sd card */
            break;
        
        case 80:
			GCODE_DBG("M80 Start...\n");
            mcode_index = 0;
			if (motor_is_disable()) {
            	stepper_enable_drivers();
			}
			if (stepper_is_stop() == false) {
				GCODE_DBG("M80 stepper already run\n");
            	break;
			}

            relative_mode = false;
		    tmp_extruder = 0;
            active_extruder = 0;

            unicorn_restart();

			if (pa.machine_type == MACHINE_DELTA) {
				destination[X_AXIS] = 0.0; 
				destination[Y_AXIS] = 0.0; 
				if((stepper_check_lmsw(X_AXIS) && stepper_check_lmsw(Y_AXIS) 
							&& stepper_check_lmsw(Z_AXIS))){
					destination[Z_AXIS] = pa.z_home_pos;
				} else {
					destination[Z_AXIS] = 0.0;
				}

				current_position[X_AXIS] = destination[X_AXIS];
				current_position[Y_AXIS] = destination[Y_AXIS];
				current_position[Z_AXIS] = destination[Z_AXIS];

				calculate_delta(current_position);

				plan_set_position(delta[X_AXIS], 
								  delta[Y_AXIS], 
								  delta[Z_AXIS], 
								  current_position[E_AXIS]);

				/* Set the current position for PRU */
				stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
									 lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
									 lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
									 lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
			}
			GCODE_DBG("M80 Done...\n");
            break;

        case 82:
            /* M82: Set E codes absolute */
            axis_relative_modes[3] = false;
            break;
        case 83:
            /* M83: Set E codes relative while in Absolute Coordinates (G90) mode */
            axis_relative_modes[3] = true;
            break;

        case 84:
            /* M84: Cancel printing immediately or blocking */
			if (send_ok){
            	gcode_send_response_remote("ok\n");
			}
            GCODE_DBG("M84 Stop...\n");
            if (has_code(line, 'S')) {
            	gcode_send_response_remote("print done\n");
                unicorn_stop(false);
            } else {
                unicorn_stop(true);
            	gcode_send_response_remote("print done\n");
            }

    		matrix_set_to_identity(&plan_bed_level_matrix);

    		destroy_MCode_list();
           	GCODE_DBG("M84 Done...\n");
            return SEND_REPLY;

        case 85:
            /* M85: Set inactivity shudown timer with parameter S<seconds> */
            break;

        case 92:
            /* M92: Set axis_steps_per_unit */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line, axis_codes[i])) {
                    pa.axis_steps_per_unit[i] = get_float(line, axis_codes[i]);
                    axis_steps_per_sqr_second[i] = pa.max_acceleration_units_per_sq_second[i]
                                                   * pa.axis_steps_per_unit[i];
                }
            }

			float min_E_axis_steps_per_sqr_second = pa.max_acceleration_units_per_sq_second[E_AXIS] * pa.axis_steps_per_unit[E_AXIS];
			if (has_code(line, 'A') && (pa.ext_count >1) ) { //e1
				pa.axis_steps_per_unit[NUM_AXIS + 0] = get_float(line, 'A');
				float tmp = pa.max_acceleration_units_per_sq_second[E_AXIS] * pa.axis_steps_per_unit[NUM_AXIS + 0];
				if ( tmp < min_E_axis_steps_per_sqr_second){
					min_E_axis_steps_per_sqr_second  = tmp;
				}
			}
			if (has_code(line, 'B') && (pa.ext_count >2) ) { //e2
				pa.axis_steps_per_unit[NUM_AXIS + 1] = get_float(line, 'B');
				float tmp = pa.max_acceleration_units_per_sq_second[E_AXIS] * pa.axis_steps_per_unit[NUM_AXIS + 1];
				if ( tmp < min_E_axis_steps_per_sqr_second){
					min_E_axis_steps_per_sqr_second  = tmp;
				}
			}

			axis_steps_per_sqr_second[E_AXIS] = min_E_axis_steps_per_sqr_second;
            break;
        case 93:
            /* M93: Send current axis_steps_per_unit to host */
			GCODE_DBG("X:%d Y:%d Z:%d E:%d",
					(int)pa.axis_steps_per_unit[0],
					(int)pa.axis_steps_per_unit[1],
					(int)pa.axis_steps_per_unit[2],
					(int)pa.axis_steps_per_unit[3]);

            if (unicorn_get_mode() == FW_MODE_REMOTE) {
                memset(buf, 0, sizeof(buf));
                sprintf(buf, "M93 X%d Y%d Z%d E%d\n", 
                        (int)pa.axis_steps_per_unit[0],
                        (int)pa.axis_steps_per_unit[1],
                        (int)pa.axis_steps_per_unit[2],
                        (int)pa.axis_steps_per_unit[3]);
				if (send_ok){
    	            gcode_send_response_remote("ok\n");
				}
                gcode_send_response_remote((char *)&buf);
            }
            return SEND_REPLY;

        case 104: 
            /* M104: Set extruder target temp */
            if (has_code(line, 'S')) {
                if (has_code(line, 'T')) {
                    idx = get_int(line, 'T'); 
                } else {
                    idx = 0;
                }
                heater = heater_lookup_by_index(idx);
                if (heater) {
                    heater_set_setpoint(heater, get_uint(line, 'S'));
                    heater_enable(heater);
                }
            }
            break;

        case 105: 
            /* M105: Read current temp */
			if (send_ok){
            	gcode_send_response_remote("ok\n");
			}

			buf_pos = 0;
			temp_ext_reached = 0;
            memset(buf, 0, sizeof(buf));

			for (i=0; i<pa.ext_count; i++) {
				heater = heater_lookup_by_index(i);
				if (heater) {
					heater_get_celsius(heater, &temp_ext[i]);
					heater_get_setpoint(heater, &setpoint_ext[i]);
					if (unicorn_get_mode() == FW_MODE_REMOTE) {
						if (heater_temp_reached(heater)) {
							temp_ext_reached |= 1<<i;
						}
					}
					sprintf(buf + buf_pos, "   T%d:%f %f ",  i, temp_ext[i], setpoint_ext[i]);
					//sprintf(buf + buf_pos, "   T%d:%f %f ",  i, 200.0f, 200.0f);
					buf_pos += 30;
				}
			}

            heater = heater_lookup_by_name("heater_bed");
            if (heater) {
                heater_get_celsius(heater, &temp_bed);
                heater_get_setpoint(heater, &setpoint_bed);

                if (unicorn_get_mode() == FW_MODE_REMOTE) {
                    if (heater_temp_reached(heater)) {
                        temp_bed_reached = 1;
                    }
                }
				if (pa.has_bed) {
					sprintf(buf + buf_pos, "B:%f %f ", temp_bed, setpoint_bed);
					buf_pos += 30;
				}
            }
            
            if (unicorn_get_mode() == FW_MODE_REMOTE) {
				if (temp_ext_reached != (pow(2.0, pa.ext_count) - 1) || ( pa.has_bed && temp_bed_reached == 0)) {
						;
				} else {
					buf[0] = 'O';
					buf[1] = 'K';
				}
				buf[buf_pos] = '\n';
				//COMM_DBG("M105 %s", buf);
			
                /* ok\n 
                 * T:192 200 B:55 70\n
                 */
                //gcode_send_response_remote((char *)&buf);
				gcode_send_byte_response_remote((char *)&buf, buf_pos+1);
            }
            return SEND_REPLY;

        case 106:
            /* M106: Fan 1,2 on */
			if (has_code(line, 'P')) {
					int fan_id = get_uint(line, 'P');
					if ( fan_id == 1){
						if (bbp_board_type == BOARD_BBP1) {
							break;
						}
            			fan = fan_lookup_by_name("fan_ext");
					} else {
            			fan = fan_lookup_by_name("fan_ext2");
					}
			} else {
            	fan = fan_lookup_by_name("fan_ext");
				if (bbp_board_type == BOARD_BBP1) {
					break;
				}
			}
            if (fan) {
                if (has_code(line, 'S')) {
					level = get_uint(line, 'S');
					if (level >255){
						level = level/255;
					}
                    level = constrain(level, 0, 255);   
                    level = level * 100 / 255;
                    if (level >= 100) {
                        level = 99;
                    }

					if (level > 0) {
							fan_enable(fan);
                    }
                    if (level == 0) {
							fan_disable(fan);
                    }
					fan_set_level(fan, level);
                } else {
						fan_enable(fan);
						fan_set_level(fan, DEFAULT_FAN_LEVEL);
                }
            }
            break;
        case 107:
            /* M107: Fan 1,2 off */
			if (has_code(line, 'P')) {
					int fan_id = get_uint(line, 'P');
					if ( fan_id == 1){
						if (bbp_board_type == BOARD_BBP1) {
							break;
						}
            			fan = fan_lookup_by_name("fan_ext");
					} else {
            			fan = fan_lookup_by_name("fan_ext2");
					}
			} else {
            	fan = fan_lookup_by_name("fan_ext");
				if (bbp_board_type == BOARD_BBP1) {
					break;
				}
			}
            if (fan) {
					fan_disable(fan);
            }
            break;

        case 109:
            /* M109: Wait for extruder heater to reach target */
            if (has_code(line, 'T')) {
                idx = get_int(line, 'T'); 
            } else {
                idx = 0;
            }

            heater = heater_lookup_by_index(idx);

            if (has_code(line, 'S')) {
                heater = heater_lookup_by_name("heater_ext");
                if (heater) {
                    heater_set_setpoint(heater, get_uint(line, 'S'));
                    heater_enable(heater);
              		/* Add mode checking, 
				     * If print from local file, waiting untile temp reached.
               	     * If print form remote, return directly.
					 */ 
					if (unicorn_get_mode() != FW_MODE_REMOTE) {
						HEATER_DBG("Start perpare heating\n");
						while (!stop) { 
							if (heater_temp_reached(heater)) {
								break;
							} else {
								sleep(1);
							}
						} 
						HEATER_DBG("perpare heating done\n");
					}
                }
            }

            break; 

        case 110:
            /* M110: Set Current Line number */
            break;

        case 111:
            /* M111: Set debug level */
            if (has_code(line, 'S')) {
                level = get_int(line, 'S');
            }
            if (level > D_MIN && level < D_MAX) {
                debug = level;
            }
            break;

        case 112:
            	gcode_send_response_remote("print done\n");
                unicorn_stop(false);
			GCODE_DBG("E stop M112 \n");
            break;

        case 114:
            /* M114: Display current position */
            pos_x = stepper_get_position_mm(X_AXIS);
            pos_y = stepper_get_position_mm(Y_AXIS);
            pos_z = stepper_get_position_mm(Z_AXIS);
            pos_e = stepper_get_position_mm(E_AXIS);

			STEPPER_DBG("X:%f Y:%f Z:%f E:%f ", 
                         pos_x, pos_y, pos_z, pos_e);

            if (unicorn_get_mode() == FW_MODE_REMOTE) {
                memset(buf, 0, sizeof(buf));
                sprintf(buf, "X:%f Y:%f Z:%f E:%f\n",
														 current_position[X_AXIS],
                                                         current_position[Y_AXIS],                    
                                                         current_position[Z_AXIS],
                                                         current_position[E_AXIS]); //lkj pos_x,??????? 
				if (send_ok){
                	gcode_send_response_remote("ok\n");
				}
                gcode_send_response_remote((char *)&buf);
            }
            return SEND_REPLY;

        case 115:
            /* M115: Capabilities string */
			GCODE_DBG("Unicorn firmware version : %s\n", &pa.version[0]);

            if (unicorn_get_mode() == FW_MODE_REMOTE) {
                memset(buf, 0, sizeof(buf));
                sprintf(buf, "Unicorn firmware version %s\n", &pa.version[0]);

				if (send_ok){
                	gcode_send_response_remote("ok\n");
				}
                gcode_send_response_remote((char *)&buf);
            }
            return SEND_REPLY;

        case 119:
            /* M119: show endstop state */
			if (send_ok){
            	gcode_send_response_remote("ok\n");
			}

            memset(buf, 0, sizeof(buf));
            for (i = 0; i < NUM_AXIS - 1; i++) {
                limited = stepper_check_lmsw(i);
                if (limited) {
                	sprintf(buf+strlen(buf), "Min_%c:hit ", axis_name(i));
                    printf("axis_%d hit\n", i);
                } else {
                	sprintf(buf+strlen(buf), "Min_%c:not hit ", axis_name(i));
                    printf("axis_%d not hit\n", i);
                }
                //gcode_send_response_remote((char *)&buf);
            }

			for (i = MAX_X_AXIS; i <= AUTOLEVEL_Z_AXIS; i++) {
				char name[20] = {0};
                //memset(buf, 0, sizeof(buf));
				if( i == MAX_X_AXIS ){
					strcpy(name, "Max_X");
				} else if( i == MAX_Y_AXIS ){
					strcpy(name, "Max_Y");
				} else if( i == MAX_Z_AXIS ){
					strcpy(name, "Max_Z");
				} else if( i == AUTOLEVEL_Z_AXIS ){
					strcpy(name, "AutoLevel_Z");
				}
                limited = stepper_check_lmsw(i);
                if (limited) {
                	sprintf(buf+strlen(buf), "%s:hit ", name);
                    printf("axis_%d hit\n", i);
                } else {
                	sprintf(buf+strlen(buf), "%s:not hit ", name);
                    printf("axis_%d not hit\n", i);
                }
            } 
                gcode_send_response_remote((char *)&buf);

            return SEND_REPLY;

        case 140:
            /* M140: Set bed temperature */
			if (!pa.has_bed) {
				HEATER_DBG("M140 don't have bed\n");
				break; 
			}
            if (has_code(line, 'S')) {
                heater = heater_lookup_by_name("heater_bed");
                if (heater) {
                    heater_set_setpoint(heater, get_uint(line, 'S'));
                    heater_enable(heater);
                }
            }
            break;

        case 150:
            /* M150: Set Led light level, R<0~255>, G<0~255>, B<0~255>
             * Fan control : R -> Fan_4
             *               G -> Fan_5
             *               B -> Fan_6
             */
            if (has_code(line, 'R')) {
				level = get_uint(line, 'R');
				if (level >255){
					level = level/255;
				}
                level = constrain(level, 0, 255);   
                level = level * 99 / 255;
                fan = fan_lookup_by_name("fan_4");
                if (fan) {
                    fan_set_level(fan, level);
                    if (level > 0) {
                    	fan_enable(fan);
				    } else {
						fan_disable(fan);
                    }
                } 
            }

            if (has_code(line, 'G')) {
				level = get_uint(line, 'G');
				if (level >255){
					level = level/255;
				}
                level = constrain(level, 0, 255);   
                level = level * 99 / 255;
                fan = fan_lookup_by_name("fan_5");
                if (fan) {
                    fan_set_level(fan, level);
                    if (level > 0) {
                    	fan_enable(fan);
				    } else {
						fan_disable(fan);
                    }
                }
            }

            //TODO: need to check board version, 1 or 1S
            if (has_code(line, 'B')) {
				level = get_uint(line, 'B');
				if (level >255){
					level = level/255;
				}
                level = constrain(level, 0, 255);   
                level = level * 99 / 255;
                fan = fan_lookup_by_name("fan_6");
                if (fan) {
                    fan_set_level(fan, level);
                    if (level > 0) {
                    	fan_enable(fan);
				    } else {
						fan_disable(fan);
                    }
                }
            }
            break;

        case 176:
            /* M176: Fan 3 on */
            if (has_code(line, 'S')) {
				level = get_uint(line, 'S');
				if (level >255){
					level = level/255;
				}
                level = constrain(level, 0, 255);   
                level = level * 99 / 255;
                fan = fan_lookup_by_name("fan_3");
                if (fan) {
					if (bbp_board_type == BOARD_BBP1S) {
						fan_set_level(fan, level);
						fan_enable(fan);
					} else {
						if (level) {
							fan_enable(fan);
						} else {
                			fan_disable(fan);
						}	
					}
                }
            }
            break; 

        case 177:
            /* M177: Fan3 off */
            fan = fan_lookup_by_name("fan_3");
            if (fan) {
                fan_disable(fan);
            }
            break;     

        case 190:
			if (!pa.has_bed) {
				HEATER_DBG("M190 don't have bed\n");
				break; 
			}
            /* M190: Wait for bed heater to reach target temperature. */
            if (has_code(line, 'S')) {
                heater = heater_lookup_by_name("heater_bed");
                if (heater) {
                    heater_set_setpoint(heater, get_uint(line, 'S'));
                    heater_enable(heater);
					HEATER_DBG("Start perpare hbp heating\n");
                    if (unicorn_get_mode() != FW_MODE_REMOTE) {
                        while (!stop) { 
                            if (heater_temp_reached(heater)) {
                                break;
                            } else {
                                sleep(1);
                            }
						} 
						HEATER_DBG("perpare heating done\n");
                    }
                }
            }
            break;

        case 200:
            /* M200: Set filament diameter and set E axis units to cubic millimeters */
            break;

        case 201: 
            /* M201: Set maximum acceleration in units/s^2 for print move (M201 X1000 Y1000) */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line, axis_codes[i])) {
                    float acc = get_float(line, axis_codes[i]);
                    pa.max_acceleration_units_per_sq_second[i] = acc;
                    axis_steps_per_sqr_second[i] = acc * pa.axis_steps_per_unit[i];
                }
			}
			float min_E_axis_steps_per_sqr_second_1 = axis_steps_per_sqr_second[E_AXIS];
			if (pa.ext_count >1) { //e1
				float tmp = pa.max_acceleration_units_per_sq_second[E_AXIS] *  pa.axis_steps_per_unit[NUM_AXIS + 0];
					if (tmp < min_E_axis_steps_per_sqr_second_1  ){
						min_E_axis_steps_per_sqr_second_1  = tmp; 
					}
			}
			if (pa.ext_count >2) { //e2
				float tmp = pa.max_acceleration_units_per_sq_second[E_AXIS] *  pa.axis_steps_per_unit[NUM_AXIS + 1];
					if (tmp < min_E_axis_steps_per_sqr_second_1  ){
						min_E_axis_steps_per_sqr_second_1  = tmp; 
					}
			}
			axis_steps_per_sqr_second[E_AXIS] = min_E_axis_steps_per_sqr_second_1 ;
            break;
        case 203:
            /* M203: max feedrate mm/sec */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line, axis_codes[i])) {
                    pa.max_feedrate[i] = get_float(line, axis_codes[i]);
                }
            }
            break;
        case 204:
            /* M204: Acceleration S normal moves T filmanent only moves */
            if (has_code(line, 'S')) {
                pa.move_acceleration = get_float(line, 'S');
            }
            if (has_code(line, 'F')) {
                pa.retract_acceleration = get_float(line, 'F');
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
            if (has_code(line, 'S')) {
                pa.minimumfeedrate = get_float(line, 'S'); 
            }
            if (has_code(line, 'F')) {
                pa.mintravelfeedrate = get_float(line, 'F');
            }
            if (has_code(line, 'B')) {
                pa.minsegmenttime = get_float(line, 'B');
            }
            if (has_code(line, 'X')) {
                pa.max_xy_jerk = get_float(line, 'X');
            }
            if (has_code(line, 'Z')) {
                pa.max_z_jerk = get_float(line, 'Z');
            }
            if (has_code(line, 'E')) {
                pa.max_e_jerk = get_float(line, 'E');
            }
            break;

        case 206:
            /* M206: additional homing offset */
			if (send_ok){
        		gcode_send_response_remote("ok\n");
			}
            if (has_code(line, 'S')) {
                pa.ext_count = get_int(line, 'S');
            }

            if (has_code(line, 'T')) {
                int i = get_int(line, 'T');
                pa.ext_offset[X_AXIS][i] = get_float(line, 'X');
                pa.ext_offset[Y_AXIS][i] = get_float(line, 'Y');
                pa.ext_offset[Z_AXIS][i] = get_float(line, 'Z');
            }

			GCODE_DBG("M206 pa.ext_count:0x%x\n", pa.ext_count);
			for (i = 0; i < pa.ext_count; i++){
				GCODE_DBG("ext[%d] offset x:%f, y:%f, z:%f\n", i, 
                           pa.ext_offset[X_AXIS][i], 
						   pa.ext_offset[Y_AXIS][i], 
                           pa.ext_offset[Z_AXIS][i]);
			}
            return SEND_REPLY;

        case 207:
            /* M207: Set retract length (mm), retract_feedrate (mm/s) */
            if (has_code(line,'S')) {
                pa.retract_length = get_float(line, 'S'); 
            }
            if (has_code(line,'F')) {
                pa.retract_feedrate = get_float(line, 'F'); 
            }
            if (has_code(line,'Z')) {
                pa.retract_zlift = get_float(line, 'Z'); 
            }
            break;

        case 208:
            /* M208: Set recover length */
            if (has_code(line,'S')) {
                pa.retract_recover_length = get_float(line, 'S'); 
            }
            if (has_code(line,'F')) {
                pa.retract_recover_feedrate = get_float(line, 'F'); 
            }
            break;

        case 209:
            /* M209: Enable automatic retract detect. */
            //TODO
            if (has_code(line,'S')) {

            }
            break;

        case 210:
            /* M210: Homing Feedrate mm/min Xnnn Ynnn Znnn */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line,axis_codes[i])) {
                    pa.homing_feedrate[i] = get_float(line, axis_codes[i]);
                }
            }
			if (has_code(line,'D')) {
				pa.autolevel_down_rate = get_float(line, 'D');
			}
            break;

        case 218:
            /* M218: Set hotend offset */
            //TODO
            break;

        case 220:
            /* M220: S<factor in percent> -> set speed factor override percentage */
            if (has_code(line,'S')) {
                multiply = get_int(line, 'S');
                gcode_set_feed(multiply);
            }
            break;
        case 221:
            /* M221: S<factor in percent> -> set extrude factor override percentage */
            if (has_code(line,'S')) {
                int tmp_code = get_uint(line, 'S');
				printf("M221 multiply:%d\n", tmp_code);
				if (has_code(line, 'T')) {
                	int tmp_multiply_extruder = get_uint(line, 'T');
					if (tmp_multiply_extruder > MAX_EXTRUDER) {
						break;
					}
					extruder_multiply[tmp_multiply_extruder] = tmp_code;
				} else {
					extrudemultiply = tmp_code;
				}
            }
            break;
        
        case 280:
            /* M280: Set servo position absolute. P: servo index, S: angle or microseconds */
            // not support microsecond value yet!!
#ifdef SERVO
            if (has_code(line,'P')) { 
                idx = get_uint(line, 'P');
            } else {
                idx = 0;
            }
            servo = servo_lookup_by_index(idx);
            if (has_code(line,'S')) { 
                angle = get_uint(line, 'S');
                if (servo) {
                    servo_set_angle(servo, angle);
                }
            }
#endif
            break;

		case 301:
			/* M301: Set extruder heater PID parameters */
			{
				//lkj
				int ext = 0;
				//char *heater_name[] = {"heater_ext", "heater_ext2", "heater_bed"};
				char *heater_name[] = {"heater_ext", "heater_ext2", "heater_ext3", "heater_bed"};
				//gcode_send_response_remote("ok\n");
				if (has_code(line,'T')) {
					ext = get_uint(line, 'T');
					printf("M301 ext:%d\n", ext);
				}

				heater = heater_lookup_by_name(heater_name[ext]);
				if (heater) {
					heater_get_pid_values(heater, &pid);

					if (has_code(line,'P')) { 
						pid.P = get_float(line, 'P');
					}
					if (has_code(line,'I')) { 
						pid.I = get_float(line, 'I');
					}
					if (has_code(line,'D')) { 
						pid.D = get_float(line, 'D');
					}
					if (has_code(line,'S')) { 
						pid.FF_factor = get_float(line, 'S');
					}
					if (has_code(line,'B')) { 
						pid.FF_offset = get_float(line, 'B');
					}
					if (has_code(line,'W')) { 
						pid.I_limit = get_float(line, 'W');
						//TODO
					}
					//lkj
					printf("pid.P:%f, pid.I:%f, pid.D:%f, pid.I_limit:%f, pid.FF_factor:%f, pid.FF_offset:%f \n",
							pid.P, pid.I, pid.D, pid.I_limit, pid.FF_factor, pid.FF_offset);
					heater_set_pid_values(heater, &pid);
				}
			}
			//return SEND_REPLY;
			break;
        case 303:
            /* M303: PID relay autotune S<temperature>, set the target temperature */
			{
				float temp = 150.0;
				int e=0; 
				int c=5; 
				int w=0; // add for max pwm 
				if (has_code(line,'E')) e = get_int(line, 'E');
				if (e < 0)
					temp=70;
				if (has_code(line,'S')) temp =  get_float(line, 'S');
				if (has_code(line,'C')) c =  get_uint(line, 'C');
				if (has_code(line,'W')) w =  get_uint(line, 'W');
        		gcode_send_response_remote("ok\n");
				printf("e:%d, temp:%f, c=%d\n", e, temp, c);
				if (bbp_board_type == BOARD_BBP1 && e>=2) {
        			gcode_send_response_remote("PID autotune filed, BBP1 only have two extruder\n");
					return 1;
				}
				pid_autotune(temp, e, c, w, gcode_send_response_remote);
				if ((e > MAX_EXTRUDER) || (!pa.has_bed  && e < 0)) {
        			gcode_send_response_remote("PID autotune filed\n");
				}
			}
			return 1;
            //break;
        case 304:
            /* M304: Set Hot Bed PID parameters */
            heater = heater_lookup_by_name("heater_hbp");
            if (heater) {
                heater_get_pid_values(heater, &pid);

                if (has_code(line,'P')) { 
                    pid.P = get_uint(line, 'P');
                }
                if (has_code(line,'I')) { 
                    pid.I = get_uint(line, 'I');
                }
                if (has_code(line,'D')) { 
                    pid.D = get_uint(line, 'D');
                }
                if (has_code(line,'S')) { 
                    pid.FF_factor = get_uint(line, 'S');
                }
                if (has_code(line,'B')) { 
                    pid.FF_offset = get_uint(line, 'B');
                }
                if (has_code(line,'W')) { 

                }
                heater_set_pid_values(heater, &pid);
            }
            break;

        case 350:
            break;

        case 355:
            /* M355*/
            break;

        case 400:
            /* M400: finish all moves */
            stepper_sync();
            break;

        case 401:
            /* M401: Lower z-probe */
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
            	engage_z_probe();
            break;
        case 402:
            /* M402: Raise z-probe */
			if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_SERVO)
            	retract_z_probe();
            break;

        case 404:
            /* M404: Filament width */
            //TODO
            break;
        case 405:
            /* M405: Filament sensor on */
            //TODO
            break;
        case 406:
            /* M406: Filament sensor off */
            //TODO
            break;
        case 407:
            /* M407: Display filament diameter */
            //TODO
            break;

        case 500:
            /* M500: Store parameters in EEEPROM */
        	//gcode_send_response_remote("ok\n");
            parameter_save_to_eeprom();
			break;
            //return SEND_REPLY;
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
            parameter_restore_default();
            break;
		case 503:
			/* M503: show settings */
			//FIXME: Send parameters to host
			if (has_code(line,'A')) {
				int a_val = get_int(line, 'A');
				if (a_val == 0) { //erase eeprom
					unlink(AUTO_LEVEL_BIN);
					reset_bed_level();
					matrix_set_to_identity(&plan_bed_level_matrix);
				} else if (a_val == 1) { //print parameters 
					if (pa.machine_type == MACHINE_DELTA) {
						float tmp_bed_level[100][100];
						memset(tmp_bed_level, 0, sizeof(tmp_bed_level));
						load_autolevel((float *)tmp_bed_level, sizeof(tmp_bed_level));
						print_bed_level(tmp_bed_level);			
					} else {
						matrix_t bed_level_matrix= {
							.matrix = {
								1.0, 0.0, 0.0,
								0.0, 1.0, 0.0,
								0.0, 0.0, 1.0 
							},
						};
						load_autolevel((float *)bed_level_matrix.matrix, 
								sizeof(bed_level_matrix.matrix));
						printf("matrix 0-8:%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
								bed_level_matrix.matrix[0], bed_level_matrix.matrix[1], 
								bed_level_matrix.matrix[2], bed_level_matrix.matrix[3], 
								bed_level_matrix.matrix[4], bed_level_matrix.matrix[5],
								bed_level_matrix.matrix[6], bed_level_matrix.matrix[7], 
								bed_level_matrix.matrix[8]);	
						char buf[750] = {0};
						memset(buf, 0, sizeof(buf));
						sprintf(buf + strlen(buf), "%f,%f,%f\n", bed_level_matrix.matrix[0], 
								bed_level_matrix.matrix[1], bed_level_matrix.matrix[2]);
						sprintf(buf + strlen(buf), "%f,%f,%f\n", bed_level_matrix.matrix[3], 
								bed_level_matrix.matrix[4], bed_level_matrix.matrix[5]);
						sprintf(buf + strlen(buf), "%f,%f,%f\n", bed_level_matrix.matrix[6], 
								bed_level_matrix.matrix[7], bed_level_matrix.matrix[8]);
						gcode_send_response_remote((char *)&buf);
					}
				}
			} else {
				parameter_dump(gcode_send_response_remote);
			}
            break;

        case 504:
            /* M504: get firmware path */
			if (has_code(line,'F')) {
				//M504 F1 ext1:/.octoprint/upload/aa.fbot
				//N34 M504 F1.000000 extruder1:/.octoprint/uploads/temp_curve_100K.fbot*125
                char curve_type[10] = {0}, curve_file_path[100] = {0};
                const char *line_start;
                char *p, *line_path, *line_end;

                line_start = get_str(line, 'F');
                char *line_comma = strstr(line_start, ":");
				if (line_comma == NULL) {
					GCODE_DBG("bad curve format \n");
					break;
				}
                line_start = strstr(line_start, "extruder");
				if (line_start == NULL) {
                	line_start = strstr(line, "bed");
					if (line_start == NULL) {
						GCODE_DBG("bad curve format, cann't find curve type\n");
						break;
					}
				}
				if (line_comma < line_start) {
					GCODE_DBG("bad curve format, curve type is null\n");
					break;
				}
				p = (char *)line_start;
                line_path = strstr(p, ":");

				memset(curve_type, 0, sizeof(curve_type));
				memset(curve_file_path, 0, sizeof(curve_file_path));

				strncpy(curve_type, line_start , line_path - line_start);
				line_path++; 
				line_end = strstr(line_path, "*");
				strncpy(curve_file_path, line_path, line_end - line_path);
				GCODE_DBG("M504 get curve_type:%s, curve_file:%s\n", curve_type, curve_file_path);

				curve_config_save_to_eeprom(curve_type, EEPROM_DEV, curve_file_path);

				GCODE_DBG("load_temp_curve_from_eeprom\n");
				load_temp_curve_from_eeprom(EEPROM_DEV);
				break;
			}

             if (has_code(line,'S')) {
                const char *firmware_version = get_str(line, 'S');
				strncpy(pa.version, firmware_version, 4);
				eeprom_write_pru_code(EEPROM_DEV, 0, PRU_UPLOAD_FIRMWARE_PATH);
				eeprom_read_pru_code(EEPROM_DEV, 0, "/tmp/pru.bin");
				pa.pru_checksum = calculate_pru_file_crc("/tmp/pru.bin");
				parameter_save_to_eeprom();
				GCODE_DBG("M504 get firmware version:%s, pru checksum:%lu \n", firmware_version, pa.pru_checksum);
				GCODE_DBG("M504 write pru code to epprom :%s\n", PRU_UPLOAD_FIRMWARE_PATH);
				GCODE_DBG("M504 get firmware version:%s\n", firmware_version);
			 }
             
            break;
        case 505:
            /* M505: Save parameter to sd card file */
            //TODO
            parameter_save_to_sd();
            break;

        case 510:
            /* M510: Axis invert */
            if (has_code(line,'X')) {
                pa.invert_x_dir = get_bool(line, 'X');
            }
            if (has_code(line,'Y')) {
                pa.invert_y_dir = get_bool(line, 'Y');
            }
            if (has_code(line,'Z')) {
                pa.invert_z_dir = get_bool(line, 'Z');
            }
            if (has_code(line,'E')) {
                pa.invert_e_dir = get_bool(line, 'E');
            }
            break;

        case 520:
            /* M520: Maximum Area unit */
            if (has_code(line,'X')) {
                pa.x_max_length = abs(get_int(line, 'X'));
            }
            if (has_code(line,'Y')) {
                pa.y_max_length = abs(get_int(line, 'Y'));
            }
            if (has_code(line,'Z')) {
                pa.z_max_length = abs(get_int(line, 'Z'));
            }
            break;

        case 521:
            /* M521: Disable axis when unused */
            if (has_code(line,'X')) {
                pa.disable_x_en = get_bool(line, 'X');
            }
            if (has_code(line,'Y')) {
                pa.disable_y_en = get_bool(line, 'Y');
            }
            if (has_code(line,'Z')) {
                pa.disable_z_en = get_bool(line, 'Z');
            }
            if (has_code(line,'E')) {
                pa.disable_e_en = get_bool(line, 'E');
            }
            break;

        case 522:
            /* M522: Software Endstop */
            if (has_code(line,'I')) {
                pa.min_software_endstops = get_bool(line, 'I');
            }
            if (has_code(line,'A')) {
                pa.max_software_endstops = get_bool(line, 'A');
            }
            break;

        case 523:
            /* M523: Min Endstop */
            if (has_code(line,'X')) {
                pa.x_min_endstop_aktiv = get_int(line, 'X') == 1 ? 1 : -1;
            }
            if (has_code(line,'Y')) {
                pa.y_min_endstop_aktiv = get_int(line, 'Y') == 1 ? 1 : -1;
            }
            if (has_code(line,'Z')) {
                pa.z_min_endstop_aktiv = get_int(line, 'Z') == 1 ? 1 : -1;
            }
            break;

        case 524:
            /* M524: Max Endstop */
            if (has_code(line,'X')) {
                pa.x_max_endstop_aktiv = get_int(line, 'X') == 1 ? 1 : -1;
            }
            if (has_code(line,'Y')) {
                pa.y_max_endstop_aktiv = get_int(line, 'Y') == 1 ? 1 : -1;
            }
            if (has_code(line,'Z')) {
                pa.z_max_endstop_aktiv = get_int(line, 'Z') == 1 ? 1 : -1;
            }
            break;

        case 525:
            /* M525: Homing Direction */
            //FIXME:
            //Change the homing direction in pru code!!
            if (has_code(line,'X')) {
                pa.x_home_dir = get_int(line, 'X') == 1 ? 1 : -1;
            }
            if (has_code(line,'Y')) {
                pa.y_home_dir = get_int(line, 'Y') == 1 ? 1 : -1;
            }
            if (has_code(line,'Z')) {
                pa.z_home_dir = get_int(line, 'Z') == 1 ? 1 : -1;
            }

            break;

        case 526:
            /* M526: Endstop Invert */
            if (has_code(line,'X')) {
                pa.x_endstop_invert = get_bool(line, 'X');
                stepper_config_lmsw(X_AXIS, pa.x_endstop_invert);
            }
            if (has_code(line,'Y')) {
                pa.y_endstop_invert = get_bool(line, 'Y');
                stepper_config_lmsw(Y_AXIS, pa.y_endstop_invert);
            }
            if (has_code(line,'Z')) {
                pa.z_endstop_invert = get_bool(line, 'Z');
                stepper_config_lmsw(Z_AXIS, pa.z_endstop_invert);
            }
            if (has_code(line,'A')) {
                pa.autolevel_endstop_invert = get_bool(line, 'A');
                stepper_config_lmsw(AUTOLEVEL_Z_AXIS, pa.autolevel_endstop_invert); //set pru invert 
				if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY) {
	            	stepper_autoLevel_gpio_turn(false);
				}
				//set gpio when turn on autolevel. see func stepper_autoLevel_gpio_turn() 
            }
            break;
        
        case 600:
            /* M600: Printing pause */
            unicorn_pause();
            break;

        case 601:
            /* M601: Printing resume */
            unicorn_resume();
            break;

        case 602:
            /* M602: upload, or unload, pause filament*/
            if (has_code(line,'S')) { // upload
                unsigned int filament_cmd = get_int(line, 'S');
				COMM_DBG("get filament cmd:%d \n", filament_cmd);
				stepper_load_filament(filament_cmd); //1, upload , 2 unload , 3 pause 
            }
            break;

        case 665:
            /* M665: Set delta configurations */
            if (has_code(line,'D')) {  //D lkj fix ?????
                pa.diagonal_rod = get_float(line, 'D');
            }
            if (has_code(line,'R')) {
                pa.radius = get_float(line, 'R');
            }
            if (has_code(line,'S')) {
                pa.segments_per_second = get_float(line, 'S');
            }
            if (has_code(line,'Z')) {
                pa.z_home_pos = get_float(line, 'Z');
            }
			set_delta_constants();

            break;
#if 0 //lkj
        case 666:
            /* M666: Set delta endstop adjustment */
            for (i = 0; i < 3; i++) {
                if (has_code(line,axis_codes[i])) {
                    pa.endstop_adjust[i] = get_float(line, axis_codes[i]);
                }
            }
            break;
#endif

		case 666: // M666 set delta endstop and geometry adjustment
            if (has_code(line,'P')) {
                pa.delta_print_radius = get_float(line, 'P');
				set_delta_constants();
            }
			{
				float x_min_pos = 0, y_min_pos = 0;
				float x_max_pos = 0, y_max_pos = 0;
#define MIN_PROBE_EDGE 5 // The Z probe minimum square sides can be no smaller than this.
				if (pa.machine_type == MACHINE_DELTA) {
					//		pa.delta_print_radius = pa.radius - 40;
					x_min_pos = -pa.delta_print_radius + MIN_PROBE_EDGE;
					y_min_pos = -pa.delta_print_radius + MIN_PROBE_EDGE;
					x_max_pos =  pa.delta_print_radius - MIN_PROBE_EDGE;
					y_max_pos =  pa.delta_print_radius - MIN_PROBE_EDGE;

					float radius = pa.delta_print_radius;
					pa.probeLeftPos = -radius + MIN_PROBE_EDGE;
					pa.probeRightPos = radius - MIN_PROBE_EDGE;
					pa.probeFrontPos = -radius + MIN_PROBE_EDGE;
					pa.probeBackPos = radius - MIN_PROBE_EDGE;
				} else {
					x_min_pos = 0;
					y_min_pos = 0;
					x_max_pos = pa.x_max_length;
					y_max_pos = pa.y_max_length;
				}

				min_probe_x = max(x_min_pos, x_min_pos + (-1) * pa.endstopOffset[X_AXIS]);
				max_probe_x = min(x_max_pos, x_max_pos + (-1) * pa.endstopOffset[X_AXIS]);
				min_probe_y = max(y_min_pos, y_min_pos + (-1) * pa.endstopOffset[Y_AXIS]);
				max_probe_y = min(y_max_pos, y_max_pos + (-1) * pa.endstopOffset[Y_AXIS]);
				//printf("debug, probe x range: %f - %f \n", min_probe_x, max_probe_x);
				//printf("debug, probe y range: %f - %f \n", min_probe_y, max_probe_y);

				bool left_out_l = pa.probeLeftPos  < min_probe_x,
					 left_out = left_out_l || pa.probeLeftPos > pa.probeRightPos - (MIN_PROBE_EDGE),
					 right_out_r = pa.probeRightPos > max_probe_x,
					 right_out = right_out_r || pa.probeRightPos < pa.probeLeftPos + MIN_PROBE_EDGE,
					 front_out_f = pa.probeFrontPos < min_probe_y,
					 front_out = front_out_f ||  pa.probeFrontPos > pa.probeBackPos - (MIN_PROBE_EDGE),
					 back_out_b = pa.probeBackPos > max_probe_y,
					 back_out = back_out_b || pa.probeBackPos <  pa.probeFrontPos + MIN_PROBE_EDGE;

				if (left_out || right_out || front_out || back_out) {
					if (left_out) {
						printf("Left is out of range\n");
						pa.probeLeftPos = left_out_l ? min_probe_x : pa.probeRightPos - (MIN_PROBE_EDGE);
					}
					if (right_out) {
						printf("Right is out of range\n");
						pa.probeRightPos = right_out_r ? max_probe_x : pa.probeLeftPos + MIN_PROBE_EDGE;
					}
					if (front_out) {
						printf("Front is out of range\n");
						pa.probeFrontPos = front_out_f ? min_probe_y : pa.probeBackPos - (MIN_PROBE_EDGE);
					}
					if (back_out) {
						printf("Back is out of range\n");
						pa.probeBackPos = back_out_b ? max_probe_y :  pa.probeFrontPos + MIN_PROBE_EDGE;
					}
				}
				printf("lkj debug front:%f,back:%f, left:%f, right:%f\n", pa.probeFrontPos, pa.probeBackPos, pa.probeLeftPos, pa.probeRightPos);
			}
			if (pa.machine_type != MACHINE_DELTA) {
				break;
			}

			int i = 0;
			for(i=0; i < 3; i++) {
				if (has_code(line, axis_codes[i])) 
					pa.endstop_adj[i] = get_float(line, axis_codes[i]);
			}

			if (has_code(line, 'A')) {
				pa.radius_adj[0] = get_float(line, 'A');
				set_delta_constants();
			}
			if (has_code(line, 'B')) {
				pa.radius_adj[1] = get_float(line, 'B');
				set_delta_constants();
			}
			if (has_code(line, 'C')) {
				pa.radius_adj[2] = get_float(line, 'C');
				set_delta_constants();
			}
			if (has_code(line, 'I')) {
				pa.diagonal_rod_adj[0] = get_float(line, 'I');
				set_delta_constants();
			}
			if (has_code(line, 'J')) {
				pa.diagonal_rod_adj[1] = get_float(line, 'J');
				set_delta_constants();
			}
			if (has_code(line, 'K')) {
				pa.diagonal_rod_adj[2] = get_float(line, 'K');
				set_delta_constants();
			}
			if (has_code(line, 'R')) {
				pa.radius = get_float(line, 'R');
				set_delta_constants();
			}
			if (has_code(line, 'D')) {
				pa.diagonal_rod = get_float(line, 'D');
				set_delta_constants();
			}
			if (has_code(line, 'H')) {
				pa.z_home_pos = get_float(line, 'H');
				set_delta_constants();
			}
            if (has_code(line,'S')) {
                pa.segments_per_second = get_float(line, 'S');
				set_delta_constants();
            }
			//if (has_code(line, 'P')) {
		    //		z_probe_offset[Z_AXIS]= get_float(line, 'P');
		    //}

			if (has_code(line, 'L')) {
		    	printf("Current Delta geometry values:\n");
		    	printf("X (Endstop Adj):%f \n", pa.endstop_adj[0]);
		    	printf("Y (Endstop Adj):%f \n", pa.endstop_adj[1]);
				printf("Z (Endstop Adj):%f \n", pa.endstop_adj[2]);
				printf("(Z-Probe Offset): X:%f, Y:%f, Z:%f\n", pa.endstopOffset[X_AXIS], pa.endstopOffset[Y_AXIS], pa.endstopOffset[Z_AXIS]);
				printf("A (Tower A Radius adj):%f\n", pa.radius_adj[0]);
				printf("B (Tower B Radius adj):%f\n", pa.radius_adj[1]);
				printf("C (Tower C Radius adj):%f\n", pa.radius_adj[2]);
				printf("I (Tower A Diagonal rod adj):%f\n", pa.diagonal_rod_adj[0]);
				printf("J (Tower B Diagonal rod adj):%f\n", pa.diagonal_rod_adj[1]);
				printf("K (Tower C Diagonal rod adj):%f\n", pa.diagonal_rod_adj[2]);
				printf("R (Delta Radius):%f\n", pa.radius);
				printf("R (Delta available Radius):%f\n", pa.delta_print_radius);
				printf("D (Diagonal Rod Length):%f\n", pa.diagonal_rod);
				printf("H (Z-Height):%f\n", pa.z_home_pos);
				printf("S segments per second :%f\n", pa.segments_per_second);

            	memset(buf, 0, sizeof(buf));
                sprintf(buf+strlen(buf), "Current Delta geometry values:\n");
				sprintf(buf+strlen(buf), "A (Tower A Radius adj):%f\n", pa.radius_adj[0]);
				sprintf(buf+strlen(buf), "B (Tower B Radius adj):%f\n", pa.radius_adj[1]);
				sprintf(buf+strlen(buf), "C (Tower C Radius adj):%f\n", pa.radius_adj[2]);
		    	sprintf(buf+strlen(buf), "X (Endstop Adj):%f \n", pa.endstop_adj[0]);
		    	sprintf(buf+strlen(buf), "Y (Endstop Adj):%f \n", pa.endstop_adj[1]);
				sprintf(buf+strlen(buf), "Z (Endstop Adj):%f \n", pa.endstop_adj[2]);
                gcode_send_response_remote((char *)&buf);

            	memset(buf, 0, sizeof(buf));
				sprintf(buf+strlen(buf), "I (Tower A Diagonal rod adj):%f\n", pa.diagonal_rod_adj[0]);
				sprintf(buf+strlen(buf), "J (Tower B Diagonal rod adj):%f\n", pa.diagonal_rod_adj[1]);
				sprintf(buf+strlen(buf), "K (Tower C Diagonal rod adj):%f\n", pa.diagonal_rod_adj[2]);
				sprintf(buf+strlen(buf), "R (Delta Radius):%f\n", pa.radius);
				sprintf(buf+strlen(buf), "P (Delta Available Radius):%f\n", pa.delta_print_radius);
                gcode_send_response_remote((char *)&buf);

            	memset(buf, 0, sizeof(buf));
				sprintf(buf+strlen(buf), "D (Diagonal Rod Length):%f\n", pa.diagonal_rod);
				sprintf(buf+strlen(buf), "H (Z-Height):%f\n", pa.z_home_pos);
				sprintf(buf+strlen(buf), "S segments per second :%f\n", pa.segments_per_second);
                gcode_send_response_remote((char *)&buf);
			}
         break;

        case 906:
            /* M906: set motor current value in mA using axis codes 
             * M906 X[mA] Y[mA] Z[mA] E[mA] B[mA]
             * M906 S[mA] Set all motors current
             */
 			//gcode_send_response_remote("ok\n");

            if (has_code(line,'S')) {
                /* Set all axis current to the same value */
                current = get_int(line, 'S');
                for (i = 0; i < NUM_AXIS; i++) {
                    if (has_code(line,axis_codes[i])) {
                        pa.axis_current[i] = current;
                        stepper_set_current(i, current);
                    }
                }
            }
            
            /* Set axis current */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line,axis_codes[i])) {
                    current = get_int(line, axis_codes[i]);
					if (pa.axis_current[i] != current){
                    	pa.axis_current[i] = current;
                    	stepper_set_current(i, current);
					}
                }
            }

			if (bbp_board_type == BOARD_BBP1S) {
			    pa.axis_current[E2_AXIS] = pa.axis_current[E_AXIS];
				pa.axis_current[E3_AXIS] = pa.axis_current[E_AXIS];
				if (has_code(line,'B')) {
					pa.axis_current[E2_AXIS] = get_int(line, 'B');
				}
				if (has_code(line,'W')) {
					pa.axis_current[E3_AXIS] = get_int(line, 'W');
				}
				if (has_code(line,'U')) {
					pa.axis_current[U_AXIS] =  get_int(line, 'U');
				}

				stepper_set_current(E2_AXIS, pa.axis_current[E2_AXIS]);
				stepper_set_current(E3_AXIS, pa.axis_current[E3_AXIS]);
				stepper_set_current(U_AXIS,  pa.axis_current[U_AXIS]);
			}
			stepper_update();

            //return SEND_REPLY;
			break;

        case 907:
            /* M907: Set motor current value (0-255) using axis codes
             * M907 X[value] Y[value] Z[value] E[value] B[value]
             * M907 S[value] Set all motors current
             */
            /* Assume that max current allowed is 2000mA! */
            if (has_code(line,'S')) {
                /* Set all axis current to the same value */
                current = get_int(line, 'S') * 2000 / 255;
                for (i = 0; i < NUM_AXIS; i++) {
                    if (has_code(line,axis_codes[i])) {
                        pa.axis_current[i] = get_int(line, 'S');
                        stepper_set_current(i, current);
                    }
                }
            }
            
            /* Set axis current */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line,axis_codes[i])) {
                    current = get_int(line, axis_codes[i]) * 2000 / 255;
                    pa.axis_current[i] = current;
                    stepper_set_current(i, current);
                }
            }

            if (has_code(line,'B')) {
                //TODO: Set EXT2 stepper current
            }
            stepper_update();
            break;

        case 908:
            /* M908: HBP Avaiable? S1 -> enable, S0 -> disable */
            if (has_code(line,'S')) {
                pa.has_bed = get_int(line, 'S');
            }
            if (has_code(line,'R')) {
                pa.dangerousThermistor = get_int(line, 'R');
            }
            if (has_code(line,'G')) {
                pa.dangerousThermocouple = get_int(line, 'G');
            }
			GCODE_DBG("M908 has_bed:0x%x, dangerousThermistor:%d, dangerousThermocouple:%d \n", pa.has_bed, pa.dangerousThermistor , pa.dangerousThermocouple);
            break;

        case 909:
			/* M909: Set microstepping mode 
             * M909 X[value] Y[value] Z[value] E[value] B[value]
             * M909 S[value] set all motors
             *       1 -> full step
             *       2 -> 1/2 step
             *       4 -> 1/4 step
             *       16-> 1/16 step
             *       32-> 1/32 step
			 */
			//gcode_send_response_remote("ok\n");

            if (has_code(line,'S')) {
                /* Set all axis to the given mode */
                mode = get_int(line, 'S');
                for (i = 0; i < NUM_AXIS; i++) {
                    if (has_code(line,axis_codes[i])) {
                        pa.axis_ustep[i] = mode;
                        stepper_set_microstep(i, mode);
                    }
                }
            }
			
            
    		unsigned int  axis_ustep[NUM_AXIS] = {0};
			bool change = false;
            /* Set axis microstep */
            for (i = 0; i < NUM_AXIS; i++) {
                if (has_code(line,axis_codes[i])) {
                    mode = get_int(line, axis_codes[i]);
                    axis_ustep[i] = mode;
                }
            }

			for (i = 0; i < NUM_AXIS; i++) {
                if (pa.axis_ustep[i] != axis_ustep[i]) {
                    pa.axis_ustep[i] = axis_ustep[i];
                    stepper_set_microstep(i, pa.axis_ustep[i]);
					change = true;
                }
            }
					
			pa.axis_ustep[E2_AXIS] = pa.axis_ustep[E_AXIS];
			pa.axis_ustep[E3_AXIS] = pa.axis_ustep[E_AXIS];
			if (has_code(line,'B')) {
				pa.axis_ustep[E2_AXIS] = get_int(line, 'B');
			}
			if (has_code(line,'W')) {
				pa.axis_ustep[E3_AXIS] = get_int(line, 'W');
			}

			if (bbp_board_type == BOARD_BBP1S) {

				pa.axis_ustep[U_AXIS]  = pa.axis_ustep[Z_AXIS];

				stepper_set_microstep(E2_AXIS, pa.axis_ustep[E2_AXIS]);
				stepper_set_microstep(E3_AXIS, pa.axis_ustep[E3_AXIS]);
				stepper_set_microstep(U_AXIS, pa.axis_ustep[Z_AXIS]);
			}

            /* if (has_code(line,'B')) {
                //TODO: Set EXT2 stepper microstep mode
            } */

			//if (change) {
            	stepper_update(); 
			//}

			//return SEND_REPLY;
			break;

        case 910:
			/*M910*/
			//gcode_send_response_remote("ok\n");
			stepper_parameter_update();

			if (pa.machine_type == MACHINE_DELTA) {
				destination[X_AXIS] = 0.0; 
				destination[Y_AXIS] = 0.0; 
				if((stepper_check_lmsw(X_AXIS) && stepper_check_lmsw(Y_AXIS) 
							&& stepper_check_lmsw(Z_AXIS))){
					destination[Z_AXIS] = pa.z_home_pos;
				} else {
					destination[Z_AXIS] = 0.0;
				}

				current_position[X_AXIS] = destination[X_AXIS];
				current_position[Y_AXIS] = destination[Y_AXIS];
				current_position[Z_AXIS] = destination[Z_AXIS];

				calculate_delta(current_position);

				plan_set_position(delta[X_AXIS], 
								  delta[Y_AXIS], 
								  delta[Z_AXIS], 
								  current_position[E_AXIS]);

				/* Set the current position for PRU */
				stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
									 lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
									 lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
									 lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
			}

			GCODE_DBG("M910 finish sending parameter \n");
			//return SEND_REPLY;
			break;
            
        case 911:
			/*M911: Wether enable auto current adjust or not, S1 -> enable, S0 -> disable*/
            if (has_code(line,'S')) {
                pa.auto_current = get_int(line, 'S');
            }
            break;

        case 912:
			/*M912: Wether enable auto slow down or not, S1 -> enable, S0 -> disable*/
            if (has_code(line,'S')) {
                pa.slow_down = get_int(line, 'S');
            }
			break;

        case 913:
			/*M913: Machine type: S0 -> xyz, S1 -> delta, S2 -> corexy*/
            if (has_code(line,'S')) {
                pa.machine_type = get_int(line, 'S');
            }
            if (has_code(line,'F')) {
				if (pa.machine_type == MACHINE_XYZ) {
					//#define MOTOR56_MODE_DUAL_X_Y   1
					//#define MOTOR56_MODE_EXTRUDER   0
					pa.bbp1s_dual_xy_mode = get_int(line, 'F');
				} else {
					pa.bbp1s_dual_xy_mode = MOTOR56_MODE_EXTRUDER;
				}
            }
            if (has_code(line,'A')) {
                pa.autoLeveling = get_int(line, 'A');
            }
            if (has_code(line,'L')) {
                pa.probeDeviceType = get_int(line, 'L');
            }
            if (has_code(line,'X')) {
                pa.endstopOffset[X_AXIS] = get_float(line, 'X'); //bug, should probe - nozzle
            }
            if (has_code(line,'Y')) {
                pa.endstopOffset[Y_AXIS] = get_float(line, 'Y');
            }
            if (has_code(line,'Z')) {
                pa.endstopOffset[Z_AXIS] = get_float(line, 'Z');
            }
			if (has_code(line,'B')) {
                pa.zRaiseBeforeProbing = get_float(line, 'B');
            }
			if (has_code(line,'D')) {
                pa.zRaiseBetweenProbing = get_float(line, 'D');
            }
#if 0
            if (has_code(line,'L')) {
                pa.probePoint1[X_AXIS]= get_float(line, 'L');
            }
            if (has_code(line,'B')) {
                pa.probePoint1[Y_AXIS]= get_float(line, 'B');
            }
            if (has_code(line,'T')) {
                pa.probePoint2[X_AXIS]= get_float(line, 'T');
            }
            if (has_code(line,'F')) {
                pa.probePoint2[Y_AXIS]= get_float(line, 'F');
            }
            if (has_code(line,'I')) {
                pa.probePoint3[X_AXIS]= get_float(line, 'I');
            }
            if (has_code(line,'D')) {
                pa.probePoint3[Y_AXIS]= get_float(line, 'D');
            }
#endif
            if (has_code(line,'I')) {
                pa.probeLeftPos = get_float(line, 'I');
            }
            if (has_code(line,'E')) {
                pa.probeRightPos = get_float(line, 'E');
            }
            if (has_code(line,'R')) {
                pa.probeFrontPos= get_float(line, 'R');
            }
            if (has_code(line,'P')) {
                pa.probeBackPos= get_float(line, 'P');
            }
            if (has_code(line,'W')) {
                pa.probeGridPoints= get_int(line, 'W');
            }



			if (pa.autoLeveling && (pa.probeDeviceType == PROBE_DEVICE_SERVO || pa.probeDeviceType == PROBE_DEVICE_Z_PIN 
										|| pa.probeDeviceType == PROBE_DEVICE_FSR)) {
				stepper_autoLevel_gpio_turn(true);
			} else if (pa.autoLeveling && pa.probeDeviceType == PROBE_DEVICE_PROXIMIRY) {
				stepper_autoLevel_gpio_turn(false);
			}
            
            break;

        case 914:
            /* M914: Set servo endstop angle, M914 S0 E90 */
            if (has_code(line,'S')) {
                pa.servo_endstop_angle[0] = get_int(line, 'S');
            }

            if (has_code(line,'E')) {
                pa.servo_endstop_angle[1] = get_int(line, 'E');
            }

            if (has_code(line,'A')) {
                pa.delta_deploy_start_location[0] = get_float(line, 'A');
            }
            if (has_code(line,'B')) {
                pa.delta_deploy_start_location[1] = get_float(line, 'B');
            }
            if (has_code(line,'C')) {
                pa.delta_deploy_start_location[2] = get_float(line, 'C');
            }

            if (has_code(line,'D')) {
                pa.delta_deploy_end_location[0] = get_float(line, 'D');
            }
            if (has_code(line,'X')) {
                pa.delta_deploy_end_location[1] = get_float(line, 'X');
            }
            if (has_code(line,'F')) {
                pa.delta_deploy_end_location[2] = get_float(line, 'F');
            }

            if (has_code(line,'G')) {
                pa.delta_retract_start_location[0] = get_float(line, 'G');
            }
            if (has_code(line,'H')) {
                pa.delta_retract_start_location[1] = get_float(line, 'H');
            }
            if (has_code(line,'I')) {
                pa.delta_retract_start_location[2] = get_float(line, 'I');
            }

            if (has_code(line,'J')) {
                pa.delta_retract_end_location[0] = get_float(line, 'J');
            }
            if (has_code(line,'K')) {
                pa.delta_retract_end_location[1] = get_float(line, 'K');
            }
            if (has_code(line,'L')) {
                pa.delta_retract_end_location[2] = get_float(line, 'L');
            }
            break;

        case 915:
            /* M915: Slow down percent*/
            if (has_code(line,'S')) {
                pa.slowdown_percent = get_int(line, 'S');
            }
			break;

        case 916:
            /* M916: */
            if (has_code(line,'S')) {
				//BBP1_EXTEND_FUNC_DUAL_Z  1
				//BBP1_EXTEND_FUNC_DUAL_EXTRUDER 2
				pa.bbp1_extend_func = get_int(line, 'S');
            }
            if (has_code(line,'T')) {
				//1 -> ext0, 2 -> ext1, 3 -> ext2
				pa.thermocouple_max6675_cnnection = 3; //get_int(line, 'T');
            }
            if (has_code(line,'W')) {
				//1 -> ext0, 2 -> ext1, 3 -> ext2
				pa.thermocouple_ad597_cnnection = 4; //get_int(line, 'W');
			}
			if (pa.thermocouple_max6675_cnnection == pa.thermocouple_ad597_cnnection &&
				pa.thermocouple_ad597_cnnection != 0) {
				printf("error !!!!!!!!!!!!!!!!!!!!!!!\n "); //Octoprint verify vaiue 
			}
            if (has_code(line,'L')) {
				pa.max_heat_pwm_hotend= get_int(line, 'L');
				if (pa.max_heat_pwm_hotend > 100) {
					pa.max_heat_pwm_hotend = 100;
				}
            }
            if (has_code(line,'R')) {
				pa.max_heat_pwm_bed= get_int(line, 'R');
				if (pa.max_heat_pwm_bed > 100) {
					pa.max_heat_pwm_bed = 100;
				}
            }
			/*
				"1","THERMISTOR_1",
	    		"2","THERMISTOR_2",
	    		"3","Thermocouple_MAX6675",
	    		"4","Thermocouple_AD597",
			*/
			if (has_code(line,'X')) {
				pa.measure_ext1 = get_int(line, 'X');
            }
			if (has_code(line,'Y')) {
				pa.measure_ext2 = get_int(line, 'Y');
            }
			if (has_code(line,'Z')) {
				pa.measure_ext3 = get_int(line, 'Z');
            }

			if (bbp_board_type == BOARD_BBP1S) {
				heater_reconfig();
			}

			break;
			case 1009: //M1009
			{
				int gpio = -1;
				int output = -1;
				int level = -1;
				int show = -1;

				if (has_code(line,'G')) {
					gpio = get_int(line, 'G');
				}
				if (has_code(line,'O')) {
					output = get_int(line, 'O');
				}
				if (has_code(line,'L')) {
					level = get_int(line, 'L');
				}
				if (has_code(line,'S')) {
					show  = 1;
				}

				if (gpio > 0 && output == 0) { //input
						set_gpio(gpio, 0, 0);
				} else if (gpio > 0 && output == 1){
					if (level == 1){
						set_gpio(gpio, 1, 1);
					} else if (level == 0){
						set_gpio(gpio, 1, 0);
					}
				}
				if (gpio > 0 && show == 1) {
					int ret = read_gpio(gpio);
					memset(buf, 0, sizeof(buf));
					sprintf(buf, "GPIO number:%d,level=%d", gpio, ret);
					gcode_send_response_remote((char *)&buf);
				}
			}
			break;

        default:
            printf("[Warning: not support M%d command!\n", value);
            break;
    }

    if (unicorn_get_mode() == FW_MODE_REMOTE && send_ok) {
        gcode_send_response_remote("ok\n");
    }

    return SEND_REPLY;
}

static int gcode_process_t(char *line, int t, bool send_ok)
{
    float next_feedrate;
	bool make_move = false;

	if (has_code(line,'F')) {
		make_move = true;
		next_feedrate = get_float(line, 'F');
		if(next_feedrate > 0.0) {
			feedrate = next_feedrate;
		}
	}

    if (unicorn_get_mode() == FW_MODE_REMOTE && send_ok) {
        gcode_send_response_remote("ok\n");
    }

	tmp_extruder = t; 
	if (pa.ext_count > 1){
		if(tmp_extruder != active_extruder) {
			// Save current position to return to after applying extruder offset
			memcpy(destination, current_position, sizeof(destination));
			#ifdef DUAL_X_CARRIAGE
			#else
			// Offset extruder (only by XY)
			int i;
			for(i = 0; i < 2; i++) {
				current_position[i] = current_position[i] -
							pa.ext_offset[i][active_extruder] +
							pa.ext_offset[i][tmp_extruder];
			}
			// Set the new active extruder and position
			active_extruder = tmp_extruder;
			#endif 

			if (pa.machine_type == MACHINE_DELTA) {
				calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
				//sent position to plan_set_position();
				plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],current_position[E_AXIS]);
			} else {
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			}
			// Move to the old position if 'F' was in the parameters
			if(make_move) {
				prepare_move();
			}
		}
	}
    return SEND_REPLY;
}

//
//parser.buffer[strlen(parser.buffer) - 1] = 0; 
//last byte is 0
static char g_buf_line[1024] = {0};
int gcode_process_multi_line(char *multi_line)
{
    char *line_start = NULL;
    char *line_end = NULL;
    char *pos;

    //printf("multi line:%s\n", multi_line);
    if (multi_line == NULL){
        return -1;
    }

    line_start = multi_line; 
    pos = multi_line;
    while (true) {
        if(*pos == '\n' || *pos == 0){
            line_end = pos;
            memset(g_buf_line, 0, sizeof(g_buf_line));
            strncpy(g_buf_line, line_start, line_end-line_start + 1);
     //       printf("found a line:%s\n", g_buf_line);
            gcode_process_line(g_buf_line, true);
            line_start = pos + 1;
            if(*pos == 0){
          //      printf("exit multi line\n");
                break;
            }
        }
        pos++;
    }
    return 0;
}

// line with '\n'
//int gcode_process_line(char *line, bool send_ok)
int gcode_process_line(char *buf_line, bool send_ok)
{
    int val;
   // char buf_line[1024] = {0};

    if(buf_line == NULL){
        printf("null line\n");
        return -1;
    }

    //strcpy(buf_line, line);

 //   printf("process a line:%s\n", buf_line);

    char command = get_command(buf_line);
	if (command == 0) {
		return 0;
	}
    switch (command) {
        case 'G':
            val = get_int(buf_line, 'G');
            if (stepper_is_stop()) {
                printf("gcode: G%d, stepper is stop. !!!!, please send M80.\n", val);
                gcode_send_response_remote("ok\n");
                gcode_send_response_remote("Stepper is stop! Please click \"Motor on\" button on Control pannel\n");
            } else {
				#if 0
				if (has_code(buf_line, 'V')) {
					gcode_send_response_remote("ok\n");
					remove_str(buf_line, "V99999.0");
					append_list(buf_line, ++mcode_index);
					put_mcode_to_fifo();
					break;
				}
				#endif
                if (gcode_process_g(buf_line, val, send_ok) == NO_REPLY) {
                    printf("gcode_process_g [G%d] NO_REPLY\n", val);
                }
            }
            break;

        case 'M':
            val = get_int(buf_line, 'M');
            if (has_code(buf_line, 'V') && !stepper_is_stop()) {
                gcode_send_response_remote("ok\n");
                remove_str(buf_line, "V99999.0");
                append_list(buf_line, ++mcode_index);
				put_mcode_to_fifo();
                break;
            } else if (has_code(buf_line, 'V') && stepper_is_stop()) {
                printf("gcode: M%d with V, stepper is stop. !!!!, don't process.\n", val);
                gcode_send_response_remote("ok\n");
                break;
			}
            if (stepper_is_stop() && (val == 17 || val == 18 || val == 84 || val == 280 
                        || val == 400 || val == 401 || val == 402 || val == 600 || val == 601)) {
                printf("gcode: M%d, stepper is stop. !!!!, please send M80.\n", val);
                gcode_send_response_remote("ok\n");
                gcode_send_response_remote("Stepper is stop! Please click \"Motor on\" button on Control pannel\n");
            } else {
                if (gcode_process_m(buf_line, val, send_ok) == NO_REPLY) {
                    printf("gcode_process_m [M%d] NO_REPLY\n", val);
                }
            }
            break;

        case 'T':
            val = get_int(buf_line, 'T');
            if (gcode_process_t(buf_line, val, send_ok) == NO_REPLY) {
                printf("gcode_process_t [T%d] NO_REPLY\n", val);
            }
            break;

        default:
            printf("[Warning: not support not 'G,M,T' command=%c!\n", command);
            break;
    }
    return 0;
}

int gcode_process_line_from_file()
{
     return gcode_process_multi_line(parser.buffer);
}

void gcode_set_feed(int multiply)
{
    if (multiply >= 10 && multiply < 2000) {
        feedmultiply = multiply;
		//COMM_DBG("Speed -> %d\n", feedmultiply);
	    printf("Speed -> %d\n", feedmultiply);
    } else {
        feedmultiply = 2000;
		COMM_DBG("max multiply %d\n", feedmultiply);
    }
}

void gcode_set_extruder_feed(int multiply)
{
    if (multiply >= 10 && multiply < 2000) {
        extrudemultiply = multiply;
	    printf("Speed -> %d\n", extrudemultiply);
    } else {
        extrudemultiply = 2000;
		COMM_DBG("max multiply %d\n", extrudemultiply);
    }
}

void *gcode_get_line_from_file(FILE *fp)
{
    return fgets((char *)&parser.buffer, sizeof(parser.buffer), fp);
}

int gcode_get_line_from_remote(void)
{
    int ret;
    memset(&parser.buffer, 0, sizeof(parser.buffer));
    ret = read(parser.fd_rd, &parser.buffer, sizeof(parser.buffer));
	GCODE_DBG("[gcode] read: %s\n", (char *)&parser.buffer);

    return ret;
}

int gcode_send_response_remote(char *str)
{
    int ret = -1;
	if((parser.fd_rd > 0) && (parser.fd_wr > 0)){
    	ret = write(parser.fd_wr, str, strlen(str));
    	GCODE_DBG("[respond]: %s\n", str); 
	} else {
    	printf("!!!,failed to response, remote is close\n"); 
	}
    return ret;
}

int gcode_send_byte_response_remote(char *str, int size)
{
    int ret = -1;
	if((parser.fd_rd > 0) && (parser.fd_wr > 0)){
    	ret = write(parser.fd_wr, str, size);
    	GCODE_DBG("[respond]: %s\n", str); 
	} else {
    	printf("!!!,failed to response, remote is close\n"); 
	}
    return ret;
}

extern int read_fd;
extern int write_fd;
extern int read_emerg_fd;
extern Pause_Handle hPause_printing;

extern struct queue *pru_queue;
static void *mcode_thread_worker(void *arg)
{
    struct M_list *item;

    printf("start mcode thread\n");
    while (!stop) {
        //prussdrv_pru_wait_event(PRU_EVTOUT_1);
        //prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
        item = get_list_item();
        if (item) {
            if ((item->no <= pru_queue->mcode_count) && (item->no != 0)){
				char buf[1024] = {0};
        		COMM_DBG("exec mcode:%s, number:%d pru_code:%d \n", item->MCode, item->no, pru_queue->mcode_count);
				strcpy(buf, item->MCode);
                del_list_item(item);
                gcode_process_line(buf, false);
            }
        } 

       	usleep(100000);
        //usleep(1);
    }
    printf("exit mcode thread\n");
	return NULL;
}

static unsigned char buf_bad_line[80] = {0};
static unsigned int bad_line_len = 0;
static bool has_bad_line = false;

static void *gcode_thread_worker(void *arg)
{
    int ret = 1;
    struct timeval timeout;
	fd_set fds;

    if (hPause_printing) {
        Pause_test(hPause_printing);
    }

    while (!stop)
    {
        /* TODO: need to check socket or file available */
        if (unicorn_get_mode() == FW_MODE_REMOTE) {

			memset(&parser.buffer, 0, sizeof(parser.buffer));

			int rc;
			int max_fd;
			FD_ZERO(&fds);
			FD_SET(parser.fd_rd,  &fds);
			max_fd = parser.fd_rd;

			timeout.tv_sec = 2;
			timeout.tv_usec = 0;

			while(!stop){
					rc = select(max_fd + 1, &fds, 0, 0, &timeout);
					if (rc < 0) {
						if(errno == EINTR){
							printf("select errno == EINTR\n");
							continue;
						}
						printf("select err, break \n");
						break;
					} else if (rc >0 && FD_ISSET(parser.fd_rd, &fds)) {
						if (has_bad_line) {
							memcpy((char *)parser.buffer, (const char *)buf_bad_line, bad_line_len);
							has_bad_line = false;
							ret = read(parser.fd_rd, parser.buffer + bad_line_len, BUFFER_SIZE - bad_line_len);
							ret = ret + bad_line_len;
							#if 0
							printf("lkj, has_bad_line ret + bad_line_len =%d\n", ret);
							int i = 0;
							for (i=0; i<ret; i++) {
								printf("%c", parser.buffer[i]);
							}
							printf("\n");
							#endif
						} else {
							ret = read(parser.fd_rd, &parser.buffer, BUFFER_SIZE);
						}
						GCODE_DBG("[gcode]: ret=%d, read buf:%s\n", ret, (char *)parser.buffer);
						if (ret == BUFFER_SIZE) { //
                        	if (parser.buffer[BUFFER_SIZE - 1] == '\n') {
								GCODE_DBG("full buffer, but get right buffer\n");
							} else {
								char *last_bad_line = NULL;

								has_bad_line = true;
								last_bad_line = strrchr(parser.buffer, '\n');
								last_bad_line++ ;
								//printf("----->last_bad_line:%s-End\n", last_bad_line);

								bad_line_len = BUFFER_SIZE - (last_bad_line - (char *)parser.buffer);
								memset(buf_bad_line, 0, sizeof(buf_bad_line));
								strncpy((char *)buf_bad_line, last_bad_line, bad_line_len);

								memset(last_bad_line, 0, bad_line_len);
								GCODE_DBG("----->copy buf_bad_line :%s-End\n", buf_bad_line);
							}
						}

                        parser.buffer[BUFFER_SIZE - 1] = 0; 

						if (ret <= 0) {
							printf("[gcode]: remote read err\n");
							stop = true;
						} else if(!strncmp(parser.buffer, "exit", 4)){
							printf("[gcode]: remote exit \n");
							stop = true;
						}
						break;
					} else {
						//printf("select gcode timeout\n");
						FD_ZERO(&fds);
						FD_SET(parser.fd_rd,  &fds);
						max_fd = parser.fd_rd;
						timeout.tv_sec = 0;
						timeout.tv_usec = 10000;
						continue;
					}
			}
		}
	    gcode_process_multi_line(parser.buffer);
	}

	if (unicorn_get_mode() == FW_MODE_REMOTE) {
		if(parser.fd_rd > 0){
			close(parser.fd_rd);
			parser.fd_rd = 0;
		}
		if(parser.fd_wr > 0){
			close(parser.fd_wr);
			parser.fd_wr = 0;
		}

		read_fd = 0;
		write_fd = 0;

		unicorn_disconnect_octoprint();
        //FIXME
		//unicorn_stop(false);
	}
    
    thread_started = false;
    COMM_DBG("Leaving gcode thread!\n");
    //pthread_exit(NULL);
	return NULL;
}


static void *emerg_gcode_thread_worker(void *arg)
{
    int ret = 1;
    struct timeval timeout;
	fd_set fds;
	unsigned char buf_emerg[150];

    if (hPause_printing) {
        Pause_test(hPause_printing);
    }

    while (!stop)
    {
        /* TODO: need to check socket or file available */
        if (unicorn_get_mode() == FW_MODE_REMOTE) {
			memset(buf_emerg, 0, sizeof(buf_emerg));

			int rc;
			int max_fd;
			FD_ZERO(&fds);
			FD_SET(parser.fd_rd_emerg,  &fds);
			max_fd = parser.fd_rd_emerg;

			timeout.tv_sec = 2;
			timeout.tv_usec = 0;

			while(!stop){
					rc = select(max_fd + 1, &fds, 0, 0, &timeout);
					if (rc < 0) {
						if(errno == EINTR){
							printf("select errno == EINTR\n");
							continue;
						}
						printf("select err, break \n");
						break;
					} else if (rc >0 && FD_ISSET(parser.fd_rd_emerg, &fds)) {
							ret = read(parser.fd_rd_emerg, buf_emerg, sizeof(buf_emerg));
							printf("[gcode emergency]: ret=%d, read buf:%s\n", ret, (char *)buf_emerg);
							break;
					} else {
						//printf("select gcode timeout\n");
						FD_ZERO(&fds);
						FD_SET(parser.fd_rd_emerg,  &fds);
						max_fd = parser.fd_rd_emerg;
						timeout.tv_sec = 0;
						timeout.tv_usec = 10000;
						continue;
					}
			}
		}
		if (strlen((const char *)buf_emerg) > 0)
        	gcode_process_multi_line((char *)buf_emerg);
	}

	if (unicorn_get_mode() == FW_MODE_REMOTE) {
		if(parser.fd_rd_emerg > 0){
			close(parser.fd_rd_emerg);
			parser.fd_rd_emerg  = 0;
		}

		read_emerg_fd = 0;
	}
    
    COMM_DBG("Leaving emerg gcode thread!\n");
	return NULL;
}

/*
 * Start gcode parser
 */
int gcode_start(int fd_rd, int fd_wr, int fd_rd_emerg)
{
    int ret, i;
    pthread_attr_t attr;
    //struct sched_param sched;

    if (stop) {
        stop = false;
    }

	for (i = 0; i < NUM_AXIS; i++) {
		destination[i] = 0.0;
		current_position[i] = 0.0;
	}

	if (pa.machine_type == MACHINE_DELTA) {
		destination[X_AXIS] = 0.0; 
		destination[Y_AXIS] = 0.0; 
		if((stepper_check_lmsw(X_AXIS) && stepper_check_lmsw(Y_AXIS) 
			 && stepper_check_lmsw(Z_AXIS))){
			destination[Z_AXIS] = pa.z_home_pos;
		} else {
			destination[Z_AXIS] = 0.0;
		}
		current_position[X_AXIS] = destination[X_AXIS];
		current_position[Y_AXIS] = destination[Y_AXIS];
		current_position[Z_AXIS] = destination[Z_AXIS];

		calculate_delta(current_position);

		plan_set_position(delta[X_AXIS], 
				delta[Y_AXIS], 
				delta[Z_AXIS], 
				current_position[E_AXIS]);

		/* Set the current position for PRU */
		stepper_set_position(lround(delta[X_AXIS] * pa.axis_steps_per_unit[X_AXIS]),
				lround(delta[Y_AXIS] * pa.axis_steps_per_unit[Y_AXIS]),
				lround(delta[Z_AXIS] * pa.axis_steps_per_unit[Z_AXIS]),
				lround(current_position[E_AXIS] * pa.axis_steps_per_unit[E_AXIS]));
	}

    counter = 0;

    if (thread_started) {
		return 0;
    }

    memset(&parser, 0, sizeof(parser_t));

    if (unicorn_get_mode() == FW_MODE_REMOTE) {
        if (pthread_attr_init(&attr)) {
            return -1;
        }

		#if 0 //for android
        if (pthread_attr_setschedpolicy(&attr, PTHREAD_EXPLICIT_SCHED)) {
            return -1;
        }
        sched.sched_priority = sched_get_priority_max(SCHED_FIFO);

        if (pthread_attr_setschedparam(&attr, &sched)) {
            return -1;
        }
		#endif

        COMM_DBG("--- Creating gcode_thread..."); 
        ret = pthread_create(&gcode_thread, &attr, gcode_thread_worker, NULL);
        if (ret) {
            printf("create gcode thread failed with ret %d\n", ret);
            return -1;
        } else {
            COMM_DBG("done ---\n");
        }
        COMM_DBG("--- Creating emerg_gcode_thread..."); 
        ret = pthread_create(&emerg_gcode_thread, &attr, emerg_gcode_thread_worker, NULL);
        if (ret) {
            printf("create emerg gcode thread failed with ret %d\n", ret);
            return -1;
        } else {
            COMM_DBG("done ---\n");
        }

        COMM_DBG("--- Creating mcode_thread..."); 
        ret = pthread_create(&mcode_thread, NULL, mcode_thread_worker, NULL);
        if (ret) {
            printf("create mcode thread failed with ret %d\n", ret);
            return -1;
        } else {
            COMM_DBG("done ---\n");
        }
    }

    if (unicorn_get_mode() == FW_MODE_REMOTE) {
        if (fd_rd && fd_wr) {
            parser.fd_rd = fd_rd;
            parser.fd_wr = fd_wr;
            parser.fd_rd_emerg = fd_rd_emerg;
        } else {
            return -1;
        }
    }

    thread_started = true;

    return 0;
}
/*
 * Stop gcode parser
 */
void gcode_stop(void)
{
	int i;
    feedrate = 1800;

	for (i = 0; i < NUM_AXIS; i++) {
		destination[i] = 0.0;
		current_position[i] = 0.0;
	}
}
/*
 * Init gcode parse
 */
int gcode_init(void)
{
    init_MCode_list();
    return 0;
}
/*
 * Delete gcode parser
 */
void gcode_exit(void)
{
    GCODE_DBG("gcode_exit called.\n");
    if (!stop) {
        stop = true;
    }

    thread_started = false;

    if (unicorn_get_mode() == FW_MODE_REMOTE) {
        GCODE_DBG("waiting gcode to quit\n");
        pthread_join(gcode_thread, NULL);
        pthread_join(emerg_gcode_thread, NULL);
        pthread_join(mcode_thread, NULL);
        GCODE_DBG("ok\n");
    }

    destroy_MCode_list();
    GCODE_DBG("gcode_exit ok\n");
}
