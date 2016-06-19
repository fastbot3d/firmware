/*
 * Unicorn 3D Printer Firmware
 * temp.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "analog.h"
#include "temp.h"

typedef struct {
    channel_tag id;
    channel_tag analog_input;
    double      value;
    double      setpoint;    
    double      rang_low;
    double      rang_high;
    int         out_of_range;
    int         in_range_time;
    temp_convert_f *convert;
} temp_t;

static temp_t *temps = NULL;
static int *last_analog = NULL;
static int last_analog_failed = 0;
static unsigned int nr_temps = 0;

static int temp_index_lookup(channel_tag temp_ch)
{
    int idx;

    for (idx = 0; idx < nr_temps; idx++) {
        if (temps[idx].id == temp_ch || !strcmp(temps[idx].id,temp_ch)) {
            return idx;
        }
    }

	TEMP_DBG("temp_index_lookup failed for '%s'\n",
			tag_name(temp_ch));
	return -1;
}

channel_tag temp_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_temps; idx++) {
        channel_tag tag = temps[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

int temp_config(temp_config_t *pcfgs, int nr_cfgs)
{
    int i;

    TEMP_DBG("temp_config called with %d records\n", nr_cfgs);

    temps = calloc(nr_cfgs, sizeof(temp_t));
    if (!temps) {
        return -1;
    }
    nr_temps = 0;

    last_analog = calloc(nr_cfgs, sizeof(int));
    if (!last_analog ) {
        return -1;
    }

    for (i = 0; i < nr_cfgs; i++) { 
        temp_t *pd        = &temps[i];
        temp_config_t *ps = &pcfgs[i];

        pd->id            = ps->tag;
        pd->analog_input  = ps->analog_input;
        pd->convert       = ps->convert;
        pd->in_range_time = ps->in_range_time;

        nr_temps++;
    }

    return 0;
}

int temp_init(void)
{
    TEMP_DBG("temp_init called.\n");
    
    if (!temps || nr_temps <= 0) {
        return -1;
    }

    return 0;
}

void temp_exit(void)
{
    TEMP_DBG("temp_exit called.\n");

    if (temps) {
        free(temps);
    }
}

static int temp_update(channel_tag temp_ch)
{
    int ret;
    int analog;
    double celsius;
    int idx = 0;

    idx = temp_index_lookup(temp_ch);
    if (idx < 0) { 
        return -1;
    }

    temp_t *p = &temps[idx];
    
    ret = analog_get_input(p->analog_input, &analog);
    if ((last_analog_failed <= 30) && (ret < 0 || (analog > 4096))) {  //0xFFF
        printf("Get analog input failed\n");
		analog = last_analog[idx];
		last_analog_failed++;
        printf("analog <0, idx=%d, %d \n", idx, analog);
        //return -1;
    } else if (last_analog_failed > 30) { 
        printf("last_analog_failed > 50 \n");
        return -1;
    } else {
		last_analog[idx] = analog;
		last_analog_failed = 0;
	}

    if (p->convert) {
        ret = p->convert(analog, &celsius);
        if (ret) {
            TEMP_DBG("failed to convert %s\n", temp_ch);
        	return -1;
        } else {
            p->value = celsius;
        }
    } else {
        fprintf(stderr, "convert func not registered\n");
        return -1;
    }
    
    TEMP_DBG("analog %d, celsius %f\n", analog, celsius);

    if (p->rang_low <= celsius && celsius <= p->rang_high) {
        if (p->out_of_range > 0) {
            p->out_of_range--;
			if (p->out_of_range == 0) {
				TEMP_DBG("temperature for %s has stabilized!\n", tag_name(temp_ch));
			}
        }
    } else {
        if (p->out_of_range == 0) { 
            TEMP_DBG("temperature for %s is out of range!\n", tag_name(temp_ch));
        }
        p->out_of_range = p->in_range_time;
    }

    return 0;
}

static pthread_mutex_t lock;
int temp_get_celsius(channel_tag temp_ch, double *value)
{
    int idx = 0;
    
    if (!value) {
        return -1;
    }

   // pthread_mutex_lock(&lock);
    idx = temp_index_lookup(temp_ch);
    if (idx < 0) {
    	pthread_mutex_unlock(&lock);
        return -1;
    }

    if (temp_update(temp_ch) < 0){
  //  	pthread_mutex_unlock(&lock);
        return -1;
	}

    *value = temps[idx].value;

 //   pthread_mutex_unlock(&lock);
    return 0;
}

int temp_set_setpoint(channel_tag temp_ch, double setpoint, double delta_low, double delta_high)
{
    int idx = 0;
    temp_t *p = NULL; 

    idx = temp_index_lookup(temp_ch);
    if (idx < 0) {
        printf("look up %s failed\n", temp_ch);
        return -1;
    }
    
    p = &temps[idx];
    
    p->setpoint  = setpoint;
    p->rang_low  = setpoint + delta_low;
    p->rang_high = setpoint + delta_high; 

    p->out_of_range = p->in_range_time;
    return 0;
}

int temp_achieved(channel_tag temp_ch)
{
    int idx = 0;
    temp_t *p = NULL; 

    idx = temp_index_lookup(temp_ch);
    if (idx < 0) {
        printf("look up %s failed\n", temp_ch);
        return -1;
    }
    
    p = &temps[idx];

    TEMP_DBG("out_of_range %d\n", temps[idx].out_of_range);
    return p->out_of_range == 0;
}

int temp_all_zero(void)
{
    int idx;
    
    for (idx = 0; idx < nr_temps; idx++) {
        if (temps[idx].setpoint != 0.0) {
            return 0;
        }
    }

    return 1;
}

