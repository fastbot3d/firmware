/*
 * Unicorn 3D Printer Firmware
 * analog.c 
 * Analog input measure and output adjustment interface
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "analog.h"

typedef struct {
    channel_tag  id;
    channel_tag  device_path;
    unsigned int type;
} analog_t;

static analog_t *analogs = NULL;
static unsigned int nr_analogs = 0;

static int analog_index_lookup(channel_tag analog_ch)
{
    int idx;

    for (idx = 0; idx < nr_analogs; idx++) {
        if (analogs[idx].id == analog_ch) {
            return idx;
        }
    }

    if (DBG(D_ANALOG)) {
        fprintf(stderr, "analog_index_lookup failed for '%s'\n",
                tag_name(analog_ch));
    }
    return -1;
}

channel_tag analog_lookup_by_name(const char *name)
{
    int idx;
    
    for (idx = 0; idx < nr_analogs; idx++) {
        channel_tag tag = analogs[idx].id;
        if (strcmp(tag_name(tag), name) == 0) {
            return tag;
        }
    }

    return NULL;
}

int analog_config(analog_config_t *pcfgs, int nr_cfgs)
{
    int i;

    if (DBG(D_ANALOG)) {
        printf("analog_config called with %d records\n", nr_cfgs);
    }
    
    if (!pcfgs || nr_cfgs <= 0) {
        return -1;
    }

    analogs = calloc(nr_cfgs, sizeof(analog_t));
    if (!analogs) {
        return -1;
    }
    nr_analogs = 0;

    for (i = 0; i < nr_cfgs; i++) {
        analog_t *pd        = &analogs[i]; 
        analog_config_t *ps = &pcfgs[i];
        
        pd->id          = ps->tag;
        pd->device_path = ps->device_path;
        pd->type        = ps->type;

        nr_analogs++;
    }

    return 0;
}

int analog_init(void)
{
    if (DBG(D_ANALOG)) {
        printf("analog_init called.\n");
    }

    if (!analogs || nr_analogs <= 0) {
        return -1;
    }

    return 0;
}

void analog_exit(void)
{
    if (DBG(D_ANALOG)) {
        printf("analog_exit called.\n");
    }

    if (analogs) {
        free(analogs);
    }
}

int analog_get_input(channel_tag ain, int *vol_uv)
{
    int idx = -1;
    analog_t *pd = NULL;
    
    if (DBG(D_ANALOG)) {
        printf("analog_get_input called.\n");
    }

    idx = analog_index_lookup(ain);
    if (idx < 0) { 
        return -1;
    }
    
    pd = &analogs[idx];

    if (pd->type == ANALOG_TYPE_IN) { 
#ifndef HOST
        analog_read_sysfs(pd->device_path, vol_uv);
#endif
    } else {
        return -1;
    }

    return 0;
}

int analog_set_output(channel_tag aout, int vol_uv)
{
    int idx = -1;
    analog_t *pd = NULL;
    
    if (DBG(D_ANALOG)) {
        printf("analog_set_output called.\n");
    }

    idx = analog_index_lookup(aout);
    if (idx < 0) { 
        printf("analog idx err......\n");
        return -1;
    }
    
    pd = &analogs[idx];
    if (pd->type == ANALOG_TYPE_OUT) { 
#ifndef HOST
        analog_write_sysfs(pd->device_path, vol_uv);
#endif
    } else {
        printf("analog type not output\n");
        return -1;
    }
    return 0;
}

