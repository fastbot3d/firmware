/*
 * Unicorn 3D Printer Firmware
 * pruss.c
 * PRUSS driver interface
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

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "pruss.h"

#include "pruss_unicorn.h"

/*
 * pruss drver interface
 */
int pruss_init(void)
{
    int ret;
    tpruss_intc_initdata intc_initdata = PRUSS_INTC_INITDATA; 
    
    prussdrv_init();
    
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret) {
        printf("prussdrv_open failed\n");
        return ret;
    }
    prussdrv_pruintc_init(&intc_initdata);    
    
    /* Clean up DRAM */
    prussdrv_pru_clear_memory(PRUSS0_PRU0_DATARAM, 8192);

    prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, 0, PRUcode, sizeof(PRUcode));

    return 0;
}

void pruss_exit(void)
{
    prussdrv_pru_disable(PRU_NUM);
    prussdrv_exit();
}

int pruss_reset(void)
{
    return prussdrv_pru_reset(PRU_NUM);
}

int pruss_disable(void)
{
    return prussdrv_pru_disable(PRU_NUM);
}

int pruss_enable(void)
{
    return prussdrv_pru_enable(PRU_NUM);
}
