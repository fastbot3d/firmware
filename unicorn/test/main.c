/*
 * Unicron 3D Printer Firmware
 * main.c
 * Test routine for unicron
 * Copyright (c) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <errno.h>

#include <unicorn/printer.h>


int main(int argc, char *argv[])
{
    fprintf(stderr, "Unicron 3D printer firmware\n");
    
    printer_init();
    
    sleep(10);

    printer_exit();

    return 0;
}
