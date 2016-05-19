/*
 * Unicron 3D Printer Firmware
 * main.c
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
