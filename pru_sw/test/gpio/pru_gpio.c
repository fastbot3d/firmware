/*--------------------------------------------------------------------
 * pru_gpio.c
 * This example code demonstrates the PRU toggling GPIO pins.
 --------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define  PRU_NUM        (0)

int main(int argc, char **argv)
{
    int ret;
    tpruss_intc_initdata intc_initdata = PRUSS_INTC_INITDATA; 
    
    printf("\nStarting %s example.\r\n", "pru_gpio");
    prussdrv_init();
    
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret) {
        printf("prussdrv_open failed\n");
        return ret;
    }
    prussdrv_pruintc_init(&intc_initdata);    
    
    printf("PRU loading code : pru_gpio.bin\n");
    prussdrv_exec_program(PRU_NUM, "./pru_gpio.bin");
    
    printf("Waiting for HALT command.\r\n");
    prussdrv_pru_wait_event(PRU_EVTOUT_0);

    printf("PRU completed.\r\n"); 
    prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

    printf("Disable PRU and close memory mapping\n");
    prussdrv_pru_disable (PRU_NUM);
    prussdrv_exit ();

    return 0;
}

