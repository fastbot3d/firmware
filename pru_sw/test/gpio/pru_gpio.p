//=============================================================================
//  PRU Example to demonstrate toggling direct connected GPIOs (through R30)
//=============================================================================
.origin 0
.entrypoint MAIN

#include "pru_gpio.hp"

#define PRU_GPO   5

#define GPIO_PORT 0
#define GPIO_NR   27

#define COUNT     0xFFFFF
#define DELAY     10000

#define PRU0_ARM_INTERRUPT      19

MAIN:    
    MOV delay.cnt, COUNT
    MOV delay.ns,  DELAY

    LBCO r0, C4, 4, 4                    // Clear STANDBY_INIT bit
    CLR  r0, r0, 4
    SBCO r0, C4, 4, 4 

LOOP_M:
    //pru_gpio_set PRU_GPO               // Set gpio high
    gpio_set GPIO_PORT, GPIO_NR

    delay_ns delay.ns                    // High level pluse
    
    //pru_gpio_clr PRU_GPO               // Set gpio low
    gpio_clr GPIO_PORT, GPIO_NR        

    delay_ns delay.ns                    // Low level pluse

    SUB delay.cnt, delay.cnt, 1          // Loop count
    QBNE LOOP_M, delay.cnt, 0

    MOV R31.b0, PRU0_ARM_INTERRUPT + 16  // Send notification to Host for program completion
    HALT                                 // PRU Halt
