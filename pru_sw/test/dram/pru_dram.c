/*
 * pru_dram.c
 *
 * The PRU reads and stores values into the PRU Data RAM memory. PRU Data RAM
 * memory has an address in both the local data memory map and global memory
 * map. The example accesses the local Data RAM using both the local address
 * through a register pointed base address and the global address pointed by
 * entries in the constant table.
 */
#include <stdio.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM 	0
#define ADDEND1		0x0010F012u
#define ADDEND2		0x0000567Au

#define AM33XX

static void *pruDataMem;
static unsigned int *pruDataMem_int;

static int LOCAL_exampleInit ( unsigned short pruNum )
{
    //Initialize pointer to PRU data memory
    if (pruNum == 0) {
        prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);
    } else if (pruNum == 1) {
        prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &pruDataMem);
    }
    pruDataMem_int = (unsigned int*) pruDataMem;

    // Flush the values in the PRU data memory locations
    pruDataMem_int[1] = 0x00;
    pruDataMem_int[2] = 0x00;

    return(0);
}

static unsigned short LOCAL_examplePassed ( unsigned short pruNum )
{
  return ((pruDataMem_int[1] ==  ADDEND1) & (pruDataMem_int[2] ==  ADDEND1 + ADDEND2));
}

int main (void)
{
    unsigned int ret;
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    printf("\nINFO: Starting %s example.\r\n", "PRU_memAccessPRUDataRam");
    /* Initialize the PRU */
    prussdrv_init();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret) {
        printf("prussdrv_open open failed\n");
        return (ret);
    }

    /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

    /* Initialize example */
    printf("\tINFO: Initializing example.\r\n");
    LOCAL_exampleInit(PRU_NUM);

    /* Execute example on PRU */
    printf("\tINFO: Executing example.\r\n");
    prussdrv_exec_program(PRU_NUM, "./pru_dram.bin");

    /* Wait until PRU0 has finished execution */
    printf("\tINFO: Waiting for HALT command.\r\n");
    prussdrv_pru_wait_event(PRU_EVTOUT_0);
    printf("\tINFO: PRU completed transfer.\r\n");
    prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

    /* Check if example passed */
    if (LOCAL_examplePassed(PRU_NUM)) {
        printf("INFO: Example executed succesfully.\r\n");
    } else {
        printf("INFO: Example failed.\r\n");
    }

    /* Disable PRU and close memory mapping*/
    prussdrv_pru_disable (PRU_NUM);
    prussdrv_exit ();

    return(0);
}
