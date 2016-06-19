/*
 * Unicorn 3D Printer Firmware
 * pruss.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "parameter.h"
#include "pruss.h"
#include "common.h"
#include "eeprom.h"

#define LOAD_PRU_BIN
#define PRU_BIN_PATH "/.octoprint/pruss_unicorn.bin"

//#ifndef LOAD_PRU_BIN
#include "./build/target/bin/bbp1_bin.h"
#include "./build/target/bin/bbp1s_bin.h"
//#endif


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
#if 0
	ret = prussdrv_open(PRU_EVTOUT_1);
    if (ret) {
        printf("prussdrv_open evout_1 failed\n");
        return ret;
    }
#endif
    prussdrv_pruintc_init(&intc_initdata);    
    
    /* Clean up DRAM */
#if 0
    prussdrv_pru_clear_memory(PRUSS0_PRU0_DATARAM, 8192);
#else
    prussdrv_pru_clear_memory(PRUSS0_SHARED_DATARAM, 12288);
#endif

	eeprom_read_pru_code(EEPROM_DEV, 0, PRU_BIN_PATH);

	unsigned long pru_checksum = calculate_pru_file_crc(PRU_BIN_PATH);
	COMM_DBG("parameter crc:%lu, eeprom pru crc:%lu \n", 
            pa.pru_checksum, pru_checksum);

	if ((pa.pru_checksum == pru_checksum) && (pru_checksum != 0)) {
		printf("PRU loading from eprom:%s \n", PRU_BIN_PATH);
		prussdrv_exec_program(PRU_NUM, PRU_BIN_PATH);
	} else {
    	printf("error,%s not exist, PRU loading default config \n", PRU_BIN_PATH);

		if(bbp_board_type == BOARD_BBP1){
			prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, 0, BBP1_array, sizeof(BBP1_array));
		} else if(bbp_board_type == BOARD_BBP1S){
			prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, 0, BBP1S_array, sizeof(BBP1S_array));
		}
	}

#if 0
#ifdef LOAD_PRU_BIN
    fprintf(stderr, "PRU loading code:%s \n", PRU_BIN_PATH);
    //prussdrv_exec_program(PRU_NUM, "./pruss_unicorn.bin");
    prussdrv_exec_program(PRU_NUM, PRU_BIN_PATH);
#else
    prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, 0, PRUcode, sizeof(PRUcode));
    //prussdrv_pru_enable(PRU_NUM);
#endif
#endif

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
