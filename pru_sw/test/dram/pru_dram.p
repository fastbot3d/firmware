//------------------------------------------------------------------------------
// file:   pru_dram.p
// brief:  PRU access of internal Data Ram.
//------------------------------------------------------------------------------
.origin 0
.entrypoint MEMACCESSPRUDATARAM

#include "pru_dram.hp"

MEMACCESSPRUDATARAM:

#ifdef AM33XX
    // Configure the block index register for PRU0 by setting c24_blk_index[7:0] and
    // c25_blk_index[7:0] field to 0x00 and 0x00, respectively.  
    // This will make C24 point to 0x00000000 (PRU0 DRAM) and C25 point 
    // to 0x00002000 (PRU1 DRAM).
    MOV       r0, 0x00000000
    MOV       r1, CTBIR_0
    ST32      r0, r1
#endif

    //Load 32 bit value in r1
    MOV       r1, 0x0010f012

    //Load address of PRU data memory in r2
    MOV       r2, 0x0004

    // Move value from register to the PRU local data memory using registers
    ST32      r1, r2

    // Load 32 bit value into r3
    MOV       r3, 0x0000567A

    //Load 4 bytes from memory location c3(PRU0/1 Local Data)+4 into r4 using constant table
    LBCO      r4, CONST_PRUDRAM, 4, 4 

    // Add r3 and r4
    ADD       r3, r3, r4

    //Store result in into memory location c3(PRU0/1 Local Data)+8 using constant table
    SBCO      r3, CONST_PRUDRAM, 8, 4

#ifdef AM33XX
    // Send notification to Host for program completion
    MOV R31.b0, PRU0_ARM_INTERRUPT+16
#else
    MOV R31.b0, PRU0_ARM_INTERRUPT
#endif

    HALT
