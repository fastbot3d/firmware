//------------------------------------------------------------
// pruss_unicorn.p
// pru asm to generate step and dir motor control signals
//------------------------------------------------------------
// Registers
// r0, r1    -> free for calculation
// r2        -> queue pos
// r3        -> QueueHeader
// r4        -> global GPIO0 DATAIN
// r5        -> global GPIO1 DATAIN
// r6        -> global GPIO2 DATAIN 
// r25       -> global machine_type, bbp1_extend_func, reserved_1, reserved_2
// r7  ~ r12 -> scratch
// r13 ~ r24 -> Movement
// r26 ~ r29 -> Counter
//------------------------------------------------------------
#include "../pruss_unicorn.hp"

.origin 0
.entrypoint INIT

//#define DELTA
#define DUAL_Z
#define BBP_1S
;;------------------------------------------------------------
;; Init
;;------------------------------------------------------------
INIT:    
    ;; Clear STANDBY_INIT bit
    LBCO r0, C4, 4, 4                    
    CLR  r0, r0, 4
    SBCO r0, C4, 4, 4 
    
    ;;test
    MOV r0, 0x00000100
    MOV r1, CTPPR_0
    ST32 r0, r1

    LBCO    r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    SBCO    r0, CONST_PRUCFG, 4, 4

    // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
    // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
    //31 EMIF0 DDR Base 0x80nn_nn00, nnnn = c31_pointer[15:0]
    //0x80e00000;
    MOV     r0, 0xe0000000
    MOV     r1, CTPPR_1
    ST32    r0, r1

    ;; Queue address in PRU memory
    MOV r2, 0

    ;; GPIO reg base address
    MOV r4,  GPIO_0 | GPIO_DATAIN
    MOV r5,  GPIO_1 | GPIO_DATAIN
    MOV r6,  GPIO_2 | GPIO_DATAIN

    .assign ExtendParameter, r29, r29, gExtendParameter

.enter Print_Scope
    .assign QueueHeader, r3, r3, header
    .assign Movement, MOVE_START, MOVE_END, move
.leave Print_Scope

.enter Print_counter_Scope
    .assign Counter, MC_START, MC_END, counter
.leave Print_counter_Scope

.enter ReturnAxis_Scope
    .assign Queue, r13, r23, queue
.leave ReturnAxis_Scope

MAIN:
    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 48
    LBCO gExtendParameter, CONST_PRUDRAM, r0, SIZE(gExtendParameter)

    QBEQ MAIN_DUAL_Z,   gExtendParameter.machine_type, MACHINE_XYZ
    QBEQ MAIN_COREXY,   gExtendParameter.machine_type, MACHINE_COREXY 
    QBEQ MAIN_DELTA,    gExtendParameter.machine_type, MACHINE_DELTA   

;;--------------------------------------
MAIN_DUAL_Z:
    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4

    ;; JMP Table
    QBEQ NEXT_DUAL_MAIN, r1, STATE_IDLE
    QBNE NEXT_DUAL_Z_1,  r1, STATE_STOP
    JMP PAUSE
NEXT_DUAL_MAIN:
        MOV r2, 0
        JMP MAIN
NEXT_DUAL_Z_1:
    QBNE NEXT_DUAL_Z_2,  r1, STATE_PRINT
    JMP PRINT
NEXT_DUAL_Z_2:
    QBNE NEXT_DUAL_Z_3,  r1, STATE_PAUSE
    JMP PAUSE
NEXT_DUAL_Z_3:
    QBNE NEXT_DUAL_Z_4, r1, STATE_RESUME
    JMP RESUME
NEXT_DUAL_Z_4:
    QBNE NEXT_DUAL_Z_5, r1, STATE_HOME
    JMP HOMING
NEXT_DUAL_Z_5:
    QBEQ MAIN,   r1, STATE_PAUSE_FINISH  
    QBEQ TEST,   r1, STATE_TEST
    QBEQ LOAD_FILAMENT_STEP,     r1, LOAD_FILAMENT
    QBEQ UNLOAD_FILAMENT_STEP,   r1, UNLOAD_FILAMENT

    JMP MAIN

;;--------------------------------------
MAIN_DELTA:
    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4

    ;; JMP Table
    QBEQ NEXT_DELTA_MAIN, r1, STATE_IDLE
    QBNE NEXT_DELTA_1,  r1, STATE_STOP
    JMP PAUSE_DELTA
NEXT_DELTA_MAIN:
        MOV r2, 0
        JMP MAIN
NEXT_DELTA_1:
    QBNE NEXT_DELTA_2,  r1, STATE_PRINT
    JMP PRINT_DELTA
NEXT_DELTA_2:
    QBNE NEXT_DELTA_3,  r1, STATE_PAUSE
    JMP PAUSE_DELTA
NEXT_DELTA_3:
    QBNE NEXT_DELTA_4, r1, STATE_RESUME
    JMP RESUME_DELTA
NEXT_DELTA_4:
    QBNE NEXT_DELTA_5, r1, STATE_HOME
    JMP HOMING_DELTA
NEXT_DELTA_5:
    QBEQ MAIN,   r1, STATE_PAUSE_FINISH  
    QBEQ TEST,   r1, STATE_TEST
    QBEQ LOAD_FILAMENT_STEP,     r1, LOAD_FILAMENT
    QBEQ UNLOAD_FILAMENT_STEP,   r1, UNLOAD_FILAMENT

    JMP MAIN

;;--------------------------------------
MAIN_COREXY:
    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4

    ;; JMP Table
    QBEQ NEXT_COREXY_MAIN, r1, STATE_IDLE
    QBNE NEXT_COREXY_1,  r1, STATE_STOP
    JMP PAUSE_COREXY
NEXT_COREXY_MAIN:
        MOV r2, 0
        JMP MAIN
NEXT_COREXY_1:
    QBNE NEXT_COREXY_2,  r1, STATE_PRINT
    JMP PRINT_COREXY
NEXT_COREXY_2:
    QBNE NEXT_COREXY_3,  r1, STATE_PAUSE
    JMP PAUSE_COREXY
NEXT_COREXY_3:
    QBNE NEXT_COREXY_4, r1, STATE_RESUME
    JMP RESUME_COREXY
NEXT_COREXY_4:
    QBNE NEXT_COREXY_5, r1, STATE_HOME
    JMP HOMING_COREXY
NEXT_COREXY_5:
    QBEQ MAIN,   r1, STATE_PAUSE_FINISH  
    QBEQ TEST,   r1, STATE_TEST
    QBEQ LOAD_FILAMENT_STEP,     r1, LOAD_FILAMENT
    QBEQ UNLOAD_FILAMENT_STEP,   r1, UNLOAD_FILAMENT

    JMP MAIN

;;------------------------------------------------------------
;; Filament 
;;------------------------------------------------------------
;;; E0 E1 E2 
; #define STEP_E    28 ;;GPIO1_28
; #define STEP_E2   24 ;;GPIO1_24
; #define STEP_E3   25 ;;GPIO1_25
; #define DIR_E     2  ;;GPIO2_2
; #define DIR_E2    5  ;;GPIO2_5
; #define DIR_E3    7  ;;GPIO3_7

LOAD_FILAMENT_STEP:
    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    NOT r3, r3

    ;; dir 
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    UpdateDir r1, r10, r3, AXIS_E, DIR_E
    UpdateDir r1, r10, r3, AXIS_E, DIR_E2
    SBBO r1, r6, 4, 4

    MOV r10, GPIO_3_BASE_2
    ADD  r7, r6, r10 
    LBBO r1, r7, 4, 4  ;;GPIO3_DATAOUT
    UpdateDir r1, r10, r3, AXIS_E, DIR_E3
    SBBO r1, r7, 4, 4
    
    ;; step
    mov r3, 1500000 ;; delay
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    SET r0, STEP_E
    SET r0, STEP_E2
    SET r0, STEP_E3
    SBBO r0, r5, 4, 4

    DELAY_NS r3

    CLR r0, STEP_E
    CLR r0, STEP_E2
    CLR r0, STEP_E3
    SBBO r0, r5, 4, 4

    DELAY_NS r3

    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBEQ LOAD_FILAMENT_STEP, r1, LOAD_FILAMENT

    JMP MAIN

UNLOAD_FILAMENT_STEP:
    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    ;; dir 
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    UpdateDir r1, r10, r3, AXIS_E, DIR_E
    UpdateDir r1, r10, r3, AXIS_E, DIR_E2
    SBBO r1, r6, 4, 4

    MOV r10, GPIO_3_BASE_2
    ADD  r7, r6, r10 
    LBBO r1, r7, 4, 4  ;;GPIO3_DATAOUT
    UpdateDir r1, r10, r3, AXIS_E, DIR_E3
    SBBO r1, r7, 4, 4
    
    ;; step
    mov r3, 1500000 ;; delay
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    SET r0, STEP_E
    SET r0, STEP_E2
    SET r0, STEP_E3
    SBBO r0, r5, 4, 4

    DELAY_NS r3

    CLR r0, STEP_E
    CLR r0, STEP_E2
    CLR r0, STEP_E3
    SBBO r0, r5, 4, 4

    DELAY_NS r3

    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBEQ UNLOAD_FILAMENT_STEP, r1, UNLOAD_FILAMENT

    JMP MAIN

;;------------------------------------------------------------
;; Testing
;;------------------------------------------------------------
TEST:
#ifdef TEST_BBP_BOARD 
    ;; Get testing direction bits
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT

/*
    MOV  R31.b0, PRU1_ARM_IRQ + 16
    mov r8, 1000000
    DELAY_NS r8 
    MOV  R31.b0, PRU1_ARM_IRQ + 16
    mov r8, 1000000
    DELAY_NS r8 
    MOV  R31.b0, PRU1_ARM_IRQ + 16
    mov r8, 1000000
    DELAY_NS r8 
    MOV  R31.b0, PRU1_ARM_IRQ + 16
*/

    UpdateDir r1, r10, r3, AXIS_X, DIR_X
    UpdateDir r1, r10, r3, AXIS_Y, DIR_Y
    UpdateDir r1, r10, r3, AXIS_Z, DIR_Z
    UpdateDir r1, r10, r3, AXIS_Z, DIR_E
    UpdateDir r1, r10, r3, AXIS_Z, DIR_E2
#ifdef  BBP_1S
    UpdateDir r1, r10, r3, AXIS_Z, DIR_E2
    UpdateDir r1, r10, r3, AXIS_Z, DIR_E2
    //TODO: U2 dir set up
#endif

    SBBO r1, r6, 4, 4

    ;; Get testing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4
 
TEST_STEP:
    ;; generate test step
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r11, r6, 0, 4  ;;GPIO2_DATAIN
    
    ;; gen step signal
    SET r0, STEP_X
    SET r0, STEP_Y
    SET r0, STEP_Z
    SET r0, STEP_E
    SET r0, STEP_E2
#ifdef BBP_1S
    SET r0, STEP_E3
    SET r0, STEP_U
    SET r0, STEP_U2
#endif

    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z
    CLR r0, STEP_E
    CLR r0, STEP_E2
#ifdef BBP_1S
    CLR r0, STEP_E3
    CLR r0, STEP_U
    CLR r0, STEP_U2
#endif

    SBBO r0, r5, 4, 4
    DELAY_NS r3

    ;; check limit switch min_x
    QBBS TEST_STEP, r11, MIN_X

TEST_DONE:
    ;; Change state to print
    MOV  r0,  QUEUE_SIZE
    MOV  r10, STATE_PRINT
    SBCO r10, CONST_PRUDRAM, r0, 4
#endif

    JMP MAIN

#include "pruss_unicorn_normal.p"
#include "pruss_unicorn_delta.p"
#include "pruss_unicorn_corexy.p"
