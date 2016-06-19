;;-----------------------------------
;;  Delta control 
;;------------------------------------------------------------
;; Pause
;;       Move xy to lmsw
;;------------------------------------------------------------
    ;; Check Control command : ST_CMD_PAUSE
PAUSE_DELTA:
    ;; Get homing direction bits
    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 72
    LBCO r20, CONST_PRUDRAM, r0, 4  ;; endstop invert

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT

    UpdateDir r1, r10, r3, AXIS_X, DIR_X
    UpdateDir r1, r10, r3, AXIS_Y, DIR_Y
    UpdateDir r1, r10, r3, AXIS_Z, DIR_Z

    SBBO r1, r6, 4, 4

    ;; Get homing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4

PAUSE_HOME_STEP_DELTA:
    ;; generate homing step
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r10, r6, 0, 4  ;;GPIO2_DATAIN

    QBBS PAUSE_MIN_X_CHECK_HIGHT_DELTA_1, r20, 0
        QBBC PAUSE_CH_MIN_Y_DELTA, r10, MIN_X
        jmp PAUSE_MIN_X_CHECK_HIGHT_OUT_DELTA_1
PAUSE_MIN_X_CHECK_HIGHT_DELTA_1:
        QBBS PAUSE_CH_MIN_Y_DELTA, r10, MIN_X
PAUSE_MIN_X_CHECK_HIGHT_OUT_DELTA_1:
    SET r0, STEP_X

PAUSE_CH_MIN_Y_DELTA:
    QBBS PAUSE_MIN_Y_CHECK_HIGHT_DELTA_1, r20, 1
        QBBC PAUSE_CH_MIN_Z_DELTA, r8, MIN_Y
        jmp PAUSE_MIN_Y_CHECK_HIGHT_OUT_DELTA_1
PAUSE_MIN_Y_CHECK_HIGHT_DELTA_1:
        QBBS PAUSE_CH_MIN_Z_DELTA, r8, MIN_Y
PAUSE_MIN_Y_CHECK_HIGHT_OUT_DELTA_1:
    SET r0, STEP_Y

PAUSE_CH_MIN_Z_DELTA:
    QBBS PAUSE_MIN_Z_CHECK_HIGHT_DELTA_1, r20, 2
        QBBC PAUSE_CH_LMSW_OUT_DELTA, r7, MIN_Z
        jmp PAUSE_MIN_Z_CHECK_HIGHT_OUT_DELTA_1
PAUSE_MIN_Z_CHECK_HIGHT_DELTA_1:
        QBBS PAUSE_CH_LMSW_OUT_DELTA, r7, MIN_Z
PAUSE_MIN_Z_CHECK_HIGHT_OUT_DELTA_1:
    SET r0, STEP_Z


PAUSE_CH_LMSW_OUT_DELTA:
    ;; gen step signal
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z

    SBBO r0, r5, 4, 4
    DELAY_NS r3

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r10, r6, 0, 4  ;;GPIO2_DATAIN

    QBBS PAUSE_MIN_X_CHECK_HIGHT_DELTA_2, r20, 0
        QBBS PAUSE_HOME_STEP_DELTA, r10, MIN_X
        jmp PAUSE_MIN_X_CHECK_HIGHT_DELTA_2_OUT
PAUSE_MIN_X_CHECK_HIGHT_DELTA_2:
        QBBC PAUSE_HOME_STEP_DELTA, r10, MIN_X
PAUSE_MIN_X_CHECK_HIGHT_DELTA_2_OUT:

    QBBS PAUSE_MIN_Y_CHECK_HIGHT_DELTA_2, r20, 1
        QBBS PAUSE_HOME_STEP_DELTA, r8, MIN_Y
        jmp PAUSE_MIN_Y_CHECK_HIGHT_DELTA_2_OUT
PAUSE_MIN_Y_CHECK_HIGHT_DELTA_2:
        QBBC PAUSE_HOME_STEP_DELTA, r8, MIN_Y
PAUSE_MIN_Y_CHECK_HIGHT_DELTA_2_OUT:

    QBBS PAUSE_MIN_Z_CHECK_HIGHT_DELTA_2, r20, 2
        QBBS PAUSE_HOME_STEP_DELTA, r7, MIN_Z
        jmp PAUSE_MIN_Z_CHECK_HIGHT_DELTA_2_OUT
PAUSE_MIN_Z_CHECK_HIGHT_DELTA_2:
        QBBC PAUSE_HOME_STEP_DELTA, r7, MIN_Z
PAUSE_MIN_Z_CHECK_HIGHT_DELTA_2_OUT:

    MOV  r0, QUEUE_SIZE
    MOV  r10, STATE_PAUSE_FINISH  
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBNE CH_STATE_DELTA, r1, STATE_STOP
    MOV r2, 0
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
CH_STATE_DELTA:
    SBCO r10, CONST_PRUDRAM, r0, 4
    ;;MOV  R31.b0, PRU0_ARM_IRQ + 16
    JMP MAIN
;;------------------------------------------------------------
;; Resume:
;;        move back to the last position
;;------------------------------------------------------------
RESUME_DELTA:
    ;; Get homing direction bits
    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    NOT r3, r3

    ;; generate direction
    LBBO r1, r6, 4, 4 ;;GPIO2_DATAOUT

    UpdateDir r1, r10, r3, AXIS_X, DIR_X
    UpdateDir r1, r10, r3, AXIS_Y, DIR_Y
    UpdateDir r1, r10, r3, AXIS_Z, DIR_Z

    SBBO r1, r6, 4, 4

    ;; Get homing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4

    ;; Get pause position
    ADD  r0, r0, 20
    LBCO r8, CONST_PRUDRAM, r0, 4

    ADD r0, r0, 4
    LBCO r11, CONST_PRUDRAM, r0, 4

    ADD r0, r0, 4
    LBCO r10, CONST_PRUDRAM, r0, 4

RESUME_STEP_DELTA:
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    
    QBEQ RESUME_UP_Y_DELTA, r8, 0
    SUB r8, r8, 1
    SET r0, STEP_X

RESUME_UP_Y_DELTA:
    QBEQ RESUME_UP_Z, r11, 0
    SUB r11, r11, 1
    SET r0, STEP_Y

RESUME_UP_Z:
    QBEQ RESUME_GEN_STEP_DELTA, r10, 0
    SUB r10, r10, 1
    SET r0, STEP_Z

RESUME_GEN_STEP_DELTA:
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z

    SBBO r0, r5, 4, 4
    DELAY_NS r3
    
    QBNE RESUME_STEP_DELTA, r8, 0
    QBNE RESUME_STEP_DELTA, r11, 0
    QBNE RESUME_STEP_DELTA, r10, 0
    
    ;; Change state to print
    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBNE RESUME_CH_STATE_DELTA, r1, STATE_STOP
    ;;If state change to stop, irq ARM
    ;;MOV  R31.b0, PRU0_ARM_IRQ + 16
RESUME_CH_STATE_DELTA:
    MOV  r10, STATE_PRINT
    SBCO r10, CONST_PRUDRAM, r0, 4
    JMP MAIN

;;------------------------------------------------------------
;; Homing
;;------------------------------------------------------------
HOMING_DELTA:
#ifdef HOMING_DELTA
    ;; Get homing direction bits
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT

    UpdateDir r1, r10, r3, AXIS_X, DIR_X
    UpdateDir r1, r10, r3, AXIS_Y, DIR_Y
    UpdateDir r1, r10, r3, AXIS_Z, DIR_Z

    SBBO r1, r6, 4, 4

    ;; Get homing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4

HOME_STEP_DELTA:
    ;; Get homing_axis
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 4
    LBCO r10, CONST_PRUDRAM, r0, 4

    ;; generate homing step
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r11, r6, 0, 4  ;;GPIO2_DATAIN

CH_MIN_X_DELTA:
    QBBC CH_MIN_Y_DELTA, r10, 0
    QBBC CH_MIN_Y_DELTA, r11, MIN_X
    SET r0, STEP_X

CH_MIN_Y_DELTA:
    QBBC CH_MIN_Z_DELTA, r10, 1
    QBBC CH_MIN_Z_DELTA, r8, MIN_Y
    SET r0, STEP_Y

CH_MIN_Z_DELTA:
    QBBC CH_LMSW_OUT_DELTA, r10, 2
    QBBC CH_LMSW_OUT_DELTA, r7, MIN_Z
    SET r0, STEP_Z

CH_LMSW_OUT_DELTA:
    ;; gen step signal
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r11, r6, 0, 4  ;;GPIO2_DATAIN

CH_MIN_X_DONE_DELTA:    
    QBBC CH_MIN_Y_DONE_DELTA, r10, 0
    QBBS HOME_STEP_DELTA, r11, MIN_X
    CLR r10, 0

CH_MIN_Y_DONE_DELTA:
    QBBC CH_MIN_Z_DONE_DELTA, r10, 1
    QBBS HOME_STEP_DELTA, r8, MIN_Y
    CLR r10, 1

CH_MIN_Z_DONE_DELTA:
    QBBC HOMING_DONE_DELTA, r10, 2
    QBBS HOME_STEP_DELTA, r7, MIN_Z
    CLR r10, 2

HOMING_DONE_DELTA:
    ;; Clear homing_axis flag
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 4
    SBCO r10, CONST_PRUDRAM, r0, 4

    ;; Change state to print
    MOV  r0,  QUEUE_SIZE
    MOV  r10, STATE_PRINT
    SBCO r10, CONST_PRUDRAM, r0, 4
#endif
    
    JMP MAIN

;;------------------------------------------------------------
;; Queue Movement
;;------------------------------------------------------------
.using Print_Scope
PRINT_DELTA:
    ;; Read next element from the ring-buffer
    ;;.assign QueueHeader, r3, r3, header
    LBCO header, CONST_PRUDRAM, r2, SIZE(header)
    QBNE PRINT_DELTA_1, header.state, STATE_EMPTY
    jmp MAIN

PRINT_DELTA_1:
    ;; Get travel parameters
    ADD r8, r2, SIZE(QueueHeader)
    LBCO move, CONST_PRUDRAM, r8, SIZE(move)


    QBBC PRINT_DELTA_GCODE, header.type, BLOCK_M_CMD_BIT

    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 52
    LBCO r7, CONST_PRUDRAM, r8, 4
    ADD r7, r7, 1
    SBCO r7, CONST_PRUDRAM, r8, 4 

    jmp  DONE_STEP_GEN_DELTA 

PRINT_DELTA_GCODE:

    ;; Load DIR GPIO value
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    
    ;; Set direction bits
    UpdateDir r1, r8, header.dir_bits, AXIS_X, DIR_X
    UpdateDir r1, r8, header.dir_bits, AXIS_Y, DIR_Y
    UpdateDir r1, r8, header.dir_bits, AXIS_Z, DIR_Z
     
    ;; Output direction bits to GPIO
    SBBO r1, r6, 4, 4

    MOV  r9, EXT_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER1_DIR, r9.t0 
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT0_STEP_DIR_OFFSET 
        MOV  r9, EXT_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
 DELTA_ACTIVE_EXTRUDER1_DIR:   
    MOV  r9, EXT_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER2_DIR, r9.t1
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT1_STEP_DIR_OFFSET 
        MOV  r9, EXT_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
 DELTA_ACTIVE_EXTRUDER2_DIR:   
    MOV  r9, EXT2_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER_DIR_OUT, r9.t2 
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT2_STEP_DIR_OFFSET 
        MOV  r9, EXT2_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER_DIR_OUT:   

    MOV r1, move.steps_count
    MOV r8, move.steps_count
    LSR r8, r8, 1
.using Print_counter_Scope
    MOV counter.x, r8
    MOV counter.y, r8 
    MOV counter.z, r8 
    MOV counter.e, r8 


STEP_GEN_DELTA:
;; check limit switch
    MOV r7, QUEUE_SIZE
    ADD r7, r7, 8
    LBCO r8, CONST_PRUDRAM, r7, 4 ;; home_dir
    
    XOR r7, r8, header.dir_bits

DELTA_HIT_CHECK_AUTO_LEVEL_Z:
    QBBC DELTA_HIT_CHECK_X, r7, AXIS_Z_BIT
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS DELTA_HIT_CHECK_AUTOLEVEL_HIGHT_1, r8, 3 
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBS DELTA_HIT_CHECK_X, r8, AUTO_LEVEL_Z
        jmp DELTA_HIT_CHECK_AUTOLEVEL_HIGHT_1_OUT
DELTA_HIT_CHECK_AUTOLEVEL_HIGHT_1:
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBC DELTA_HIT_CHECK_X, r8, AUTO_LEVEL_Z
DELTA_HIT_CHECK_AUTOLEVEL_HIGHT_1_OUT:
    MOV move.steps_x, 0
    MOV move.steps_y, 0
    MOV move.steps_z, 0
    MOV r1, 0
    jmp  DONE_STEP_GEN_DELTA 


DELTA_HIT_CHECK_X:
    QBBS DELTA_HIT_CHECK_Y, r7, AXIS_X_BIT
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS DELTA_HIT_CHECK_X_HIGHT_1, r8, 0 
        LBBO r8, r6, 0, 4  ;;GPIO2_DATAIN
        QBBS DELTA_HIT_CHECK_Y, r8, MIN_X
        jmp DELTA_HIT_CHECK_X_HIGHT_1_OUT
DELTA_HIT_CHECK_X_HIGHT_1:
        LBBO r8, r6, 0, 4  ;;GPIO2_DATAIN
        QBBC DELTA_HIT_CHECK_Y, r8, MIN_X
DELTA_HIT_CHECK_X_HIGHT_1_OUT:
    MOV move.steps_x, 0
    ;jmp DELTA_HIT_CHECK_ALL_HIT

DELTA_HIT_CHECK_Y:
    QBBS DELTA_HIT_CHECK_Z, r7, AXIS_Y_BIT
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS DELTA_HIT_CHECK_Y_HIGHT_1, r8, 1 
        LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
        QBBS DELTA_HIT_CHECK_Z, r8, MIN_Y
        jmp DELTA_HIT_CHECK_Y_HIGHT_1_OUT
DELTA_HIT_CHECK_Y_HIGHT_1:
        LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
        QBBC DELTA_HIT_CHECK_Z, r8, MIN_Y
DELTA_HIT_CHECK_Y_HIGHT_1_OUT:
    MOV move.steps_y, 0
    ;jmp  DELTA_HIT_CHECK_ALL_HIT


DELTA_HIT_CHECK_Z:
    QBBS DELTA_NOT_HIT_STEP_GEN, r7, AXIS_Z_BIT
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS DELTA_HIT_CHECK_Z_HIGHT_1, r8, 2 
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBS DELTA_NOT_HIT_STEP_GEN, r8, MIN_Z
        jmp DELTA_HIT_CHECK_Z_HIGHT_1_OUT
DELTA_HIT_CHECK_Z_HIGHT_1:
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBC DELTA_NOT_HIT_STEP_GEN, r8, MIN_Z
DELTA_HIT_CHECK_Z_HIGHT_1_OUT:
    MOV move.steps_z, 0
    ;jmp  DELTA_HIT_CHECK_ALL_HIT


DELTA_HIT_CHECK_ALL_HIT:
    MOV r8, move.steps_x
    ADD r8, r8, move.steps_y
    ADD r8, r8, move.steps_z

    ;;DELTA_ALL_HIT
    QBNE DELTA_NOT_HIT_STEP_GEN, r8, 0
    MOV move.steps_x, 0
    MOV move.steps_y, 0
    MOV move.steps_z, 0
    MOV r1, 0
    jmp  DONE_STEP_GEN_DELTA 

DELTA_NOT_HIT_STEP_GEN:

    ;; Load STEP gpio data 
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    ;; Generate motor step high voltage
    UpdateStep r0, counter.x, move.steps_x, move.steps_count, STEP_X
    UpdateStep r0, counter.y, move.steps_y, move.steps_count, STEP_Y
    UpdateStep r0, counter.z, move.steps_z, move.steps_count, STEP_Z
    SBBO r0, r5, 4, 4

    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER1_CTL_SET, r9.t0
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT0_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER1_CTL_SET:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER2_CTL_SET, r9.t1
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT1_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER2_CTL_SET:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER_CTL_SET_OUT, r9.t2
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT2_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER_CTL_SET_OUT:   

    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    ;; Update current pos
UP_POS_X_DELTA:
    QBBC UP_POS_Y_DELTA, r0, STEP_X
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 16

    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_X
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_Y_DELTA:
    QBBC UP_POS_Z_DELTA, r0, STEP_Y
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 20
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_Y
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_Z_DELTA:
    QBBC UP_POS_E_DELTA, r0, STEP_Z
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 24
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_Z
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_E_DELTA:
    QBBC UP_POS_DONE_DELTA, r0, STEP_E
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 28
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_E
    SBCO r9, CONST_PRUDRAM, r8, 4
UP_POS_DONE_DELTA:


    ;; Calc Delay time for speed control
    CalculateDelay r8, move
    ;;CalculateDelay_T r8

    DELAY_NS r8 

    ;; Generate motor step low voltage
    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z

    SBBO r0, r5, 4, 4

    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER1_CTL_CLR, r9.t0
        CLR  r7, EXT0_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER1_CTL_CLR:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER2_CTL_CLR, r9.t1
        CLR  r7, EXT1_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER2_CTL_CLR:
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC DELTA_ACTIVE_EXTRUDER_CTL_CLR_OUT, r9.t2
        CLR  r7, EXT2_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
DELTA_ACTIVE_EXTRUDER_CTL_CLR_OUT:   

    DELAY_NS r8

    ;; Check all step done
    SUB r1, r1, 1
    QBEQ DONE_STEP_GEN_DELTA, r1, 0

    ;;check queue state 
    MOV  r8, QUEUE_SIZE
    LBCO r9, CONST_PRUDRAM, r8, 4
    QBEQ DONE_STEP_GEN_DELTA, r9, STATE_STOP

    JMP STEP_GEN_DELTA
.leave Print_counter_Scope

DONE_STEP_GEN_DELTA:
    ;; Make slot as empty
    MOV  header.state, STATE_EMPTY 
    SBCO header.state, CONST_PRUDRAM, r2, 1

IRQ_DELTA:
    ;; Send IRQ_DELTA to ARM
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 44
    LBCO r3, CONST_PRUDRAM, r0, 4
    QBNE IRQ_OUT_DELTA, r3, r2
    MOV  R31.b0, PRU0_ARM_IRQ + 16
IRQ_OUT_DELTA:

    ;; Next position in ring buffer
    ADD  r2, r2, QUEUE_ELEMENT_SIZE
    MOV  r1, QUEUE_LEN * QUEUE_ELEMENT_SIZE

    QBGE DELTA_PRINT_END_1, r1, r2
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
    jmp MAIN

DELTA_PRINT_END_1:
    ZERO &r2, 4
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
    JMP  MAIN
.leave Print_Scope

;;------------------------------------------------------------
;; Finish
;;------------------------------------------------------------
;FINISH:
;    MOV  header.state, STATE_EMPTY
;    SBCO header.state, CONST_PRUDRAM, r2, 1
;    MOV  R31.b0, PRU0_ARM_IRQ + 16
;
;    HALT
