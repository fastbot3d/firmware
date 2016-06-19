;;------------------------------------------------------------
;; Pause
;;       Move xy to lmsw
;;------------------------------------------------------------
    ;; Check Control command : ST_CMD_PAUSE
PAUSE:
    ;; Get homing direction bits
    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 56
    LBCO r11, CONST_PRUDRAM, r0, 4  ;;lift z distance

    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 72
    LBCO r20, CONST_PRUDRAM, r0, 4  ;; endstop invert

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r21, CONST_PRUDRAM, r9, 4 
    QBBC NORMAL_PAUSE_DUAL_XY_DIR_OUT, r21, 0 
        MOV  r9, EXT_STEP_DIR_GPIO  ;;ext1 --> X 
        LBBO r21, r9, 0, 4  
        UpdateDir r21, r10, r3, AXIS_X, EXT1_STEP_DIR_OFFSET 
        SBBO r21, r9, 0, 4

        MOV  r9, EXT2_STEP_DIR_GPIO  ;;ext2 --> Y 
        LBBO r21, r9, 0, 4  
        UpdateDir r21, r10, r3, AXIS_Y, EXT2_STEP_DIR_OFFSET 
        SBBO r21, r9, 0, 4
NORMAL_PAUSE_DUAL_XY_DIR_OUT:

    UpdateDir r1, r10, r3, AXIS_X, DIR_X
    UpdateDir r1, r10, r3, AXIS_Y, DIR_Y
    XOR r3, r3,  AXIS_Z
    UpdateDir r1, r10, r3, AXIS_Z, DIR_Z

    SBBO r1, r6, 4, 4

    MOV r7, r6 
    MOV r8, GPIO_3_BASE_2 
    ADD r7, r7, r8       ;;GPIO3_DATAOUT
    LBBO r1, r7, 4, 4
    UpdateDir r1, r10, r3, AXIS_Z, DIR_U
    SBBO r1, r7, 4, 4

    ;; Get homing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4

PAUSE_HOME_STEP:
    ;; generate homing step
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r10, r6, 0, 4  ;;GPIO2_DATAIN

    QBBS PAUSE_MIN_X_CHECK_HIGHT_1, r20, 0 
        QBBC PAUSE_CH_MIN_Y, r10, MIN_X
        jmp PAUSE_MIN_X_CHECK_HIGHT_1_OUT
PAUSE_MIN_X_CHECK_HIGHT_1:
        QBBS PAUSE_CH_MIN_Y, r10, MIN_X
PAUSE_MIN_X_CHECK_HIGHT_1_OUT:
    SET r0, STEP_X

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r21, CONST_PRUDRAM, r9, 4 
    QBBC PAUSE_NORMAL_DUAL_X_SET_OUT, r21, 0
        SET r0, EXT1_STEP_CTL_OFFSET
PAUSE_NORMAL_DUAL_X_SET_OUT:

PAUSE_CH_MIN_Y:
    QBBS PAUSE_MIN_Y_CHECK_HIGHT_1, r20, 1
        QBBC NORMAL_PAUSE_MOVE_DOWN_Z, r8, MIN_Y
        jmp PAUSE_MIN_Y_CHECK_HIGHT_1_OUT
PAUSE_MIN_Y_CHECK_HIGHT_1:
        QBBS NORMAL_PAUSE_MOVE_DOWN_Z, r8, MIN_Y
PAUSE_MIN_Y_CHECK_HIGHT_1_OUT:
        SET r0, STEP_Y

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r21, CONST_PRUDRAM, r9, 4 
    QBBC PAUSE_NORMAL_DUAL_Y_SET_OUT, r21, 0
        SET r0,  EXT2_STEP_CTL_OFFSET
PAUSE_NORMAL_DUAL_Y_SET_OUT:


NORMAL_PAUSE_MOVE_DOWN_Z:
    QBEQ PAUSE_CH_LMSW_OUT, r11, 0
    SET r0, STEP_Z
    SET r0, STEP_U
    SUB r11, r11, 1

    ;; cancel down 
    MOV r12, QUEUE_SIZE
    ADD r12, r12, 64
    LBCO r13, CONST_PRUDRAM, r12, 4
    SUB r13, r13, 1
    SBCO r13, CONST_PRUDRAM, r12, 4

PAUSE_CH_LMSW_OUT:
    ;; gen step signal
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z
    CLR r0, STEP_U

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r21, CONST_PRUDRAM, r9, 4 
    QBBC PAUSE_NORMAL_DUAL_XY_CLR_OUT, r21, 0 
        CLR r0, EXT1_STEP_CTL_OFFSET 
        CLR r0, EXT2_STEP_CTL_OFFSET 
PAUSE_NORMAL_DUAL_XY_CLR_OUT:

    SBBO r0, r5, 4, 4

    DELAY_NS r3

    QBBS PAUSE_MIN_X_CHECK_HIGHT_2, r20, 0
        QBBS PAUSE_HOME_STEP, r10, MIN_X
        jmp PAUSE_MIN_X_CHECK_HIGHT_2_OUT
PAUSE_MIN_X_CHECK_HIGHT_2:
        QBBC PAUSE_HOME_STEP, r10, MIN_X
PAUSE_MIN_X_CHECK_HIGHT_2_OUT:

    QBBS PAUSE_MIN_Y_CHECK_HIGHT_2, r20, 1
        QBBS PAUSE_HOME_STEP, r8, MIN_Y
        jmp PAUSE_MIN_Y_CHECK_HIGHT_2_OUT
PAUSE_MIN_Y_CHECK_HIGHT_2:
        QBBC PAUSE_HOME_STEP, r8, MIN_Y
PAUSE_MIN_Y_CHECK_HIGHT_2_OUT:

    QBNE PAUSE_HOME_STEP, r11, 0

    MOV  r0, QUEUE_SIZE
    MOV  r10, STATE_PAUSE_FINISH  
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBNE CH_STATE, r1, STATE_STOP
    MOV r2, 0
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
CH_STATE:
    SBCO r10, CONST_PRUDRAM, r0, 4
    ;;MOV  R31.b0, PRU0_ARM_IRQ + 16
    JMP MAIN

;;------------------------------------------------------------
;; Resume:
;;        move back to the last position
;;------------------------------------------------------------
RESUME:
    ;; Get homing direction bits
    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 8
    LBCO r3, CONST_PRUDRAM, r0, 4

    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 56
    LBCO r11, CONST_PRUDRAM, r0, 4  ;;move down z distance

    NOT r3, r3

    ;; generate direction
    LBBO r1, r6, 4, 4 ;;GPIO2_DATAOUT

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC RESUME_NORMAL_DUAL_XY_DIR_OUT, r7, 0 
        MOV  r9, EXT_STEP_DIR_GPIO  ;;ext1 --> X 
        LBBO r7, r9, 0, 4  
        UpdateDir r7, r10, r3, AXIS_X, EXT1_STEP_DIR_OFFSET 
        SBBO r7, r9, 0, 4

        MOV  r9, EXT2_STEP_DIR_GPIO  ;;ext2 --> Y 
        LBBO r7, r9, 0, 4  
        UpdateDir r7, r10, r3, AXIS_Y, EXT2_STEP_DIR_OFFSET 
        SBBO r7, r9, 0, 4
RESUME_NORMAL_DUAL_XY_DIR_OUT:

    UpdateDir r1, r10, r3, AXIS_X, DIR_X
    UpdateDir r1, r10, r3, AXIS_Y, DIR_Y
    XOR r3, r3,  AXIS_Z
    UpdateDir r1, r10, r3, AXIS_Z, DIR_Z

    SBBO r1, r6, 4, 4

    MOV r7, r6 
    MOV r8, GPIO_3_BASE_2 
    ADD r7, r7, r8       ;;GPIO3_DATAOUT
    LBBO r1, r7, 4, 4  
    UpdateDir r1, r10, r3, AXIS_Z, DIR_U
    SBBO r1, r7, 4, 4

    ;; Get homing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4

    ;; Get pause position
    ADD  r0, r0, 20
    LBCO r8, CONST_PRUDRAM, r0, 4

    ADD r0, r0, 4
    LBCO r10, CONST_PRUDRAM, r0, 4

NORMAL_RESUME_UP_Z_SET:
    QBEQ RESUME_STEP, r11, 0
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    SET r0, STEP_Z
    SET r0, STEP_U
    SBBO r0, r5, 4, 4
    SUB r11, r11, 1

    DELAY_NS r3

    CLR r0, STEP_Z
    CLR r0, STEP_U
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    QBNE NORMAL_RESUME_UP_Z_SET, r11, 0

RESUME_STEP:
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    
    QBEQ RESUME_UP_Y, r8, 0
    SUB r8, r8, 1
    SET r0, STEP_X

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC RESUME_NORMAL_DUAL_X_SET_OUT, r7, 0
        SET r0, EXT1_STEP_CTL_OFFSET
RESUME_NORMAL_DUAL_X_SET_OUT:

RESUME_UP_Y:
    QBEQ RESUME_GEN_STEP, r10, 0
    SUB r10, r10, 1
    SET r0, STEP_Y

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC RESUME_NORMAL_DUAL_Y_SET_OUT, r7, 0
        SET r0, EXT2_STEP_CTL_OFFSET
RESUME_NORMAL_DUAL_Y_SET_OUT:


RESUME_GEN_STEP:
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC RESUME_NORMAL_DUAL_XY_CLR_OUT, r7, 0 
        CLR r0, EXT1_STEP_CTL_OFFSET 
        CLR r0, EXT2_STEP_CTL_OFFSET 
RESUME_NORMAL_DUAL_XY_CLR_OUT:

    SBBO r0, r5, 4, 4

    DELAY_NS r3
    
    QBNE RESUME_STEP, r8, 0
    QBNE RESUME_STEP, r10, 0
    
    ;; Change state to print
    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBNE RESUME_CH_STATE, r1, STATE_STOP
    ;;If state change to stop, irq ARM
    ;;MOV  R31.b0, PRU0_ARM_IRQ + 16
RESUME_CH_STATE:
    MOV  r10, STATE_PRINT
    SBCO r10, CONST_PRUDRAM, r0, 4
    JMP MAIN

;;------------------------------------------------------------
;; Homing
;;------------------------------------------------------------
HOMING:
#ifdef NORMAL_HOMING 
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

    #ifdef DUAL_Z
        MOV r7, r6 
        MOV r8, GPIO_3_BASE_2 
        ADD r7, r7, r8       ;;GPIO3_DATAOUT
        LBBO r1, r7, 4, 4  
        UpdateDir r1, r10, r3, AXIS_Z, DIR_U
        SBBO r1, r7, 4, 4
    #endif 


    ;; Get homing_time
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 12
    LBCO r3, CONST_PRUDRAM, r0, 4

HOME_STEP:
    ;; Get homing_axis
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 4
    LBCO r10, CONST_PRUDRAM, r0, 4

    ;; generate homing step
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
    LBBO r11, r6, 0, 4  ;;GPIO2_DATAIN

CH_MIN_X:
    QBBC CH_MIN_Y, r10, 0
    QBBC CH_MIN_Y, r11, MIN_X
    SET r0, STEP_X

CH_MIN_Y:
    QBBC CH_MIN_Z, r10, 1
    QBBC CH_MIN_Z, r8, MIN_Y
    SET r0, STEP_Y

CH_MIN_Z:
    QBBS CH_CONTINUE_MIN_Z, r7, AUTO_LEVEL_Z 
    CLR r10, 2
    jmp HOMING_DONE

CH_CONTINUE_MIN_Z:
    QBBC CH_LMSW_OUT, r10, 2
    QBBC CH_LMSW_OUT, r7, MIN_Z
    SET r0, STEP_Z

    #ifdef DUAL_Z
        SET r0, STEP_U
    #endif

CH_LMSW_OUT:
    ;; gen step signal
    SBBO r0, r5, 4, 4
    DELAY_NS r3

    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z

    #ifdef DUAL_Z
        CLR r0, STEP_U
    #endif

    SBBO r0, r5, 4, 4
    DELAY_NS r3

CH_MIN_X_DONE:    
    QBBC CH_MIN_Y_DONE, r10, 0
    QBBS HOME_STEP, r11, MIN_X
    CLR r10, 0

CH_MIN_Y_DONE:
    QBBC CH_MIN_Z_DONE, r10, 1
    QBBS HOME_STEP, r8, MIN_Y
    CLR r10, 1

CH_MIN_Z_DONE:
    QBBC HOMING_DONE, r10, 2
    QBBS HOME_STEP, r7, MIN_Z
    CLR r10, 2

HOMING_DONE:
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
PRINT:
    ;; Read next element from the ring-buffer
    ;;.assign QueueHeader, r3, r3, header
    LBCO header, CONST_PRUDRAM, r2, SIZE(header)
    QBNE PRINT_DUAL_Z_1, header.state, STATE_EMPTY
    jmp MAIN

PRINT_DUAL_Z_1:
    ;; Get travel parameters
    ADD r8, r2, SIZE(QueueHeader)
    LBCO move, CONST_PRUDRAM, r8, SIZE(move)

    QBBC NORMAL_GCODE, header.type, BLOCK_M_CMD_BIT

    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 52
    LBCO r7, CONST_PRUDRAM, r8, 4 
    ADD r7, r7, 1
    SBCO r7, CONST_PRUDRAM, r8, 4 

    jmp  DONE_STEP_GEN

NORMAL_GCODE:

    ;; Load DIR GPIO value
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    
    ;; Set direction bits
    UpdateDir r1, r8, header.dir_bits, AXIS_X, DIR_X
    UpdateDir r1, r8, header.dir_bits, AXIS_Y, DIR_Y
    UpdateDir r1, r8, header.dir_bits, AXIS_Z, DIR_Z

    ;; Output direction bits to GPIO
    SBBO r1, r6, 4, 4

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC NORMAL_DUAL_XY_DIR_OUT, r7, 0 
        MOV  r9, EXT_STEP_DIR_GPIO  ;;ext1 --> X 
        LBBO r7, r9, 0, 4  
        UpdateDir r7, r8, header.dir_bits, AXIS_X, EXT1_STEP_DIR_OFFSET 
        SBBO r7, r9, 0, 4

        MOV  r9, EXT2_STEP_DIR_GPIO  ;;ext2 --> Y 
        LBBO r7, r9, 0, 4  
        UpdateDir r7, r8, header.dir_bits, AXIS_Y, EXT2_STEP_DIR_OFFSET 
        SBBO r7, r9, 0, 4
NORMAL_DUAL_XY_DIR_OUT:

    #ifdef DUAL_Z
        MOV r7, r6 
        MOV r8, GPIO_3_BASE_2 
        ADD r7, r7, r8       ;;GPIO3_DATAOUT
        LBBO r1, r7, 4, 4  
        UpdateDir r1, r8, header.dir_bits, AXIS_Z, DIR_U
        SBBO r1, r7, 4, 4
    #endif 


    MOV  r9, EXT_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER1_DIR, r9.t0 
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT0_STEP_DIR_OFFSET 
        MOV  r9, EXT_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
 NORMAL_ACTIVE_EXTRUDER1_DIR:   
    MOV  r9, EXT_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER2_DIR, r9.t1
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT1_STEP_DIR_OFFSET 
        MOV  r9, EXT_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
 NORMAL_ACTIVE_EXTRUDER2_DIR:   
    MOV  r9, EXT2_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER_DIR_OUT, r9.t2 
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT2_STEP_DIR_OFFSET 
        MOV  r9, EXT2_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER_DIR_OUT:   


    MOV r1, move.steps_count
    MOV r8, move.steps_count
    LSR r8, r8, 1
.using Print_counter_Scope
    MOV counter.x, r8
    MOV counter.y, r8 
    MOV counter.z, r8 
    MOV counter.e, r8


STEP_GEN:

;; check limit switch
    MOV r7, QUEUE_SIZE
    ADD r7, r7, 8
    LBCO r8, CONST_PRUDRAM, r7, 4 ;; home_dir
    
    XOR r7, r8, header.dir_bits
    
NORMAL_HIT_CHECK_X:
    QBBS NORMAL_HIT_CHECK_Y, r7, AXIS_X_BIT
    QBEQ NORMAL_HIT_CHECK_Y, move.steps_x, 0
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS NORMAL_HIT_CHECK_X_HIGHT_1, r8, 0 
        LBBO r8, r6, 0, 4  ;;GPIO2_DATAIN
        QBBS NORMAL_HIT_CHECK_Y, r8, MIN_X
        jmp NORMAL_HIT_CHECK_X_HIGHT_1_OUT
NORMAL_HIT_CHECK_X_HIGHT_1:
        LBBO r8, r6, 0, 4  ;;GPIO2_DATAIN
        QBBC NORMAL_HIT_CHECK_Y, r8, MIN_X
NORMAL_HIT_CHECK_X_HIGHT_1_OUT:
    MOV r1, 0
    MOV move.steps_x, 0

    ;jmp NORMAL_HIT
    jmp  DONE_STEP_GEN

NORMAL_HIT_CHECK_Y:
    QBBS NORMAL_HIT_CHECK_AUTO_LEVEL_Z, r7, AXIS_Y_BIT
    QBEQ NORMAL_HIT_CHECK_AUTO_LEVEL_Z, move.steps_y, 0
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS NORMAL_HIT_CHECK_Y_HIGHT_1, r8, 1 
        LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
        QBBS NORMAL_HIT_CHECK_AUTO_LEVEL_Z, r8, MIN_Y
        jmp NORMAL_HIT_CHECK_Y_HIGHT_1_OUT
NORMAL_HIT_CHECK_Y_HIGHT_1:
        LBBO r8, r5, 0, 4  ;;GPIO0_DATAIN
        QBBC NORMAL_HIT_CHECK_AUTO_LEVEL_Z, r8, MIN_Y
NORMAL_HIT_CHECK_Y_HIGHT_1_OUT:
    MOV r1, 0
    MOV move.steps_y, 0

    ;jmp NORMAL_HIT
    jmp  DONE_STEP_GEN

NORMAL_HIT_CHECK_AUTO_LEVEL_Z:
    QBBS NORMAL_HIT_CHECK_Z, r7, AXIS_Z_BIT
    QBEQ NORMAL_HIT_CHECK_Z, move.steps_z, 0
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS NORMAL_HIT_CHECK_AUTOLEVEL_HIGHT_1, r8, 3 
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBS NORMAL_HIT_CHECK_Z, r8, AUTO_LEVEL_Z
        jmp NORMAL_HIT_CHECK_AUTOLEVEL_HIGHT_1_OUT
NORMAL_HIT_CHECK_AUTOLEVEL_HIGHT_1:
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBC NORMAL_HIT_CHECK_Z, r8, AUTO_LEVEL_Z 
NORMAL_HIT_CHECK_AUTOLEVEL_HIGHT_1_OUT:
    MOV r1, 0
    MOV move.steps_z, 0

    ;jmp NORMAL_HIT
    jmp  DONE_STEP_GEN

NORMAL_HIT_CHECK_Z:
    QBBS NORMAL_NOT_HIT_STEP_GEN, r7, AXIS_Z_BIT
    QBEQ NORMAL_NOT_HIT_STEP_GEN, move.steps_z, 0
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS NORMAL_HIT_CHECK_Z_HIGHT_1, r8, 2 
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBS NORMAL_NOT_HIT_STEP_GEN, r8, MIN_Z
        jmp NORMAL_HIT_CHECK_Z_HIGHT_1_OUT
NORMAL_HIT_CHECK_Z_HIGHT_1:
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBC NORMAL_NOT_HIT_STEP_GEN, r8, MIN_Z
NORMAL_HIT_CHECK_Z_HIGHT_1_OUT:
    MOV r1, 0
    MOV move.steps_z, 0

    ;jmp NORMAL_HIT
    jmp  DONE_STEP_GEN

NORMAL_HIT:
NORMAL_NOT_HIT_STEP_GEN:

    ;; Load STEP gpio data 
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    ;; Generate motor step high voltage
    MOV r8, counter.x 
    UpdateStep r0, counter.x, move.steps_x, move.steps_count, STEP_X
    UpdateStep r0, counter.y, move.steps_y, move.steps_count, STEP_Y
    UpdateDualStep r0, counter.z, move.steps_z, move.steps_count, STEP_Z, STEP_U

    SBBO r0, r5, 4, 4


    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC NORMAL_DUAL_XY_SET_OUT, r7, 0 
        MOV  r9, EXT_STEP_CTL_GPIO  ;;ext1 --> X 
        LBBO r7, r9, 0, 4  
        MOV  r0, r8
        UpdateStep r7, r0, move.steps_x, move.steps_count, EXT1_STEP_CTL_OFFSET 
        SBBO r7, r9, 0, 4

        MOV  r9, EXT_STEP_CTL_GPIO  ;;ext2 --> Y 
        LBBO r7, r9, 0, 4  
        MOV  r0, r8
        UpdateStep r7, r0, move.steps_y, move.steps_count, EXT2_STEP_CTL_OFFSET 
        SBBO r7, r9, 0, 4
NORMAL_DUAL_XY_SET_OUT:


    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER1_CTL_SET, r9.t0
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT0_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER1_CTL_SET:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER2_CTL_SET, r9.t1
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT1_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER2_CTL_SET:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER_CTL_SET_OUT, r9.t2
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT2_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER_CTL_SET_OUT:   

    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    ;; Update current pos
UP_POS_X:
    QBBC UP_POS_Y, r0, STEP_X
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 16

    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_X
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_Y:
    QBBC UP_POS_Z, r0, STEP_Y
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 20
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_Y
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_Z:
    QBBC UP_POS_E, r0, STEP_Z
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 24
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_Z
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_E:
    QBBC UP_POS_DONE, r0, STEP_E
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 28
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_E
    SBCO r9, CONST_PRUDRAM, r8, 4
UP_POS_DONE:

    ;; Calc Delay time for speed control
    CalculateDelay r8, move
    ;;CalculateDelay_T r8

    DELAY_NS r8 

    ;; Generate motor step low voltage
    CLR r0, STEP_X
    CLR r0, STEP_Y
    CLR r0, STEP_Z

    #ifdef DUAL_Z
        CLR r0, STEP_U
    #endif

    SBBO r0, r5, 4, 4

    ;; motor56 mode
    MOV  r9, QUEUE_SIZE
    ADD  r9, r9, 68
    LBCO r7, CONST_PRUDRAM, r9, 4 
    QBBC NORMAL_DUAL_XY_CLR_OUT, r7, 0 
        MOV  r9, EXT_STEP_CTL_GPIO  ;;ext1 --> X 
        LBBO r7, r9, 0, 4  
        CLR r7, EXT1_STEP_CTL_OFFSET 
        SBBO r7, r9, 0, 4

        MOV  r9, EXT_STEP_CTL_GPIO  ;;ext2 --> Y 
        LBBO r7, r9, 0, 4  
        CLR r7, EXT2_STEP_CTL_OFFSET 
        SBBO r7, r9, 0, 4
NORMAL_DUAL_XY_CLR_OUT:


    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER1_CTL_CLR, r9.t0
        CLR  r7, EXT0_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER1_CTL_CLR:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER2_CTL_CLR, r9.t1
        CLR  r7, EXT1_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER2_CTL_CLR:
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC NORMAL_ACTIVE_EXTRUDER_CTL_CLR_OUT, r9.t2
        CLR  r7, EXT2_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
NORMAL_ACTIVE_EXTRUDER_CTL_CLR_OUT:   


    DELAY_NS r8 

    ;; Check all step done
    SUB r1, r1, 1
    QBEQ DONE_STEP_GEN, r1, 0

    ;;check queue state 
    MOV  r8, QUEUE_SIZE
    LBCO r9, CONST_PRUDRAM, r8, 4
    QBEQ DONE_STEP_GEN, r9, STATE_STOP

    JMP STEP_GEN
.leave Print_counter_Scope

DONE_STEP_GEN:
    ;; Make slot as empty
    MOV  header.state, STATE_EMPTY 
    SBCO header.state, CONST_PRUDRAM, r2, 1

IRQ:
    ;; Send IRQ to ARM
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 44
    LBCO r3, CONST_PRUDRAM, r0, 4
    QBNE IRQ_OUT, r3, r2
    MOV  R31.b0, PRU0_ARM_IRQ + 16
IRQ_OUT:

    ;; Next position in ring buffer
    ADD  r2, r2, QUEUE_ELEMENT_SIZE
    MOV  r1, QUEUE_LEN * QUEUE_ELEMENT_SIZE

    QBGE DUAL_Z_PRINT_END_1, r1, r2
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
	jmp MAIN
DUAL_Z_PRINT_END_1:
    ZERO &r2, 4
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
    JMP  MAIN
.leave Print_Scope


