;;-----------------------------------
;;  CoreXY control 
;;------------------------------------------------------------
;; Pause
;;       Move xy to lmsw
;;------------------------------------------------------------
;; Check Control command : ST_CMD_PAUSE

PAUSE_COREXY:
.using ReturnAxis_Scope
;;X AXIS
    ;; load queue struct 
    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)
    MOV queue.pause_x, 0
    MOV queue.pause_y, 0

    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 72
    LBCO r9, CONST_PRUDRAM, r0, 4  ;; endstop invert

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    UpdateDir r1, r10,  queue.homing_dir, AXIS_X, DIR_X
    UpdateDir r1, r10,  queue.homing_dir, AXIS_Y, DIR_Y
    XOR queue.homing_dir, queue.homing_dir,  AXIS_Z
    UpdateDir r1, r10, queue.homing_dir, AXIS_Z, DIR_Z
    SBBO r1, r6, 4, 4

    MOV r7, r6 
    MOV r8, GPIO_3_BASE_2 
    ADD r7, r7, r8       ;;GPIO3_DATAOUT
    LBBO r1, r7, 4, 4
    UpdateDir r1, r10, queue.homing_dir, AXIS_Z, DIR_U
    SBBO r1, r7, 4, 4

    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 56
    LBCO r11, CONST_PRUDRAM, r0, 4  ;;lift z distance

    ;; generate homing step
PAUSE_MIN_X_WHILE_COREXY:
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT
    LBBO r7, r6, 0, 4  ;;GPIO2_DATAIN

    QBBS PAUSE_MIN_X_CHECK_HIGHT_COREXY_1, r9, 0 
        QBBC PAUSE_MIN_Y_START_COREXY, r7, MIN_X
        jmp PAUSE_MIN_X_CHECK_HIGHT_COREXY_1_OUT
PAUSE_MIN_X_CHECK_HIGHT_COREXY_1:
        QBBS PAUSE_MIN_Y_START_COREXY, r7, MIN_X
PAUSE_MIN_X_CHECK_HIGHT_COREXY_1_OUT:
    ADD queue.pause_x, queue.pause_x, 1
    SET r0, STEP_X
    SET r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_X
    CLR r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    JMP PAUSE_MIN_X_WHILE_COREXY

;;Y
PAUSE_MIN_Y_START_COREXY:
    ;; load queue struct
    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 32
    SBCO queue.pause_x, CONST_PRUDRAM, r0, 4

    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)
    MOV queue.pause_y, 0

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    UpdateDir r1, r10, queue.homing_dir, AXIS_X, DIR_X
    XOR queue.homing_dir, queue.homing_dir, 1<<1  //lkj
    UpdateDir r1, r10, queue.homing_dir, AXIS_Y, DIR_Y
    SBBO r1, r6, 4, 4

    ;; generate homing step
PAUSE_MIN_Y_WHILE_COREXY:
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN

    QBBS PAUSE_MIN_Y_CHECK_HIGHT_COREXY_1, r9, 1 
        QBBC COREXY_PAUSE_UP_Z_SET, r8, MIN_Y
        jmp PAUSE_MIN_Y_CHECK_HIGHT_COREXY_1_OUT
PAUSE_MIN_Y_CHECK_HIGHT_COREXY_1:
        QBBS COREXY_PAUSE_UP_Z_SET, r8, MIN_Y
PAUSE_MIN_Y_CHECK_HIGHT_COREXY_1_OUT:
    ADD queue.pause_y, queue.pause_y, 1
    SET r0, STEP_X
    SET r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_X
    CLR r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    JMP PAUSE_MIN_Y_WHILE_COREXY

;;Z
COREXY_PAUSE_UP_Z_SET:
    QBEQ PAUSE_XY_OUT_COREXY, r11, 0
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    SET r0, STEP_Z
    SET r0, STEP_U
    SBBO r0, r5, 4, 4
    SUB r11, r11, 1

    DELAY_NS queue.homing_time

    CLR r0, STEP_Z
    CLR r0, STEP_U
    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    QBNE COREXY_PAUSE_UP_Z_SET, r11, 0

;;out
PAUSE_XY_OUT_COREXY:
    MOV  r0, QUEUE_SIZE
    ADD  r0, r0, 32
    SBCO queue.pause_x, CONST_PRUDRAM, r0, 4
    ADD  r0, r0, 4
    SBCO queue.pause_y, CONST_PRUDRAM, r0, 4

    MOV  r0, QUEUE_SIZE
    MOV  r10, STATE_PAUSE_FINISH  
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBNE CH_STATE_COREXY, r1, STATE_STOP
    MOV r2, 0
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
CH_STATE_COREXY:
    SBCO r10, CONST_PRUDRAM, r0, 4
    ;;MOV  R31.b0, PRU0_ARM_IRQ + 16

.leave ReturnAxis_Scope
    JMP MAIN

;;------------------------------------------------------------
;; Resume:
;;        move back to the last position
;;------------------------------------------------------------
RESUME_COREXY:
.using ReturnAxis_Scope

;;X
    ;; load queue struct 
    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)

    MOV  r0, QUEUE_SIZE
    ADD r0, r0, 56
    LBCO r11, CONST_PRUDRAM, r0, 4  ;;move down z distance

    ;; generate direction
    NOT queue.homing_dir, queue.homing_dir

    LBBO r1, r6, 4, 4 ;;GPIO2_DATAOUT
    UpdateDir r1, r10, queue.homing_dir, AXIS_X, DIR_X
    UpdateDir r1, r10, queue.homing_dir, AXIS_Y, DIR_Y
    XOR  queue.homing_dir, queue.homing_dir, AXIS_Z
    UpdateDir r1, r10, queue.homing_dir, AXIS_Z, DIR_Z

    SBBO r1, r6, 4, 4

    MOV r7, r6 
    MOV r8, GPIO_3_BASE_2 
    ADD r7, r7, r8       ;;GPIO3_DATAOUT
    LBBO r1, r7, 4, 4
    UpdateDir r1, r10, queue.homing_dir, AXIS_Z, DIR_U
    SBBO r1, r7, 4, 4

COREXY_RESUME_UP_Z_SET:
    QBEQ RESUME_X_STEP_WHILE_COREXY, r11, 0
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    SET r0, STEP_Z
    SET r0, STEP_U
    SBBO r0, r5, 4, 4
    SUB r11, r11, 1

    DELAY_NS queue.homing_time

    CLR r0, STEP_Z
    CLR r0, STEP_U
    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    QBNE COREXY_RESUME_UP_Z_SET, r11, 0

RESUME_X_STEP_WHILE_COREXY:
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    
    QBEQ RESUME_Y_START_COREXY, queue.pause_x, 0
    SUB queue.pause_x, queue.pause_x, 1

    SET r0, STEP_X
    SET r0, STEP_Y
    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_X
    CLR r0, STEP_Y
    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time
    
    JMP RESUME_X_STEP_WHILE_COREXY

;;Y
RESUME_Y_START_COREXY:
    ;; load queue struct 
    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)

    NOT queue.homing_dir, queue.homing_dir

    ;; generate direction
    LBBO r1, r6, 4, 4 ;;GPIO2_DATAOUT
    UpdateDir r1, r10, queue.homing_dir, AXIS_X, DIR_X
    XOR queue.homing_dir, queue.homing_dir, 1<<1  //lkj
    UpdateDir r1, r10, queue.homing_dir, AXIS_Y, DIR_Y
    SBBO r1, r6, 4, 4


RESUME_Y_STEP_WHILE_COREXY:
    LBBO r0, r5, 4, 4 ;;GPIO1_DATAOUT
    
    QBEQ RESUME_XY_STEP_OUT_COREXY, queue.pause_y, 0
    SUB queue.pause_y, queue.pause_y, 1

    SET r0, STEP_X
    SET r0, STEP_Y
    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_X
    CLR r0, STEP_Y
    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time
    
    JMP RESUME_Y_STEP_WHILE_COREXY

RESUME_XY_STEP_OUT_COREXY:
    ;; Change state to print
    MOV  r0, QUEUE_SIZE
    LBCO r1, CONST_PRUDRAM, r0, 4
    QBNE RESUME_CH_STATE_COREXY, r1, STATE_STOP
    ;;If state change to stop, irq ARM
    ;;MOV  R31.b0, PRU0_ARM_IRQ + 16
RESUME_CH_STATE_COREXY:
    MOV  r10, STATE_PRINT
    SBCO r10, CONST_PRUDRAM, r0, 4

.leave ReturnAxis_Scope
    JMP MAIN

;;------------------------------------------------------------
;; Homing
;;------------------------------------------------------------
HOMING_COREXY:
#ifdef HOMING_COREXY
.using ReturnAxis_Scope
;;X AXIS
    ;; load queue struct 
    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)

    QBBC HOMING_MIN_Y_START_COREXY, queue.homing_axis, 0

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    UpdateDir r1, r10,  queue.homing_dir, AXIS_X, DIR_X
    UpdateDir r1, r10,  queue.homing_dir, AXIS_Y, DIR_Y
    SBBO r1, r6, 4, 4

    ;; generate homing step
HOMING_MIN_X_WHILE_COREXY:
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT
    LBBO r7, r6, 0, 4  ;;GPIO2_DATAIN

    QBBC HOMING_MIN_Y_START_COREXY, r7, MIN_X
    SET r0, STEP_X
    SET r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_X
    CLR r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    JMP HOMING_MIN_X_WHILE_COREXY

;;Y
HOMING_MIN_Y_START_COREXY:
    ;; load queue struct 
    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)

    QBBC HOMING_MIN_Z_START_COREXY, queue.homing_axis, 1

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    UpdateDir r1, r10, queue.homing_dir, AXIS_X, DIR_X
    XOR queue.homing_dir, queue.homing_dir, 1<<1 //lkj Y
    UpdateDir r1, r10, queue.homing_dir, AXIS_Y, DIR_Y
    SBBO r1, r6, 4, 4

    ;; generate homing step
HOMING_MIN_Y_WHILE_COREXY:
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT
    LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN

    QBBC HOMING_MIN_Z_START_COREXY, r8, MIN_Y
    SET r0, STEP_X
    SET r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_X
    CLR r0, STEP_Y

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    JMP HOMING_MIN_Y_WHILE_COREXY

;;Z
HOMING_MIN_Z_START_COREXY:
    ;; load queue struct 
    MOV  r0, QUEUE_SIZE
    LBCO queue, CONST_PRUDRAM, r0, SIZE(queue)

    QBBC HOMING_DONE_COREXY, queue.homing_axis, 2

    ;; generate direction
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT

    UpdateDir r1, r10, queue.homing_dir, AXIS_Z, DIR_Z
    SBBO r1, r6, 4, 4
    #ifdef DUAL_Z
        MOV r7, r6 
        MOV r8, GPIO_3_BASE_2 
        ADD r7, r7, r8       ;;GPIO3_DATAOUT
        LBBO r1, r7, 4, 4  
        UpdateDir r1, r10, queue.homing_dir, AXIS_Z, DIR_U
        SBBO r1, r7, 4, 4
    #endif


    ;; generate homing step
HOMING_MIN_Z_WHILE_COREXY:
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT
    LBBO r7, r4, 0, 4  ;;GPIO0_DATAIN

    QBBC HOMING_DONE_COREXY, r7, AUTO_LEVEL_Z 
    QBBC HOMING_DONE_COREXY, r7, MIN_Z
    SET r0, STEP_Z
    #ifdef DUAL_Z
        SET r0, STEP_U
    #endif

    SBBO r1, r6, 4, 4

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    CLR r0, STEP_Z
    #ifdef DUAL_Z
        CLR r0, STEP_U
    #endif

    SBBO r0, r5, 4, 4
    DELAY_NS queue.homing_time

    JMP HOMING_MIN_Z_WHILE_COREXY

;;out
HOMING_DONE_COREXY:
    ;; Clear homing_axis flag
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 4
    MOV  r10, 0
    SBCO r10, CONST_PRUDRAM, r0, 4

    ;; Change state to print
    MOV  r0,  QUEUE_SIZE
    MOV  r10, STATE_PRINT
    SBCO r10, CONST_PRUDRAM, r0, 4
    
.leave ReturnAxis_Scope
#endif
    JMP MAIN

;;------------------------------------------------------------
;; Queue Movement
;;------------------------------------------------------------
.using Print_Scope
PRINT_COREXY:
    ;; Read next element from the ring-buffer
    ;;.assign QueueHeader, r3, r3, header
    LBCO header, CONST_PRUDRAM, r2, SIZE(header)
    QBNE PRINT_COREXY_1, header.state, STATE_EMPTY
    jmp MAIN

PRINT_COREXY_1:
    ;; Get travel parameters
    ADD r8, r2, SIZE(QueueHeader)
    LBCO move, CONST_PRUDRAM, r8, SIZE(move)


    QBBC PRINT_COREXY_GCODE, header.type, BLOCK_M_CMD_BIT

    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 52
    LBCO r7, CONST_PRUDRAM, r8, 4
    ADD r7, r7, 1
    SBCO r7, CONST_PRUDRAM, r8, 4 

    jmp  DONE_STEP_GEN_COREXY

PRINT_COREXY_GCODE:

    ;; Load DIR GPIO value
    LBBO r1, r6, 4, 4  ;;GPIO2_DATAOUT
    
    ;; Set direction bits
    UpdateDir r1, r8, header.dir_bits, AXIS_X, DIR_X
    UpdateDir r1, r8, header.dir_bits, AXIS_Y, DIR_Y
    UpdateDir r1, r8, header.dir_bits, AXIS_Z, DIR_Z

    ;; Output direction bits to GPIO
    SBBO r1, r6, 4, 4

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
    QBBC COREXY_ACTIVE_EXTRUDER1_DIR, r9.t0 
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT0_STEP_DIR_OFFSET 
        MOV  r9, EXT_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
 COREXY_ACTIVE_EXTRUDER1_DIR:   
    MOV  r9, EXT_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER2_DIR, r9.t1
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT1_STEP_DIR_OFFSET 
        MOV  r9, EXT_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
 COREXY_ACTIVE_EXTRUDER2_DIR:   
    MOV  r9, EXT2_STEP_DIR_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER_DIR_OUT, r9.t2 
        UpdateDir r7, r8, header.dir_bits, AXIS_E, EXT2_STEP_DIR_OFFSET 
        MOV  r9, EXT2_STEP_DIR_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER_DIR_OUT:   

    MOV r1, move.steps_count
    MOV r8, move.steps_count
    LSR r8, r8, 1
.using Print_counter_Scope
    MOV counter.x, r8
    MOV counter.y, r8 
    MOV counter.z, r8 
    MOV counter.e, r8 


STEP_GEN_COREXY:

;; check limit switch
    MOV r7, QUEUE_SIZE
    ADD r7, r7, 8
    LBCO r8, CONST_PRUDRAM, r7, 4 ;; home_dir
    
    XOR r7, r8, header.dir_bits
    
COREXY_HIT_CHECK_X:
    QBBS COREXY_HIT_CHECK_Y, r7, AXIS_X_BIT
    QBEQ COREXY_HIT_CHECK_X_1, move.steps_y, 0     ;; diaganol line to origin
    QBBS COREXY_HIT_CHECK_Y, r7, AXIS_Y_BIT
COREXY_HIT_CHECK_X_1:
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS COREXY_HIT_CHECK_X_HIGHT_1, r8, 0 
        LBBO r8, r6, 0, 4  ;;GPIO2_DATAIN
        QBBS COREXY_HIT_CHECK_Y, r8, MIN_X
        jmp COREXY_HIT_CHECK_X_HIGHT_1_OUT
COREXY_HIT_CHECK_X_HIGHT_1:
        LBBO r8, r6, 0, 4  ;;GPIO2_DATAIN
        QBBC COREXY_HIT_CHECK_Y, r8, MIN_X
COREXY_HIT_CHECK_X_HIGHT_1_OUT:
    MOV move.steps_x, 0
    MOV move.steps_y, 0
    MOV r1, 0

    ;;jmp COREXY_HIT
    jmp  DONE_STEP_GEN_COREXY

COREXY_HIT_CHECK_Y:
    QBBS COREXY_HIT_CHECK_AUTO_LEVEL_Z, r7, AXIS_X_BIT
    QBBC COREXY_HIT_CHECK_AUTO_LEVEL_Z, r7, AXIS_Y_BIT
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS COREXY_HIT_CHECK_Y_HIGHT_1, r8, 1 
        LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
        QBBS COREXY_HIT_CHECK_AUTO_LEVEL_Z, r8, MIN_Y
        jmp COREXY_HIT_CHECK_Y_HIGHT_1_OUT
COREXY_HIT_CHECK_Y_HIGHT_1:
        LBBO r8, r5, 0, 4  ;;GPIO1_DATAIN
        QBBC COREXY_HIT_CHECK_AUTO_LEVEL_Z, r8, MIN_Y
COREXY_HIT_CHECK_Y_HIGHT_1_OUT:
    MOV move.steps_x, 0
    MOV move.steps_y, 0
    MOV r1, 0

    ;jmp COREXY_HIT
    JMP DONE_STEP_GEN_COREXY

COREXY_HIT_CHECK_AUTO_LEVEL_Z:
    QBBS COREXY_HIT_CHECK_Z, r7, AXIS_Z_BIT
    QBEQ COREXY_HIT_CHECK_Z, move.steps_z, 0
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS COREXY_HIT_CHECK_AUTOLEVEL_HIGHT_1, r8, 3 
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBS COREXY_HIT_CHECK_Z, r8, AUTO_LEVEL_Z 
        jmp COREXY_HIT_CHECK_AUTOLEVEL_HIGHT_1_OUT
COREXY_HIT_CHECK_AUTOLEVEL_HIGHT_1:
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBC COREXY_HIT_CHECK_Z, r8, AUTO_LEVEL_Z 
COREXY_HIT_CHECK_AUTOLEVEL_HIGHT_1_OUT:
    MOV move.steps_z, 0
    MOV r1, 0

    JMP DONE_STEP_GEN_COREXY

COREXY_HIT_CHECK_Z:
    QBBS COREXY_NOT_HIT_STEP_GEN, r7, AXIS_Z_BIT
    QBEQ COREXY_NOT_HIT_STEP_GEN, move.steps_z, 0
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 72  
    LBCO r8, CONST_PRUDRAM, r8, 4  ;; endstop invert
    QBBS COREXY_HIT_CHECK_Z_HIGHT_1, r8, 2 
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBS COREXY_NOT_HIT_STEP_GEN, r8, MIN_Z
        jmp COREXY_HIT_CHECK_Z_HIGHT_1_OUT
COREXY_HIT_CHECK_Z_HIGHT_1:
        LBBO r8, r4, 0, 4  ;;GPIO0_DATAIN
        QBBC COREXY_NOT_HIT_STEP_GEN, r8, MIN_Z
COREXY_HIT_CHECK_Z_HIGHT_1_OUT:
    MOV move.steps_z, 0
    MOV move.steps_x, 0
    MOV move.steps_y, 0
    MOV move.steps_z, 0
    MOV r1, 0

    JMP DONE_STEP_GEN_COREXY

COREXY_HIT:
COREXY_NOT_HIT_STEP_GEN:

    ;; Load STEP gpio data
    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    ;; Generate motor step high voltage
    UpdateStep r0, counter.x, move.steps_x, move.steps_count, STEP_X
    UpdateStep r0, counter.y, move.steps_y, move.steps_count, STEP_Y
    UpdateDualStep r0, counter.z, move.steps_z, move.steps_count, STEP_Z, STEP_U

    SBBO r0, r5, 4, 4

    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER1_CTL_SET, r9.t0
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT0_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER1_CTL_SET:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER2_CTL_SET, r9.t1
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT1_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER2_CTL_SET:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER_CTL_SET_OUT, r9.t2
        UpdateStep r7, counter.e, move.steps_e, move.steps_count, EXT2_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER_CTL_SET_OUT:   


    LBBO r0, r5, 4, 4  ;;GPIO1_DATAOUT

    ;; Update current pos
UP_POS_X_COREXY:
    QBBC UP_POS_Y_COREXY, r0, STEP_X
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 16

    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_X
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_Y_COREXY:
    QBBC UP_POS_Z_COREXY, r0, STEP_Y
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 20
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_Y
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_Z_COREXY:
    QBBC UP_POS_E_COREXY, r0, STEP_Z
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 24
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_Z
    SBCO r9, CONST_PRUDRAM, r8, 4

UP_POS_E_COREXY:
    QBBC UP_POS_DONE_COREXY, r0, STEP_E
    MOV  r8, QUEUE_SIZE
    ADD  r8, r8, 28
    LBCO r9, CONST_PRUDRAM, r8, 4
    UpdatePos r9, r7, header.dir, AXIS_E
    SBCO r9, CONST_PRUDRAM, r8, 4
UP_POS_DONE_COREXY:

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

    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER1_CTL_CLR, r9.t0
        CLR  r7, EXT0_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER1_CTL_CLR:   
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER2_CTL_CLR, r9.t1
        CLR  r7, EXT1_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER2_CTL_CLR:
    MOV  r9, EXT_STEP_CTL_GPIO
    LBBO r7, r9, 0, 4  ;;extruder dir gpio DATAOUT
    MOV  r9, move.ext_step_bit         ;; active extruder
    QBBC COREXY_ACTIVE_EXTRUDER_CTL_CLR_OUT, r9.t2
        CLR  r7, EXT2_STEP_CTL_OFFSET 
        MOV  r9, EXT_STEP_CTL_GPIO
        SBBO r7, r9, 0, 4
COREXY_ACTIVE_EXTRUDER_CTL_CLR_OUT:   



    DELAY_NS r8 

    ;; Check all step done
    SUB r1, r1, 1
    QBEQ DONE_STEP_GEN_COREXY, r1, 0

    ;;check queue state 
    MOV  r8, QUEUE_SIZE
    LBCO r9, CONST_PRUDRAM, r8, 4
    QBEQ DONE_STEP_GEN_COREXY, r9, STATE_STOP

    JMP STEP_GEN_COREXY
.leave Print_counter_Scope

DONE_STEP_GEN_COREXY:
    ;; Make slot as empty
    MOV  header.state, STATE_EMPTY 
    SBCO header.state, CONST_PRUDRAM, r2, 1

IRQ_COREXY:
    ;; Send IRQ to ARM
    MOV r0, QUEUE_SIZE
    ADD r0, r0, 44
    LBCO r3, CONST_PRUDRAM, r0, 4
    QBNE IRQ_OUT_COREXY, r3, r2
    MOV  R31.b0, PRU0_ARM_IRQ + 16
IRQ_OUT_COREXY:

    ;; Next position in ring buffer
    ADD  r2, r2, QUEUE_ELEMENT_SIZE
    MOV  r1, QUEUE_LEN * QUEUE_ELEMENT_SIZE

    QBGE COREXY_PRINT_END_1, r1, r2
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
	jmp MAIN
COREXY_PRINT_END_1:
    ZERO &r2, 4
    MOV r1, QUEUE_SIZE
    ADD r1, r1, 60
    SBCO r2, CONST_PRUDRAM, r1, 4
    JMP  MAIN
.leave Print_Scope


