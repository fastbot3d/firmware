//------------------------------------------------------------
// pru_queue.p
// pru asm to generate step and dir motor control signals
//------------------------------------------------------------
#include "pru_queue.hp"

.origin 0
.entrypoint INIT

#define STATE_EMPTY    (0)
#define STATE_FILLED   (1)
#define STATE_EXIT     (2)

#define QUEUE_LEN      (64)

// In calculation of delay cycles: number of bits shifted
// for higher resolution.
#define DELAY_CYCLE_SHIFT 5

// GPIO-0 - output steps.
#define MOTOR_1_STEP_BIT  2
#define MOTOR_2_STEP_BIT  3
#define MOTOR_3_STEP_BIT  4
#define MOTOR_4_STEP_BIT  5
#define MOTOR_5_STEP_BIT  7

#define AUX_1_BIT 30
#define AUX_2_BIT 31

// GPIO-0 - input bits.
#define STOP_1_BIT 23
#define STOP_2_BIT 26
#define STOP_3_BIT 27

// GPIO-1 - the direction bits are mapped on GPIO-1, starting from bit 12
// as that is a contiguous region accessible as IO pins.
#define DIRECTION_GPIO1_SHIFT 12
#define MOTOR_ENABLE_GPIO1_BIT 28

#define QUEUE_ELEMENT_SIZE (SIZE(QueueHeader) + SIZE(TravelParameters))

#define PARAM_START r7
#define PARAM_END  r16

.struct TravelParameters
    // We do at most 2^16 loops to avoid accumulating too much rounding
    // error in the fraction addition. Longer moves are split into separate
    // requests by the host.
    .u16 loops_accel         ;; Phase 1: steps spent in acceleration.
    .u16 loops_travel        ;; Phase 2: steps spent in travel.
    .u16 loops_decel         ;; Phase 3: steps spent in deceleration.

    .u16 aux                 ;; lowest two bits only used.

    .u32 accel_series_index  ;; index into the taylor series.
    .u32 hires_accel_cycles  ;; initial delay cycles, for acceleration
                             ;; shifted by DELAY_CYCLE_SHIFT
                             ;; Changes in the different phases.
    .u32 travel_delay_cycles ;; Exact cycle value for travel (do not rely
                             ;; on approximation to exactly reach that)

    // 1.31 Fixed point increments for each motor
    .u32 fraction_1
    .u32 fraction_2          
    .u32 fraction_3
    .u32 fraction_4
    .u32 fraction_5
.ends

.struct QueueHeader
    .u8 state
    .u8 direction_bits
    .u16 reserved
.ends

// counter states of the motors
#define STATE_START r20    ;; after PARAM_END
#define STATE_END   r27
.struct MotorState
    .u32 m1
    .u32 m2
    .u32 m3
    .u32 m4
    .u32 m5
    .u32 m6
    .u32 m7
    .u32 m8
.ends

// Calculate the current delay depending on the phase (acceleration, travel,
// deceleration). Modifies the values in params, which is of type
// TravelParameters.
// Needs one state_register to keep its own state and two scratch registers.
// Outputs the resulting delay in output_reg.
// Returns special value 0 when done.
// Only used once, so just macro.
.macro CalculateDelay
.mparam output_reg, params, state_register, divident_tmp, divisor_tmp
    //TODO
    MOV output_reg, 10000
.endm

// Check limit switch state
.macro CheckLimitSwitches
.mparam out_reg, scratch, input_location
    LBBO scratch, input_location, 0, 4
    QBBS switch_1_handled, scratch, STOP_1_BIT
    ZERO &out_reg, 4                   ;; todo set proper value
switch_1_handled:
    QBBS switch_2_handled, scratch, STOP_2_BIT
    ZERO &out_reg, 4                   ;; todo set proper value
switch_2_handled:
    QBBS switch_3_handled, scratch, STOP_3_BIT
    ZERO &out_reg, 4                   ;; todo: set proper value
switch_3_handled:
.endm

// Update the state register of a motor with its 1.31 resolution fraction.
// The 31st bit contains the overflow that we are interested in
// Uses a fixed number of 4 cycles
.macro UpdateMotor
.mparam out_reg, scratch, state_reg, fraction, bit
    ADD state_reg, state_reg, fraction
    LSR scratch, state_reg, 31
    LSL scratch, scratch, bit
    OR  out_reg, out_reg, scratch
.endm

INIT:    
    // Clear STANDBY_INIT bit
    LBCO r0, C4, 4, 4                    
    CLR  r0, r0, 4
    SBCO r0, C4, 4, 4 

    // Queue address in PRU memory
    MOV r2, 0

    // Registers
    // r0, r1 free for calculation
    // r2 = queue pos
    // r3 = state for CalculateDelay
    // r4 = motor out GPIO
    // r5, r6 scratch
    // parameter:    r7..r16
    // motor-state:  r20..r27

    // Read next element from the ring-buffer
QUEUE_READ:
    // Check queue header at our read-position until it contains something.
    ;.assign QueueHeader, r1.w0, r1.w0, queue_header
    .assign QueueHeader, r1, r1, queue_header
    LBCO queue_header, CONST_PRUDRAM, r2, SIZE(queue_header)
    QBEQ QUEUE_READ, queue_header.state, STATE_EMPTY     ;; wait until got data.
     
    QBEQ FINISH, queue_header.state, STATE_EXIT
    
    // Output direction bits to GPIO
    ;MOV r3, queue_header.direction_bits
    ;LSL r3, r3, DIRECTION_GPIO1_SHIFT
    
    // queue_header processed
    ADD r1, r2, SIZE(QueueHeader)              ;; r2 stays at queue pos
    .assign TravelParameters, PARAM_START, PARAM_END, travel_params
    LBCO travel_params, CONST_PRUDRAM, r1, SIZE(travel_params)

    .assign MotorState, STATE_START, STATE_END, mstate
    ZERO &mstate, SIZE(mstate)

    MOV r4, GPIO_0 | GPIO_DATAOUT
    ZERO &r3, 4                                ;; initialize delay calculation state register.

STEP_GEN:
    // Generate motion profile configured by TravelParameters

    // update states and extract overflow bits into r1
    // 8 times 4 = 32 cpu cycles = 160ns. So whatever step output we do is
    // some time after we set the direction bit. Good, because the Allegro 
    // chip requests this time delay.
    ZERO &r1, 4
    LSL r1, travel_params.aux, AUX_1_BIT

    // UpdateMotor
    UpdateMotor r1, r5, mstate.m1, travel_params.fraction_1, MOTOR_1_STEP_BIT
    UpdateMotor r1, r5, mstate.m2, travel_params.fraction_2, MOTOR_2_STEP_BIT
    UpdateMotor r1, r5, mstate.m3, travel_params.fraction_3, MOTOR_3_STEP_BIT
    UpdateMotor r1, r5, mstate.m4, travel_params.fraction_4, MOTOR_4_STEP_BIT
    UpdateMotor r1, r5, mstate.m5, travel_params.fraction_5, MOTOR_5_STEP_BIT

    ;MOV r6, GPIO_0 | GPIO_DATAIN
    ;CheckLimitSwitches r1, r5, r6
    
    SBBO r1, r4, 0, 4                          ;; motor bits to GPIO-0

    CalculateDelay r1, travel_params, r3, r5, r6
    ;QBEQ DONE_STEP_GEN, r1, 0                  ;; special value 0: all steps consumed.

    // Create time delay between steps
STEP_DELAY:
    SUB r1, r1, 1
    QBNE STEP_DELAY, r1, 0
    
    ;JMP STEP_GEN

DONE_STEP_GEN:
    // We are done with instruction, Make slot as empty...
    MOV queue_header.state, STATE_EMPTY 
    SBCO queue_header.state, CONST_PRUDRAM, r2, 1
    MOV R31.b0, PRU0_ARM_INTERRUPT + 16        ;; Signal host program free slot.

    // Next position in ring buffer
    ADD r2, r2, QUEUE_ELEMENT_SIZE
    MOV r1, QUEUE_LEN * QUEUE_ELEMENT_SIZE     ;; end of queue
    QBLT QUEUE_READ, r1, r2
    ZERO &r2, 4
    JMP QUEUE_READ

FINISH:
    MOV queue_header.state, STATE_EMPTY
    SBCO queue_header.state, CONST_PRUDRAM, r2, 1
    MOV R31.b0, PRU0_ARM_INTERRUPT + 16

    HALT                                       ;; PRU Halt
