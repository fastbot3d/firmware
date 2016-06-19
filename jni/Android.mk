LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

SRCS = common.c \
	   pwm.c \
	   analog.c \
	   thermistor.c \
	   temp.c \
	   fan.c \
	   heater.c \
	   servo.c \
	   lmsw.c \
	   stepper.c \
	   stepper_pruss.c \
	   pruss.c \
	   unicorn.c \
	   planner.c \
	   motion.c \
	   gcode.c \
	   eeprom.c \
	   sdcard.c \
	   parameter.c \
	   test.c \
	   mcode_list.c \
	   printer.c

SRCS += qr_solve.c \
		vector.c

SRCS += util/Pause.c \
		util/Fifo.c

src_path_files = $(addprefix ../unicorn/, /$(SRCS))

LOCAL_MODULE    := unicorn_android 
LOCAL_SRC_FILES := $(src_path_files)
LOCAL_C_INCLUDES += ../unicorn/. \
                    ../unicorn/include/.  \
					../../drivers/stepper  \
					../pru_sw/include


DEBUG_FLAGS ?= 0x0000
LOCAL_CFLAGS += -O3 -DD_INIT="$(DEBUG_FLAGS)" 
LOCAL_LDLIBS :=  -lm   -lprussdrv
#LOCAL_SHARED_LIBRARIES += libprussdrv
LOCAL_LDFLAGS += -L../pru_sw/src/libs/armeabi-v7a/

include $(BUILD_EXECUTABLE)

