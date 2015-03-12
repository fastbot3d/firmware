#
# project source top directory 
#
PRJROOT := $(word 1,$(subst unicorn,unicorn ,$(shell pwd)))
include $(PRJROOT)/build/config.mk

# this module
THISMODULE = unicorn

# source files under this folder
SRCS = common.c \
	   pwm.c \
	   analog.c \
	   thermistor.c \
	   temp.c \
	   fan.c \
	   led.c \
	   heater.c \
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
	   printer.c

DEBUG_FLAGS ?= 0x0040
#DEBUG_FLAGS ?= 0xFFFF
#LOCAL_DEFINES = -g -DD_INIT="$(DEBUG_FLAGS)" -DHOST -DDEBUG
LOCAL_DEFINES = -g -DD_INIT="$(DEBUG_FLAGS)" -DDEBUG

LOCAL_CFLAGS = $(LOCAL_DEFINES) -I. -I../../drivers/stepper -I../pru_sw/include

#require static libs only in /output/usr/lib
REQUIRE_LIBS = -lc -lm -lrt -lpthread -lprussdrv -L../pru_sw/lib

# if this module need to be built as static lib, shared lib, or executable?
TO_BUILD_STATIC_LIB := 
TO_BUILD_SHARED_LIB := 
TO_BUILD_EXECUTABLE := 1

# which files need to be install in the root filesystem
INSTALL_HEADERS = printer.h
INSTALL_LIBS    = 
INSTALL_BIN     = 

include $(PRJROOT)/build/rules.mk

# if this module need a simple test program, add these below
TEST_SUBDIRS = test

PASM?=../pru_sw/utils/pasm
ASM_FIlE:=pruss_unicorn.p

.PHONY: test test_clean fw
test:
	for dir in $(TEST_SUBDIRS);do\
		$(MAKE) -C $$dir || exit 1;\
		$(MAKE) install -C $$dir || exit 1;\
	done

test_clean:
	for dir in $(TEST_SUBDIRS);do\
		$(MAKE) clean -C $$dir || exit 1;\
		$(MAKE) uninstall -C $$dir || exit 1;\
	done

fw:
	#${PASM} -V3 -b ${ASM_FIlE}
	${PASM} -V3 -c ${ASM_FIlE}

