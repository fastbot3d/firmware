#
# project source top directory 
#
PRJROOT := $(word 1,$(subst unicorn,unicorn ,$(shell pwd)))
#include $(PRJROOT)/build/config.mk
include ./build/config.mk

# this module
THISMODULE = unicorn

# source files under this folder
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

DEBUG_FLAGS ?= 0x0000
#DEBUG_FLAGS ?= 0x1024
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
INSTALL_HEADERS = 
INSTALL_LIBS    = 
INSTALL_BIN     = 

#include $(PRJROOT)/build/rules.mk
include ./build/rules.mk

# if this module need a simple test program, add these below
TEST_SUBDIRS = test

PASM?=../pru_sw/utils/pasm
ASM_FIlE_BBP1:=./pruss/bbp1/pruss_unicorn.p
ASM_FIlE_BBP1S:=./pruss/bbp1s/pruss_unicorn.p

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
	mkdir -p ./build/target/bin/
	gcc eeprom_bin.c -o  ./build/target/bin/eeprom
	./build/target/bin/eeprom ./build/target/bin/
	${PASM} -V3 -b ${ASM_FIlE_BBP1} build/target/bin/bbp1
	${PASM} -V3 -c -CBBP1_array ${ASM_FIlE_BBP1}  build/target/bin/bbp1
	${PASM} -V3 -b ${ASM_FIlE_BBP1S} build/target/bin/bbp1s
	${PASM} -V3 -c -CBBP1S_array ${ASM_FIlE_BBP1S} build/target/bin/bbp1s

