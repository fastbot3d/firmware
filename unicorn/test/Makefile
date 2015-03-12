#
# project source top directory 
#
PRJROOT := $(word 1,$(subst unicorn,unicorn ,$(shell pwd)))
include $(PRJROOT)/build/config.mk

# this module
THISMODULE = unicorn_test

# source files under this folder
SRCS= main.c

# source code of sub-module
SRCS += 

# sub folders under this folder
SUBDIRS = 

LOCAL_DEFINES = 
LOCAL_CFLAGS = -g $(LOCAL_DEFINES) -I. -I$(PRJROOT)/output/host/usr/include

#require static libs only in /output/usr/lib
REQUIRE_LIBS = -lunicorn -lc -lpthread -lm -lrt

# if this module need to be built as static lib, shared lib, or executable?
TO_BUILD_STATIC_LIB := 
TO_BUILD_SHARED_LIB := 
TO_BUILD_EXECUTABLE := 1

# which files need to be install in the root filesystem
INSTALL_HEADERS =
INSTALL_LIBS    = 
INSTALL_BIN     = 1 

# if this module need a simple test program, add these below
TEST_SUBDIRS =

include $(PRJROOT)/build/rules.mk
