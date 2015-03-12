SHELL := /bin/sh

# Do not use make's built-in rules and variables
MAKEFLAGS += -rR

.SUFFIXES:.c .o .h .cpp .cxx .cc .C
.DELETE_ON_ERROR:

ifneq (, $(strip $(CROSS_COMPILE)))
PREFIX      :=$(PRJROOT)/output/target
BUILD_DIR 	:=$(PRJROOT)/build/target
else
PREFIX      :=$(PRJROOT)/output/host
BUILD_DIR 	:=$(PRJROOT)/build/host
endif

INCLUDES_DIR :=$(PREFIX)/usr/include
LIBRARYS_DIR :=$(PREFIX)/usr/lib
BINARYS_DIR  :=$(PREFIX)/usr/bin

OBJ_DIR 	:=$(BUILD_DIR)/obj
BIN_DIR 	:=$(BUILD_DIR)/bin
SHARED_LIB_DIR :=$(BUILD_DIR)/shared_lib
STATIC_LIB_DIR :=$(BUILD_DIR)/static_lib

AR 			:= $(CROSS_COMPILE)ar
ARFLAGS 	:=
BISON 		:= 
CC 			:= $(CROSS_COMPILE)gcc
CPP 		:= $(CROSS_COMPILE)g++
CFLAGS 		:= -Wall -g -I$(INCLUDES_DIR)
CPPFLAGS	:=
INSTALL 	:= 
LD 			:=
LDCONFIG 	:=
LDFLAGS 	:= -L$(LIBRARYS_DIR)
MAKE 		:= make
MAKEINFO 	:=
RANLIB 		:= $(CROSS_COMPILE)ranlib

CHGRP 		:= chgrp
CHMOD 		:= chmod
CHOWN 		:= chown
MKNOD 		:= mknod

CFLAGS 		+= $(LOCAL_CFLAGS)
CPPFLAGS 	+= 
LDFLAGS 	+= $(LOCAL_LDFLAGS)
OBJS 		+= $(SRCS:.c=.o) $(SRCS:.cpp=.o)

THIS_MODULE_DIR := $(shell pwd)
THIS_MODULE_OBJ_DIR := $(patsubst $(PRJROOT)%,$(OBJ_DIR)%,$(THIS_MODULE_DIR))
THIS_MODULE_OBJS 	:= $(patsubst %.c,$(THIS_MODULE_OBJ_DIR)/%.o,$(SRCS))
THIS_MODULE_REQUIRE_LIBS := $(patsubst %.a,$(PRJROOT)/output/target/usr/lib/%.a,$(REQUIRE_LIBS))

STATIC_LIB := 
ifeq ($(TO_BUILD_STATIC_LIB),1)
STATIC_LIB := $(STATIC_LIB_DIR)/lib$(THISMODULE).a
endif

SHARED_LIB := 
ifeq ($(TO_BUILD_SHARED_LIB),1)
SHARED_LIB := $(SHARED_LIB_DIR)/lib$(THISMODULE).so
CFLAGS 	+= -fPIC
endif

EXECUTABLE := 
ifeq ($(TO_BUILD_EXECUTABLE),1)
EXECUTABLE := $(BIN_DIR)/$(THISMODULE)
endif

.PHONY: all clean install uninstall

all:$(THIS_MODULE_OBJS) $(STATIC_LIB) $(SHARED_LIB) $(EXECUTABLE) 

$(STATIC_LIB):$(THIS_MODULE_OBJS) 
	@echo '------ Build static library ------'
	@echo '    $(STATIC_LIB)'
	@echo '----------------------------------'
	@mkdir -p $(dir $@)
	@-rm -rf $(STATIC_LIB);
	$(AR) rcs $@ `find $(THIS_MODULE_OBJ_DIR) -name '*.o'`

$(SHARED_LIB):$(THIS_MODULE_OBJS) 
	@echo '------ Build shared library ------'
	@echo '    $(SHARED_LIB)'
	@echo '----------------------------------'
	@mkdir -p $(dir $@)
	@-rm -rf $(SHARED_LIB);
	$(CC) -shared $(LDFLAGS) -o $@ `find $(THIS_MODULE_OBJ_DIR) -name '*.o'`
	
$(EXECUTABLE):$(THIS_MODULE_OBJS) 
	@echo '------ Build executable ----------'
	@echo '    $(EXECUTABLE)'
	@echo '----------------------------------'
	@mkdir -p $(dir $@)
	@-rm -rf $(EXECUTABLE)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ `find $(THIS_MODULE_OBJ_DIR) -name '*.o'` $(THIS_MODULE_REQUIRE_LIBS)

clean:
	@echo '------ Clean up ------------------'
	@echo '    $(THIS_MODULE_OBJ_DIR)'
	@echo '----------------------------------'
	rm -rf $(THIS_MODULE_OBJ_DIR)/*

install:
	@echo '------ Install -------------------'
	@echo '    $(THISMODULE)'
	@echo '----------------------------------'
ifneq (, $(strip $(INSTALL_HEADERS)))
	@mkdir -p $(INCLUDES_DIR)/$(THISMODULE)/
	cp -rf $(INSTALL_HEADERS) $(INCLUDES_DIR)/$(THISMODULE)/
endif

ifneq (, $(strip $(INSTALL_LIBS)))
	@mkdir -p $(LIBRARYS_DIR)
    ifeq ($(TO_BUILD_SHARED_LIB),1)
	    cp -rf $(SHARED_LIB) $(LIBRARYS_DIR)
    endif
    ifeq ($(TO_BUILD_STATIC_LIB),1)
	    cp -rf $(STATIC_LIB) $(LIBRARYS_DIR)
    endif
endif

ifneq (, $(strip $(INSTALL_BIN)))
	@mkdir -p $(BINARYS_DIR)
	cp -rf $(EXECUTABLE) $(BINARYS_DIR)
endif

uninstall: 
ifneq (, $(strip $(INSTALL_HEADERS)))
	rm -rf $(INCLUDES_DIR)/$(THISMODULE)/
endif

ifneq (, $(strip $(INSTALL_LIBS)))
    ifeq ($(TO_BUILD_SHARED_LIB),1)
	    rm -rf $(LIBRARYS_DIR)/lib$(THISMODULE).so
    endif
    ifeq ($(TO_BUILD_STATIC_LIB),1)
	    rm -rf $(LIBRARYS_DIR)/lib$(THISMODULE).a
    endif
endif

ifneq (, $(strip $(INSTALL_BIN)))
	rm -rf $(BINARYS_DIR)/$(THISMODULE)
endif

$(THIS_MODULE_OBJ_DIR)/%.o:%.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -o $@ $<

#
# eg. main.o main.d : main.c main.h sub.h stdio.h
#
$(THIS_MODULE_OBJ_DIR)/%.d: %.c
	@mkdir -p $(dir $@)
	@set -e;rm -f $@;\
	$(CC) $(CFLAGS) -MM $(CPPFLAGS) $< > $@.$$$$;\
		sed 's,\($*\)\.o[:]*,\1.o $@ :,g' < $@.$$$$ > $@;\
		rm -f $@.$$$$

#
# only appear after the final target 
#
sinclude $(THIS_MODULE_OBJS:.o=.d)
