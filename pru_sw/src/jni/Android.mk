LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := prussdrv
LOCAL_SRC_FILES := ../prussdrv.c 
#lkj LOCAL_CFLAGS += -O3 -mtune=cortex-a8 -march=armv7a
LOCAL_C_INCLUDES += ../../. 
LOCAL_C_INCLUDES += ../../include/ 
LOCAL_C_INCLUDES += ../../src/ 
LOCAL_CFLAGS += -O3 
LOCAL_LDLIBS :=  
include $(BUILD_SHARED_LIBRARY)

