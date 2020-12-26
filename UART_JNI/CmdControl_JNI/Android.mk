LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := libCmdControlJNI
LOCAL_MODULE_TAGS := optional

LOCAL_C_INCLUDES += system/core/include \
                    frameworks/base/include \
                    frameworks/base/core/jni \
                    MyUtils.h \
                    SerialCommand.h
                    
LOCAL_SRC_FILES := CmdControlJNI.cpp \
                   SerialCommand.cpp \
                   MyUtils.cpp

LOCAL_SHARED_LIBRARIES := libandroid_runtime \
                          libbinder \
                          libutils \
                          libcutils \
                          libnativehelper

LOCAL_LDLIBS := -llog

LOCAL_MULTILIB := 32
include $(BUILD_SHARED_LIBRARY)
