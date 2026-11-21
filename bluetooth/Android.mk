# SPDX-License-Identifier: GPL-3.0-or-later
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := android.hardware.bluetooth@1.0-service.pxa988
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_TAGS := optional
LOCAL_INIT_RC := android.hardware.bluetooth@1.0-service.pxa988.rc

LOCAL_SRC_FILES := service.cpp

LOCAL_SHARED_LIBRARIES := \
	android.hardware.bluetooth@1.0 \
	libbase \
	libcutils \
	libhidlbase \
	libhidltransport \
	liblog \
	libutils

LOCAL_CFLAGS := -Wall -Werror -Wno-unused-parameter
LOCAL_CPPFLAGS := -std=c++14

include $(BUILD_EXECUTABLE)
