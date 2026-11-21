# SPDX-License-Identifier: Apache-2.0
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := gralloc.pxa988
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_TAGS := optional

LOCAL_CPP_EXTENSION := .cc

LOCAL_SRC_FILES := \
	drv.c \
	etnaviv.c \
	helpers.c \
	helpers_array.c \
	cros_gralloc/cros_gralloc_buffer.cc \
	cros_gralloc/cros_gralloc_driver.cc \
	cros_gralloc/cros_gralloc_helpers.cc \
	cros_gralloc/gralloc0/gralloc0.cc

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH) \
	external/libdrm \
	external/libdrm/include/drm \
	frameworks/native/libs/nativewindow/include \
	frameworks/native/libs/arect/include

LOCAL_HEADER_LIBRARIES := \
	libhardware_headers \
	libnativebase_headers \
	libsystem_headers

LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libdrm \
	libhardware \
	liblog \
	libnativewindow \
	libsync

LOCAL_STATIC_LIBRARIES := libarect

LOCAL_CFLAGS := \
	-D_GNU_SOURCE=1 \
	-D_FILE_OFFSET_BITS=64 \
	-DDRV_ETNAVIV \
	-Wall \
	-Wsign-compare \
	-Wpointer-arith \
	-Wcast-qual \
	-Wcast-align \
	-Wno-unused-parameter

LOCAL_CPPFLAGS := -std=c++14

include $(BUILD_SHARED_LIBRARY)
