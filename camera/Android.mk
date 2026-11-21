# SPDX-License-Identifier: GPL-3.0-or-later
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS := \
	-fno-short-enums \
	-DHAVE_CONFIG_H \
	-Wno-unused-parameter \
	-Wno-unused-variable \
	-Wno-sign-compare \
	-Wno-missing-field-initializers \
	-Wno-format

LOCAL_C_INCLUDES := \
	system/media/camera/include

LOCAL_HEADER_LIBRARIES := \
	libhardware_headers

LOCAL_SRC_FILES := \
	CameraFactory.cpp \
	CameraHal.cpp \
	CameraHardware.cpp \
	Converter.cpp \
	SurfaceDesc.cpp \
	SurfaceSize.cpp \
	Utils.cpp \
	V4L2Camera.cpp

LOCAL_SHARED_LIBRARIES := \
	libcamera_client \
	libcutils \
	libjpeg \
	liblog \
	libui \
	libutils \
	libyuv

LOCAL_MODULE := camera.pxa988
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)
