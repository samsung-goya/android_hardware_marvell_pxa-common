# SPDX-License-Identifier: GPL-3.0-or-later
#
# OpenMAX IL Hardware VPU Plugin (libstagefrighthw) for Chips&Media CODA7542
# (Linux 4.19 stateful V4L2 M2M driver coda.ko on /dev/video[1-4])

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libstagefrighthw
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := \
    CodaV4L2Plugin.cpp

LOCAL_HEADER_LIBRARIES := \
    media_plugin_headers \
    libhardware_headers

LOCAL_C_INCLUDES := \
    external/libdrm/include \
    external/libdrm/include/drm \
    external/libdrm/tests/etnaviv

LOCAL_SHARED_LIBRARIES := \
    libbase \
    libcutils \
    liblog \
    libmedia_omx \
    libstagefright_foundation \
    libstagefright_omx \
    libui \
    libutils

LOCAL_CFLAGS := \
    -Wall \
    -Wextra \
    -Werror \
    -Wno-unused-parameter

include $(BUILD_SHARED_LIBRARY)
