# SPDX-License-Identifier: Apache-2.0
# Linux rtnetlink-backed Wi-Fi HAL metadata backend for the PXA988 nl80211 driver.
# Android 9's default Wi-Fi HIDL service still consumes the legacy HAL function
# table; wificond handles scan, association, and nl80211 control.

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libwifi-hal-emu
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
LOCAL_CFLAGS := -Wall -Wextra -Werror
LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/hal \
    $(call include-path-for, libhardware_legacy)/hardware_legacy
LOCAL_SRC_FILES := \
    hal/halstate.cpp \
    hal/info.cpp \
    hal/interface.cpp \
    hal/netlink.cpp \
    hal/netlinkmessage.cpp \
    hal/wifi_hal.cpp
LOCAL_SHARED_LIBRARIES := liblog
LOCAL_HEADER_LIBRARIES := \
    libcutils_headers \
    libhardware_legacy_headers
include $(BUILD_STATIC_LIBRARY)

ifeq ($(WPA_SUPPLICANT_VERSION),VER_0_8_X)
WPA_SUPPL_DIR := external/wpa_supplicant_8
include $(WPA_SUPPL_DIR)/wpa_supplicant/android.config

include $(CLEAR_VARS)
LOCAL_MODULE := lib_driver_cmd_pxa988
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
LOCAL_SRC_FILES := wpa_supplicant_lib/driver_cmd_nl80211.c
LOCAL_C_INCLUDES := \
    $(WPA_SUPPL_DIR)/src \
    $(WPA_SUPPL_DIR)/src/common \
    $(WPA_SUPPL_DIR)/src/drivers \
    $(WPA_SUPPL_DIR)/src/l2_packet \
    $(WPA_SUPPL_DIR)/src/utils \
    $(WPA_SUPPL_DIR)/src/wps \
    $(WPA_SUPPL_DIR)/wpa_supplicant \
    external/libnl/include
LOCAL_CFLAGS := -DANDROID_P2P -DCONFIG_DRIVER_NL80211 -Wno-unused-parameter -Wno-macro-redefined
ifdef CONFIG_ANDROID_LOG
LOCAL_CFLAGS += -DCONFIG_ANDROID_LOG
endif
ifdef CONFIG_P2P
LOCAL_CFLAGS += -DCONFIG_P2P
endif
LOCAL_SHARED_LIBRARIES := libc libcutils
include $(BUILD_STATIC_LIBRARY)
endif
