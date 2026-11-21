# SPDX-License-Identifier: Apache-2.0
ifndef PXA_COMMON_MK_INCLUDED
PXA_COMMON_MK_INCLUDED := true

LOCAL_PATH := $(call my-dir)

ifeq ($(TARGET_BOARD_PLATFORM),pxa988)
include $(call all-makefiles-under,$(LOCAL_PATH))
endif

endif
