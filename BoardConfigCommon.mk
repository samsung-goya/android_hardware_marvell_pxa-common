# SPDX-License-Identifier: Apache-2.0
# Common BoardConfig settings for Marvell PXA988/PXA986 platform (Linux 4.19)

# DRM Hardware Composer (hardware/marvell/pxa-common/display/hwcomposer)
BOARD_USES_DRM_HWCOMPOSER := false

# Mesa3D GPU drivers (customized Mesa 21.1 with etnaviv + kmsro)
BOARD_USE_CUSTOMIZED_MESA := true
BOARD_GPU_DRIVERS := etnaviv kmsro

# Wi-Fi (mwifiex_sdio nl80211)
# Android 9's default Wi-Fi HIDL service requires a vendor HAL function table.
# The generic emulator backend supplies the interface/rtnetlink operations;
# wificond still performs Wi-Fi control over nl80211.
BOARD_WLAN_DEVICE := emulator
# Linux rfkill state: 1 = unblocked, 0 = soft-blocked. Let the Android Wi-Fi
# HAL toggle the SD8777 radio as it starts and stops the interface.
WIFI_DRIVER_STATE_CTRL_PARAM := /sys/class/rfkill/rfkill0/state
WIFI_DRIVER_STATE_ON := 1
WIFI_DRIVER_STATE_OFF := 0
BOARD_WPA_SUPPLICANT_DRIVER := NL80211
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_pxa988
WPA_SUPPLICANT_VERSION := VER_0_8_X
BOARD_HOSTAPD_DRIVER := NL80211
BOARD_HOSTAPD_PRIVATE_LIB := lib_driver_cmd_pxa988
WIFI_HIDL_UNIFIED_SUPPLICANT_SERVICE_RC_ENTRY := true

# Bluetooth (btmrvl_sdio HCI_CHANNEL_USER)
BOARD_HAVE_BLUETOOTH := true

# Audio (TinyALSA PXA-88PM805 with XML Audio Policy)
BOARD_USES_ALSA_AUDIO := true
USE_XML_AUDIO_POLICY_CONF := 1
