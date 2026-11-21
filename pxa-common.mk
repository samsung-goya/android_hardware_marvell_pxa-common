# SPDX-License-Identifier: Apache-2.0
# Common HAL packages for Marvell PXA988/PXA986 platform (Linux 4.19)

# Lights HAL (vx5b3d-backlight)
PRODUCT_PACKAGES += \
    lights.pxa988 \
    android.hardware.light@2.0-impl \
    android.hardware.light@2.0-service

# Power HAL (cpufreq-dt schedutil)
PRODUCT_PACKAGES += \
    power.pxa988 \
    android.hardware.power@1.0-impl \
    android.hardware.power@1.0-service

# Thermal HAL (DROTS cpu-thermal + battery)
PRODUCT_PACKAGES += \
    thermal.pxa988 \
    android.hardware.thermal@1.0-impl \
    android.hardware.thermal@1.0-service

# Memtrack HAL
PRODUCT_PACKAGES += \
    memtrack.pxa988 \
    android.hardware.memtrack@1.0-impl \
    android.hardware.memtrack@1.0-service

# Graphics & Display (gralloc.pxa988 + hwcomposer.pxa988 + Mesa3D)
PRODUCT_PACKAGES += \
    gralloc.pxa988 \
    hwcomposer.pxa988 \
    android.hardware.graphics.allocator@2.0-impl \
    android.hardware.graphics.allocator@2.0-service \
    android.hardware.graphics.mapper@2.0-impl \
    android.hardware.graphics.composer@2.1-impl \
    android.hardware.graphics.composer@2.1-service \
    libGLES_mesa \
    gallium_dri \
    libdrm_etnaviv

PRODUCT_PROPERTY_OVERRIDES += \
    ro.hardware.gralloc=pxa988 \
    ro.hardware.hwcomposer=pxa988 \
    ro.hardware.egl=mesa \
    ro.opengles.version=131072 \
    debug.hwui.renderer=opengl \
    debug.sf.latch_unsignaled=1

# Audio HAL (TinyALSA PXA-88PM805)
PRODUCT_PACKAGES += \
    audio.primary.pxa988 \
    audio.a2dp.default \
    audio.usb.default \
    audio.r_submix.default \
    android.hardware.audio@4.0-impl \
    android.hardware.audio@2.0-service \
    android.hardware.audio.effect@4.0-impl

# Wi-Fi (mwifiex_sdio nl80211) & Bluetooth (btmrvl_sdio HCI_CHANNEL_USER)
PRODUCT_PACKAGES += \
    android.hardware.wifi@1.0-service \
    wificond \
    wpa_supplicant \
    hostapd \
    iw \
    android.hardware.bluetooth@1.0-service.pxa988

# Sensors HAL (IIO BMA255)
PRODUCT_PACKAGES += \
    sensors.pxa988 \
    android.hardware.sensors@1.0-impl \
    android.hardware.sensors@1.0-service

# Hardware VPU Codec Plugin (Chips&Media CODA7542 V4L2 M2M via coda.ko)
PRODUCT_PACKAGES += \
    libstagefrighthw

# Camera HAL (V4L2 SR200PC20M rear camera via marvell-ccic)
PRODUCT_PACKAGES += \
    camera.pxa988 \
    android.hardware.camera.provider@2.4-impl \
    android.hardware.camera.provider@2.4-service
