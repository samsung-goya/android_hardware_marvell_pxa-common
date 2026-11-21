// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Lights HAL for Marvell PXA988/PXA986 (Linux 4.19 DRM/KMS backlight).
 */

#define LOG_TAG "PxaLights"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <hardware/hardware.h>
#include <hardware/lights.h>
#include <log/log.h>

static const char *const kBacklightPaths[] = {
    "/sys/class/backlight/vx5b3d-backlight/brightness",
    "/sys/class/backlight/panel/brightness",
};

static const char *const kMaxBrightnessPaths[] = {
    "/sys/class/backlight/vx5b3d-backlight/max_brightness",
    "/sys/class/backlight/panel/max_brightness",
};

struct pxa_light_device {
    struct light_device_t device;
    const char *brightness_path;
    unsigned int max_brightness;
};

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

static unsigned int read_max_brightness(const char *path)
{
    char buf[16];
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return 255;

    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return 255;

    buf[n] = '\0';
    unsigned long val = strtoul(buf, NULL, 10);
    return (val > 0 && val <= 65535) ? (unsigned int)val : 255;
}

static const char *find_backlight_path(unsigned int *max_brightness)
{
    size_t i;
    for (i = 0; i < sizeof(kBacklightPaths) / sizeof(kBacklightPaths[0]); i++) {
        if (access(kBacklightPaths[i], W_OK) == 0) {
            *max_brightness = read_max_brightness(kMaxBrightnessPaths[i]);
            return kBacklightPaths[i];
        }
    }
    *max_brightness = 255;
    return kBacklightPaths[0];
}

static int pxa_set_light_backlight(struct light_device_t *dev,
                                   const struct light_state_t *state)
{
    struct pxa_light_device *pxa_dev = (struct pxa_light_device *)dev;
    if (!pxa_dev || !state)
        return -EINVAL;

    unsigned int r = (state->color >> 16) & 0xff;
    unsigned int g = (state->color >> 8) & 0xff;
    unsigned int b = state->color & 0xff;
    unsigned int lum = (77 * r + 150 * g + 29 * b) >> 8;
    unsigned int scaled = (lum * pxa_dev->max_brightness + 127) / 255;

    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%u\n", scaled);
    if (len <= 0 || (size_t)len >= sizeof(buf))
        return -EINVAL;

    pthread_mutex_lock(&g_lock);
    int fd = open(pxa_dev->brightness_path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        int err = -errno;
        ALOGE("open %s failed: %s", pxa_dev->brightness_path, strerror(errno));
        pthread_mutex_unlock(&g_lock);
        return err;
    }

    ssize_t written = write(fd, buf, (size_t)len);
    int err = (written == len) ? 0 : (written < 0 ? -errno : -EIO);
    close(fd);
    pthread_mutex_unlock(&g_lock);

    if (err != 0)
        ALOGE("write %u to %s failed: %s", scaled, pxa_dev->brightness_path, strerror(-err));

    return err;
}

static int pxa_set_light_noop(struct light_device_t *dev,
                              const struct light_state_t *state)
{
    (void)dev;
    (void)state;
    return 0;
}

static int pxa_close_lights(struct hw_device_t *dev)
{
    free(dev);
    return 0;
}

static int pxa_open_lights(const struct hw_module_t *module, const char *name,
                           struct hw_device_t **device)
{
    if (!name || !device)
        return -EINVAL;

    struct pxa_light_device *dev = calloc(1, sizeof(*dev));
    if (!dev)
        return -ENOMEM;

    dev->device.common.tag = HARDWARE_DEVICE_TAG;
    dev->device.common.version = LIGHTS_DEVICE_API_VERSION_1_0;
    dev->device.common.module = (struct hw_module_t *)module;
    dev->device.common.close = pxa_close_lights;

    if (strcmp(name, LIGHT_ID_BACKLIGHT) == 0) {
        dev->brightness_path = find_backlight_path(&dev->max_brightness);
        dev->device.set_light = pxa_set_light_backlight;
    } else if (strcmp(name, LIGHT_ID_BUTTONS) == 0 ||
               strcmp(name, LIGHT_ID_BATTERY) == 0 ||
               strcmp(name, LIGHT_ID_NOTIFICATIONS) == 0 ||
               strcmp(name, LIGHT_ID_ATTENTION) == 0) {
        dev->device.set_light = pxa_set_light_noop;
    } else {
        free(dev);
        return -EINVAL;
    }

    *device = &dev->device.common;
    return 0;
}

static struct hw_module_methods_t pxa_lights_methods = {
    .open = pxa_open_lights,
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .module_api_version = HARDWARE_MODULE_API_VERSION(1, 0),
    .hal_api_version = HARDWARE_HAL_API_VERSION,
    .id = LIGHTS_HARDWARE_MODULE_ID,
    .name = "Marvell PXA988 Lights HAL",
    .author = "samsung-goya",
    .methods = &pxa_lights_methods,
};
