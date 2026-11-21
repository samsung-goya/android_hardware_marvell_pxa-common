/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * IIO Sensors HAL for Marvell PXA988 / PXA986 (Bosch BMA255 accelerometer)
 */

#define LOG_TAG "sensors.pxa988"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <log/log.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define IIO_SYSFS_DIR   "/sys/bus/iio/devices"
#define TARGET_IIO_NAME "bma255"
#define ACCEL_HANDLE    0
#define MIN_DELAY_US    10000    /* 100 Hz */
#define MAX_DELAY_US    1000000  /* 1 Hz */
#define DEFAULT_DELAY_NS (200000000LL) /* 200 ms */

struct pxa_sensors_context {
    struct sensors_poll_device_1 device;

    pthread_mutex_t lock;
    pthread_cond_t cond;

    bool enabled;
    int flush_pending;
    int64_t sampling_period_ns;

    char dev_path[256];
    int fd_x;
    int fd_y;
    int fd_z;
    float scale;
    float mount_matrix[9];
};

static struct sensor_t s_sensor_list[] = {
    {
        .name = "BMA255 3-axis Accelerometer",
        .vendor = "Bosch Sensortec",
        .version = 1,
        .handle = ACCEL_HANDLE,
        .type = SENSOR_TYPE_ACCELEROMETER,
        .maxRange = 19.6133f,
        .resolution = 0.00958f,
        .power = 0.13f,
        .minDelay = MIN_DELAY_US,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_ACCELEROMETER,
        .requiredPermission = "",
        .maxDelay = MAX_DELAY_US,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE,
        .reserved = {0},
    },
};

static int64_t get_boottime_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int read_sysfs_str(const char *path, char *buf, size_t len) {
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return -errno;

    ssize_t n = read(fd, buf, len - 1);
    close(fd);
    if (n < 0)
        return -errno;

    buf[n] = '\0';
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' || buf[n - 1] == ' '))
        buf[--n] = '\0';
    return 0;
}

static int read_fd_int(int fd, int *out) {
    char buf[32];
    if (fd < 0)
        return -EBADF;
    if (lseek(fd, 0, SEEK_SET) < 0)
        return -errno;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (n <= 0)
        return -EIO;
    buf[n] = '\0';
    *out = atoi(buf);
    return 0;
}

static int find_bma255_device(struct pxa_sensors_context *ctx) {
    DIR *dir = opendir(IIO_SYSFS_DIR);
    if (!dir) {
        ALOGE("Failed to open %s: %s", IIO_SYSFS_DIR, strerror(errno));
        return -errno;
    }

    struct dirent *ent;
    char path[320];
    char name[64];
    int found = 0;

    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, "iio:device", 10) != 0)
            continue;

        snprintf(path, sizeof(path), "%s/%s/name", IIO_SYSFS_DIR, ent->d_name);
        if (read_sysfs_str(path, name, sizeof(name)) == 0 &&
            strcmp(name, TARGET_IIO_NAME) == 0) {
            snprintf(ctx->dev_path, sizeof(ctx->dev_path), "%s/%s",
                     IIO_SYSFS_DIR, ent->d_name);
            found = 1;
            break;
        }
    }
    closedir(dir);

    if (!found) {
        ALOGE("IIO device '%s' not found under %s", TARGET_IIO_NAME, IIO_SYSFS_DIR);
        return -ENODEV;
    }

    /* Read scale */
    char buf[128];
    snprintf(path, sizeof(path), "%s/in_accel_scale", ctx->dev_path);
    if (read_sysfs_str(path, buf, sizeof(buf)) == 0) {
        ctx->scale = strtof(buf, NULL);
    } else {
        ctx->scale = 0.0095768f;
        ALOGW("Using default scale %f", ctx->scale);
    }
    s_sensor_list[0].resolution = ctx->scale;

    /* Read mount-matrix (default to identity) */
    ctx->mount_matrix[0] = 1.0f; ctx->mount_matrix[1] = 0.0f; ctx->mount_matrix[2] = 0.0f;
    ctx->mount_matrix[3] = 0.0f; ctx->mount_matrix[4] = 1.0f; ctx->mount_matrix[5] = 0.0f;
    ctx->mount_matrix[6] = 0.0f; ctx->mount_matrix[7] = 0.0f; ctx->mount_matrix[8] = 1.0f;

    snprintf(path, sizeof(path), "%s/in_accel_mount_matrix", ctx->dev_path);
    if (read_sysfs_str(path, buf, sizeof(buf)) == 0) {
        float m[9];
        if (sscanf(buf, "%f , %f , %f ; %f , %f , %f ; %f , %f , %f",
                   &m[0], &m[1], &m[2],
                   &m[3], &m[4], &m[5],
                   &m[6], &m[7], &m[8]) == 9) {
            memcpy(ctx->mount_matrix, m, sizeof(m));
        } else {
            ALOGW("Failed to parse mount_matrix: '%s'", buf);
        }
    }

    snprintf(path, sizeof(path), "%s/in_accel_x_raw", ctx->dev_path);
    ctx->fd_x = open(path, O_RDONLY);
    snprintf(path, sizeof(path), "%s/in_accel_y_raw", ctx->dev_path);
    ctx->fd_y = open(path, O_RDONLY);
    snprintf(path, sizeof(path), "%s/in_accel_z_raw", ctx->dev_path);
    ctx->fd_z = open(path, O_RDONLY);

    if (ctx->fd_x < 0 || ctx->fd_y < 0 || ctx->fd_z < 0) {
        ALOGE("Failed to open raw accel sysfs files under %s", ctx->dev_path);
        return -EACCES;
    }

    ALOGI("Initialized BMA255 at %s (scale=%f)", ctx->dev_path, ctx->scale);
    return 0;
}

static int pxa_activate(struct sensors_poll_device_t *dev, int handle, int enabled) {
    struct pxa_sensors_context *ctx = (struct pxa_sensors_context *)dev;
    if (handle != ACCEL_HANDLE)
        return -EINVAL;

    pthread_mutex_lock(&ctx->lock);
    ctx->enabled = (enabled != 0);
    pthread_cond_broadcast(&ctx->cond);
    pthread_mutex_unlock(&ctx->lock);
    return 0;
}

static int pxa_set_delay(struct sensors_poll_device_t *dev, int handle, int64_t ns) {
    struct pxa_sensors_context *ctx = (struct pxa_sensors_context *)dev;
    if (handle != ACCEL_HANDLE)
        return -EINVAL;

    int64_t min_ns = (int64_t)MIN_DELAY_US * 1000LL;
    int64_t max_ns = (int64_t)MAX_DELAY_US * 1000LL;
    if (ns < min_ns)
        ns = min_ns;
    else if (ns > max_ns)
        ns = max_ns;

    pthread_mutex_lock(&ctx->lock);
    ctx->sampling_period_ns = ns;
    pthread_cond_broadcast(&ctx->cond);
    pthread_mutex_unlock(&ctx->lock);
    return 0;
}

static int pxa_batch(struct sensors_poll_device_1 *dev, int handle, int flags,
                     int64_t sampling_period_ns, int64_t max_report_latency_ns) {
    (void)flags;
    (void)max_report_latency_ns;
    return pxa_set_delay(&dev->v0, handle, sampling_period_ns);
}

static int pxa_flush(struct sensors_poll_device_1 *dev, int handle) {
    struct pxa_sensors_context *ctx = (struct pxa_sensors_context *)dev;
    if (handle != ACCEL_HANDLE)
        return -EINVAL;

    pthread_mutex_lock(&ctx->lock);
    if (!ctx->enabled) {
        pthread_mutex_unlock(&ctx->lock);
        return -EINVAL;
    }
    ctx->flush_pending++;
    pthread_cond_broadcast(&ctx->cond);
    pthread_mutex_unlock(&ctx->lock);
    return 0;
}

static int pxa_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count) {
    struct pxa_sensors_context *ctx = (struct pxa_sensors_context *)dev;
    if (count <= 0)
        return -EINVAL;

    pthread_mutex_lock(&ctx->lock);
    while (!ctx->enabled && ctx->flush_pending == 0) {
        pthread_cond_wait(&ctx->cond, &ctx->lock);
    }

    if (ctx->flush_pending > 0) {
        ctx->flush_pending--;
        pthread_mutex_unlock(&ctx->lock);

        memset(data, 0, sizeof(*data));
        data->version = META_DATA_VERSION;
        data->type = SENSOR_TYPE_META_DATA;
        data->sensor = 0;
        data->meta_data.what = META_DATA_FLUSH_COMPLETE;
        data->meta_data.sensor = ACCEL_HANDLE;
        data->timestamp = get_boottime_ns();
        return 1;
    }

    int64_t period_ns = ctx->sampling_period_ns;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t target_ns = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec + period_ns;
    ts.tv_sec = target_ns / 1000000000LL;
    ts.tv_nsec = target_ns % 1000000000LL;

    pthread_cond_timedwait(&ctx->cond, &ctx->lock, &ts);

    bool still_enabled = ctx->enabled;
    int flush_now = ctx->flush_pending;
    pthread_mutex_unlock(&ctx->lock);

    if (flush_now > 0)
        return pxa_poll(dev, data, count);

    if (!still_enabled)
        return 0;

    int rx = 0, ry = 0, rz = 0;
    if (read_fd_int(ctx->fd_x, &rx) < 0 ||
        read_fd_int(ctx->fd_y, &ry) < 0 ||
        read_fd_int(ctx->fd_z, &rz) < 0) {
        return -EIO;
    }

    float ax = rx * ctx->scale;
    float ay = ry * ctx->scale;
    float az = rz * ctx->scale;
    const float *m = ctx->mount_matrix;

    memset(data, 0, sizeof(*data));
    data->version = sizeof(sensors_event_t);
    data->sensor = ACCEL_HANDLE;
    data->type = SENSOR_TYPE_ACCELEROMETER;
    data->timestamp = get_boottime_ns();
    /* Convert from the DTS landscape IIO mount-matrix (power/vol UP = -Y,
     * Samsung logo/portrait top = +X) into Android's natural portrait sensor
     * frame (+X = right/power-vol, +Y = up/Samsung logo, +Z = out of screen). */
    float mx = m[0] * ax + m[1] * ay + m[2] * az;
    float my = m[3] * ax + m[4] * ay + m[5] * az;
    float mz = m[6] * ax + m[7] * ay + m[8] * az;
    data->acceleration.x = -my;
    data->acceleration.y = mx;
    data->acceleration.z = mz;
    data->acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

    return 1;
}

static int pxa_close(struct hw_device_t *dev) {
    struct pxa_sensors_context *ctx = (struct pxa_sensors_context *)dev;
    if (ctx) {
        if (ctx->fd_x >= 0) close(ctx->fd_x);
        if (ctx->fd_y >= 0) close(ctx->fd_y);
        if (ctx->fd_z >= 0) close(ctx->fd_z);
        pthread_cond_destroy(&ctx->cond);
        pthread_mutex_destroy(&ctx->lock);
        free(ctx);
    }
    return 0;
}

static int pxa_get_sensors_list(struct sensors_module_t *module,
                                struct sensor_t const **list) {
    (void)module;
    *list = s_sensor_list;
    return sizeof(s_sensor_list) / sizeof(s_sensor_list[0]);
}

static int pxa_open_sensors(const struct hw_module_t *module, const char *id,
                            struct hw_device_t **device) {
    (void)id;
    struct pxa_sensors_context *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
        return -ENOMEM;

    ctx->fd_x = -1;
    ctx->fd_y = -1;
    ctx->fd_z = -1;
    ctx->sampling_period_ns = DEFAULT_DELAY_NS;

    pthread_mutex_init(&ctx->lock, NULL);
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&ctx->cond, &attr);
    pthread_condattr_destroy(&attr);

    int ret = find_bma255_device(ctx);
    if (ret < 0) {
        pxa_close(&ctx->device.common);
        return ret;
    }

    ctx->device.common.tag = HARDWARE_DEVICE_TAG;
    ctx->device.common.version = SENSORS_DEVICE_API_VERSION_1_3;
    ctx->device.common.module = (struct hw_module_t *)module;
    ctx->device.common.close = pxa_close;
    ctx->device.activate = pxa_activate;
    ctx->device.setDelay = pxa_set_delay;
    ctx->device.poll = pxa_poll;
    ctx->device.batch = pxa_batch;
    ctx->device.flush = pxa_flush;

    *device = &ctx->device.common;
    return 0;
}

static struct hw_module_methods_t s_sensors_module_methods = {
    .open = pxa_open_sensors,
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = SENSORS_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "PXA988 IIO Sensors HAL",
        .author = "LineageOS goyawifi",
        .methods = &s_sensors_module_methods,
    },
    .get_sensors_list = pxa_get_sensors_list,
};
