// SPDX-License-Identifier: Apache-2.0
/*
 * Power HAL for Marvell PXA988/PXA986 (Linux 4.19 cpufreq-dt + schedutil).
 */

#define LOG_TAG "PxaPower"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <hardware/hardware.h>
#include <hardware/power.h>
#include <log/log.h>

#define CPUFREQ_POLICY0_MIN "/sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq"
#define CPUFREQ_POLICY0_MAX "/sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq"
#define GPU_DEVFREQ_MIN     "/sys/class/devfreq/c0400000.gpu/min_freq"

/* PXA988 OPPs in kHz: 312000, 624000, 1066000 (1175 mV), 1204666 (1275 mV) */
#define FREQ_MIN_IDLE       "312000"
#define FREQ_BOOST_MIN      "1066000"
#define FREQ_LOW_POWER_MAX  "1066000"
#define FREQ_NORMAL_MAX     "1204666"

/* Vivante GC1000 OPPs in Hz: 156M, 312M, 416M (all 1175 mV) */
#define GPU_FREQ_MIN_IDLE   "156000000"
#define GPU_FREQ_BOOST_MIN  "416000000"

#define INTERACTION_DEFAULT_MS 200
#define INTERACTION_MIN_MS     80
#define INTERACTION_MAX_MS     1000

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cond;
static bool g_low_power = false;
static bool g_interactive = true;
static bool g_launch_active = false;
static bool g_boosted = false;
static bool g_worker_started = false;
static int64_t g_boost_until_ns = 0;

static void sysfs_write(const char *path, const char *val)
{
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0)
        return;
    ssize_t len = (ssize_t)strlen(val);
    if (write(fd, val, (size_t)len) != len)
        ALOGV("sysfs_write(%s, %s) failed: %s", path, val, strerror(errno));
    close(fd);
}

static int64_t get_monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

static void apply_boost_locked(bool boost)
{
    if (boost == g_boosted)
        return;

    sysfs_write(CPUFREQ_POLICY0_MIN, boost ? FREQ_BOOST_MIN : FREQ_MIN_IDLE);
    sysfs_write(GPU_DEVFREQ_MIN, boost ? GPU_FREQ_BOOST_MIN : GPU_FREQ_MIN_IDLE);
    g_boosted = boost;
}

static void *boost_worker_main(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_lock);
    for (;;) {
        while (g_boost_until_ns == 0)
            pthread_cond_wait(&g_cond, &g_lock);

        int64_t now_ns = get_monotonic_ns();
        if (now_ns < g_boost_until_ns) {
            struct timespec ts;
            ts.tv_sec = (time_t)(g_boost_until_ns / 1000000000LL);
            ts.tv_nsec = (long)(g_boost_until_ns % 1000000000LL);
            pthread_cond_timedwait(&g_cond, &g_lock, &ts);
            continue;
        }

        g_boost_until_ns = 0;
        if (!g_launch_active)
            apply_boost_locked(false);
    }
    pthread_mutex_unlock(&g_lock);
    return NULL;
}

static void ensure_worker_locked(void)
{
    if (g_worker_started)
        return;

    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&g_cond, &attr);
    pthread_condattr_destroy(&attr);

    pthread_attr_t tattr;
    pthread_attr_init(&tattr);
    pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED);
    pthread_t tid;
    int ret = pthread_create(&tid, &tattr, boost_worker_main, NULL);
    pthread_attr_destroy(&tattr);

    if (ret == 0) {
        g_worker_started = true;
    } else {
        pthread_cond_destroy(&g_cond);
        ALOGW("Failed to create power boost worker: %s", strerror(ret));
    }
}

static void pxa_power_init(struct power_module *module)
{
    (void)module;
    pthread_mutex_lock(&g_lock);
    ensure_worker_locked();
    pthread_mutex_unlock(&g_lock);
    ALOGI("Initialized Marvell PXA988 Power HAL");
}

static void pxa_power_set_interactive(struct power_module *module, int on)
{
    (void)module;
    pthread_mutex_lock(&g_lock);
    g_interactive = (on != 0);
    if (!g_interactive) {
        g_launch_active = false;
        g_boost_until_ns = 0;
        apply_boost_locked(false);
    } else if (!g_low_power) {
        sysfs_write(CPUFREQ_POLICY0_MAX, FREQ_NORMAL_MAX);
    }
    pthread_mutex_unlock(&g_lock);
}

static void pxa_power_hint(struct power_module *module, power_hint_t hint, void *data)
{
    (void)module;
    pthread_mutex_lock(&g_lock);

    switch (hint) {
    case POWER_HINT_LOW_POWER:
        g_low_power = (data != NULL);
        sysfs_write(CPUFREQ_POLICY0_MAX, g_low_power ? FREQ_LOW_POWER_MAX : FREQ_NORMAL_MAX);
        if (g_low_power) {
            g_boost_until_ns = 0;
            apply_boost_locked(false);
        }
        break;
    case POWER_HINT_INTERACTION: {
        int duration_ms = (data != NULL) ? *(const int *)data : 0;
        if (duration_ms <= 0)
            duration_ms = INTERACTION_DEFAULT_MS;
        else if (duration_ms < INTERACTION_MIN_MS)
            duration_ms = INTERACTION_MIN_MS;
        else if (duration_ms > INTERACTION_MAX_MS)
            duration_ms = INTERACTION_MAX_MS;

        if (g_interactive && !g_low_power) {
            ensure_worker_locked();
            if (g_worker_started) {
                int64_t deadline = get_monotonic_ns() +
                                   (int64_t)duration_ms * 1000000LL;
                if (deadline > g_boost_until_ns)
                    g_boost_until_ns = deadline;
                apply_boost_locked(true);
                pthread_cond_signal(&g_cond);
            }
        }
        break;
    }
    case POWER_HINT_LAUNCH:
        g_launch_active = (data != NULL);
        if (g_launch_active) {
            if (g_interactive && !g_low_power)
                apply_boost_locked(true);
        } else if (g_boost_until_ns == 0 || get_monotonic_ns() >= g_boost_until_ns) {
            g_boost_until_ns = 0;
            apply_boost_locked(false);
        }
        break;
    default:
        break;
    }

    pthread_mutex_unlock(&g_lock);
}

static void pxa_set_feature(struct power_module *module, feature_t feature, int state)
{
    (void)module;
    (void)feature;
    (void)state;
}

struct power_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = POWER_MODULE_API_VERSION_0_2,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = POWER_HARDWARE_MODULE_ID,
        .name = "Marvell PXA988 Power HAL",
        .author = "LineageOS goyawifi",
        .methods = NULL,
    },
    .init = pxa_power_init,
    .setInteractive = pxa_power_set_interactive,
    .powerHint = pxa_power_hint,
    .setFeature = pxa_set_feature,
};
