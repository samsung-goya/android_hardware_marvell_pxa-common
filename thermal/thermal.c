// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Thermal HAL for Marvell PXA988/PXA986 (Linux 4.19 DROTS + 88PM822 battery).
 */

#define LOG_TAG "PxaThermal"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <hardware/hardware.h>
#include <hardware/thermal.h>
#include <log/log.h>

#define NUM_CPUS 2
#define CPU_TEMP_PATH     "/sys/class/thermal/thermal_zone0/temp"
#define BATTERY_TEMP_PATH "/sys/class/thermal/thermal_zone1/temp"

static int read_millidegrees(const char *path, float *out_c)
{
    char buf[32];
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return -errno;

    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return -EIO;

    buf[n] = '\0';
    long mdeg = strtol(buf, NULL, 10);
    *out_c = (float)mdeg / 1000.0f;
    return 0;
}

static ssize_t pxa_get_temperatures(thermal_module_t *module,
                                    temperature_t *list, size_t size)
{
    (void)module;
    size_t count = 0;

    if (!list || size == 0)
        return 2;

    if (count < size) {
        float temp_c = 0.0f;
        if (read_millidegrees(CPU_TEMP_PATH, &temp_c) == 0) {
            list[count].type = DEVICE_TEMPERATURE_CPU;
            list[count].name = "cpu-thermal";
            list[count].current_value = temp_c;
            list[count].throttling_threshold = 85.0f;
            list[count].shutdown_threshold = 110.0f;
            list[count].vr_throttling_threshold = UNKNOWN_TEMPERATURE;
            count++;
        }
    }

    if (count < size) {
        float temp_c = 0.0f;
        if (read_millidegrees(BATTERY_TEMP_PATH, &temp_c) == 0) {
            list[count].type = DEVICE_TEMPERATURE_BATTERY;
            list[count].name = "battery";
            list[count].current_value = temp_c;
            list[count].throttling_threshold = 45.0f;
            list[count].shutdown_threshold = 60.0f;
            list[count].vr_throttling_threshold = UNKNOWN_TEMPERATURE;
            count++;
        }
    }

    return (ssize_t)count;
}

static ssize_t pxa_get_cpu_usages(thermal_module_t *module, cpu_usage_t *list)
{
    (void)module;
    if (!list)
        return NUM_CPUS;

    FILE *fp = fopen("/proc/stat", "re");
    if (!fp)
        return -errno;

    char line[256];
    int found = 0;
    while (fgets(line, sizeof(line), fp) && found < NUM_CPUS) {
        unsigned int cpu_id;
        unsigned long long user, nice, sys, idle, iowait, irq, softirq, steal;
        if (sscanf(line, "cpu%u %llu %llu %llu %llu %llu %llu %llu %llu",
                   &cpu_id, &user, &nice, &sys, &idle, &iowait,
                   &irq, &softirq, &steal) == 9 && cpu_id < NUM_CPUS) {
            char online_path[64];
            int online = 1;
            snprintf(online_path, sizeof(online_path),
                     "/sys/devices/system/cpu/cpu%u/online", cpu_id);
            FILE *ofp = fopen(online_path, "re");
            if (ofp) {
                if (fscanf(ofp, "%d", &online) != 1)
                    online = 1;
                fclose(ofp);
            }
            list[cpu_id].name = (cpu_id == 0) ? "cpu0" : "cpu1";
            list[cpu_id].active = user + nice + sys + irq + softirq + steal;
            list[cpu_id].total = list[cpu_id].active + idle + iowait;
            list[cpu_id].is_online = (online != 0);
            found++;
        }
    }
    fclose(fp);
    return NUM_CPUS;
}

static ssize_t pxa_get_cooling_devices(thermal_module_t *module,
                                       cooling_device_t *list, size_t size)
{
    (void)module;
    (void)list;
    (void)size;
    return 0;
}

static int pxa_close_thermal(struct hw_device_t *dev)
{
    free(dev);
    return 0;
}

static int pxa_open_thermal(const struct hw_module_t *module, const char *name,
                            struct hw_device_t **device)
{
    if (strcmp(name, THERMAL_HARDWARE_MODULE_ID) != 0)
        return -EINVAL;

    struct hw_device_t *dev = calloc(1, sizeof(*dev));
    if (!dev)
        return -ENOMEM;

    dev->tag = HARDWARE_DEVICE_TAG;
    dev->version = HARDWARE_DEVICE_API_VERSION(0, 1);
    dev->module = (struct hw_module_t *)module;
    dev->close = pxa_close_thermal;

    *device = dev;
    return 0;
}

static struct hw_module_methods_t pxa_thermal_methods = {
    .open = pxa_open_thermal,
};

thermal_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = THERMAL_HARDWARE_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = THERMAL_HARDWARE_MODULE_ID,
        .name = "Marvell PXA988 Thermal HAL",
        .author = "samsung-goya",
        .methods = &pxa_thermal_methods,
    },
    .getTemperatures = pxa_get_temperatures,
    .getCpuUsages = pxa_get_cpu_usages,
    .getCoolingDevices = pxa_get_cooling_devices,
};
