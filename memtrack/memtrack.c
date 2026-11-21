// SPDX-License-Identifier: Apache-2.0
/*
 * Minimal Memtrack HAL for Marvell PXA988/PXA986 (Linux 4.19).
 */

#include <errno.h>
#include <stddef.h>

#include <hardware/hardware.h>
#include <hardware/memtrack.h>

static int pxa_memtrack_init(const struct memtrack_module *module)
{
    (void)module;
    return 0;
}

static int pxa_memtrack_get_memory(const struct memtrack_module *module,
                                   pid_t pid,
                                   int type,
                                   struct memtrack_record *records,
                                   size_t *num_records)
{
    (void)module;
    (void)pid;
    (void)type;
    (void)records;

    if (!num_records)
        return -EINVAL;

    *num_records = 0;
    return 0;
}

static struct hw_module_methods_t pxa_memtrack_methods = {
    .open = NULL,
};

struct memtrack_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = MEMTRACK_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = MEMTRACK_HARDWARE_MODULE_ID,
        .name = "Marvell PXA988 Memtrack HAL",
        .author = "LineageOS goyawifi",
        .methods = &pxa_memtrack_methods,
    },
    .init = pxa_memtrack_init,
    .getMemory = pxa_memtrack_get_memory,
};
