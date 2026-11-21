/*
 * Copyright 2026 The Chromium OS / samsung-goya Authors.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifdef DRV_ETNAVIV

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "drv_priv.h"
#include "helpers.h"
#include "util.h"

/* Standard etnaviv DRM GEM UAPI (linux/include/uapi/drm/etnaviv_drm.h) */
#ifndef ETNA_BO_CACHED
#define ETNA_BO_CACHED 0x00010000
#endif
#ifndef ETNA_BO_WC
#define ETNA_BO_WC 0x00020000
#endif
#ifndef ETNA_PREP_READ
#define ETNA_PREP_READ 0x01
#endif
#ifndef ETNA_PREP_WRITE
#define ETNA_PREP_WRITE 0x02
#endif

#ifndef DRM_ETNAVIV_GEM_NEW
struct drm_etnaviv_timespec {
	int64_t tv_sec;
	int64_t tv_nsec;
};

struct drm_etnaviv_gem_new {
	uint64_t size;
	uint32_t flags;
	uint32_t handle;
};

struct drm_etnaviv_gem_cpu_prep {
	uint32_t handle;
	uint32_t op;
	struct drm_etnaviv_timespec timeout;
};

struct drm_etnaviv_gem_cpu_fini {
	uint32_t handle;
	uint32_t flags;
};

#define DRM_ETNAVIV_GEM_NEW 0x02
#define DRM_ETNAVIV_GEM_CPU_PREP 0x04
#define DRM_ETNAVIV_GEM_CPU_FINI 0x05

#define DRM_IOCTL_ETNAVIV_GEM_NEW \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_ETNAVIV_GEM_NEW, struct drm_etnaviv_gem_new)
#define DRM_IOCTL_ETNAVIV_GEM_CPU_PREP \
	DRM_IOW(DRM_COMMAND_BASE + DRM_ETNAVIV_GEM_CPU_PREP, struct drm_etnaviv_gem_cpu_prep)
#define DRM_IOCTL_ETNAVIV_GEM_CPU_FINI \
	DRM_IOW(DRM_COMMAND_BASE + DRM_ETNAVIV_GEM_CPU_FINI, struct drm_etnaviv_gem_cpu_fini)
#endif

#ifndef DRM_RDWR
#define DRM_RDWR O_RDWR
#endif

struct etnaviv_priv {
	int kms_fd;
};

static const uint32_t render_target_formats[] = {
	DRM_FORMAT_ABGR8888, DRM_FORMAT_ARGB8888, DRM_FORMAT_RGB565,
	DRM_FORMAT_XBGR8888, DRM_FORMAT_XRGB8888,
};

static const uint32_t sw_only_formats[] = {
	DRM_FORMAT_BGR888,
};

static const uint32_t texture_source_formats[] = {
	DRM_FORMAT_R8, DRM_FORMAT_NV12, DRM_FORMAT_NV21, DRM_FORMAT_YVU420,
	DRM_FORMAT_YVU420_ANDROID,
};

static int open_kms_drm_device(void)
{
	for (int i = 0; i < 4; i++) {
		char path[32];
		snprintf(path, sizeof(path), "/dev/dri/card%d", i);
		int fd = open(path, O_RDWR | O_CLOEXEC);
		if (fd < 0)
			continue;

		uint64_t has_dumb = 0;
		if (drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_dumb) == 0 && has_dumb) {
			drmModeRes *res = drmModeGetResources(fd);
			if (res) {
				bool has_kms = res->count_crtcs > 0 && res->count_connectors > 0;
				drmModeFreeResources(res);
				if (has_kms)
					return fd;
			}
		}
		close(fd);
	}

	return -ENODEV;
}

static int get_kms_fd(struct driver *drv)
{
	struct etnaviv_priv *priv = (struct etnaviv_priv *)drv->priv;
	if (!priv)
		return -ENODEV;

	if (priv->kms_fd < 0)
		priv->kms_fd = open_kms_drm_device();

	return priv->kms_fd;
}

static int etnaviv_init(struct driver *drv)
{
	struct etnaviv_priv *priv = calloc(1, sizeof(*priv));
	if (!priv)
		return -ENOMEM;

	priv->kms_fd = -1;
	drv->priv = priv;

	drv_add_combinations(drv, render_target_formats, ARRAY_SIZE(render_target_formats),
			     &LINEAR_METADATA, BO_USE_RENDER_MASK);

	drv_add_combinations(drv, sw_only_formats, ARRAY_SIZE(sw_only_formats),
			     &LINEAR_METADATA, BO_USE_LINEAR | BO_USE_SW);

	drv_add_combinations(drv, texture_source_formats, ARRAY_SIZE(texture_source_formats),
			     &LINEAR_METADATA,
			     BO_USE_TEXTURE_MASK | BO_USE_CAMERA_READ | BO_USE_CAMERA_WRITE |
				 BO_USE_HW_VIDEO_DECODER | BO_USE_HW_VIDEO_ENCODER);

	/*
	 * mmp-drm supports linear RGB formats on both GRA primary and DMA overlay
	 * planes, and planar YUV420/YVU420 on the DMA overlay plane.
	 */
	drv_modify_combination(drv, DRM_FORMAT_XRGB8888, &LINEAR_METADATA,
			       BO_USE_CURSOR | BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_ARGB8888, &LINEAR_METADATA,
			       BO_USE_CURSOR | BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_ABGR8888, &LINEAR_METADATA, BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_XBGR8888, &LINEAR_METADATA, BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_RGB565, &LINEAR_METADATA, BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_BGR888, &LINEAR_METADATA, BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_YVU420, &LINEAR_METADATA, BO_USE_SCANOUT);
	drv_modify_combination(drv, DRM_FORMAT_YVU420_ANDROID, &LINEAR_METADATA, BO_USE_SCANOUT);

	return 0;
}

static void etnaviv_close(struct driver *drv)
{
	struct etnaviv_priv *priv = (struct etnaviv_priv *)drv->priv;
	if (!priv)
		return;

	if (priv->kms_fd >= 0)
		close(priv->kms_fd);

	free(priv);
	drv->priv = NULL;
}

static int etnaviv_alloc_cma_scanout(struct driver *drv, uint32_t alloc_width,
				     uint32_t alloc_height, uint32_t bpp, uint32_t *out_pitch,
				     size_t *out_size, uint32_t *out_etna_handle)
{
	int kms_fd = get_kms_fd(drv);
	if (kms_fd < 0)
		return kms_fd;

	struct drm_mode_create_dumb create_dumb = { 0 };
	create_dumb.width = alloc_width;
	create_dumb.height = alloc_height;
	create_dumb.bpp = bpp;

	if (drmIoctl(kms_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_dumb) < 0)
		return -errno;

	struct drm_prime_handle prime_arg = { 0 };
	prime_arg.handle = create_dumb.handle;
	prime_arg.flags = DRM_CLOEXEC | DRM_RDWR;
	int ret = drmIoctl(kms_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime_arg);
	if (ret < 0) {
		prime_arg.flags = DRM_CLOEXEC;
		ret = drmIoctl(kms_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime_arg);
	}
	if (ret < 0) {
		int err = -errno;
		struct drm_mode_destroy_dumb destroy_dumb = { .handle = create_dumb.handle };
		drmIoctl(kms_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy_dumb);
		return err;
	}

	struct drm_prime_handle import_arg = { 0 };
	import_arg.fd = prime_arg.fd;
	ret = drmIoctl(drv->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &import_arg);
	int import_err = ret < 0 ? -errno : 0;

	close(prime_arg.fd);

	/* Drop the KMS dumb handle; etnaviv's dma_buf attachment retains the CMA BO. */
	struct drm_mode_destroy_dumb destroy_dumb = { .handle = create_dumb.handle };
	drmIoctl(kms_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy_dumb);

	if (ret < 0)
		return import_err;

	*out_pitch = create_dumb.pitch;
	*out_size = (size_t)create_dumb.size;
	*out_etna_handle = import_arg.handle;
	return 0;
}

static int etnaviv_bo_create(struct bo *bo, uint32_t width, uint32_t height, uint32_t format,
			     uint64_t use_flags)
{
	uint32_t alloc_width, alloc_height, layout_height, stride, bpp;
	bool is_planar_yv12 =
	    (format == DRM_FORMAT_YVU420_ANDROID || format == DRM_FORMAT_YVU420);
	bool is_biplanar_yuv = (format == DRM_FORMAT_NV12 || format == DRM_FORMAT_NV21);

	if (format == DRM_FORMAT_R8) {
		alloc_width = width;
		alloc_height = height;
		layout_height = height;
		stride = drv_stride_from_format(format, width, 0);
		bpp = 8;
	} else if (is_planar_yv12) {
		alloc_width = ALIGN(width, 32);
		alloc_height = 3 * DIV_ROUND_UP(height, 2);
		layout_height = height;
		stride = drv_stride_from_format(format, alloc_width, 0);
		bpp = 8;
	} else if (is_biplanar_yuv) {
		uint32_t aligned_h = ALIGN(height, 4);
		alloc_width = ALIGN(width, 32);
		alloc_height = aligned_h * 3 / 2;
		layout_height = aligned_h;
		stride = drv_stride_from_format(format, alloc_width, 0);
		bpp = 8;
	} else {
		/*
		 * Vivante GC1000 RS engine requires 16-pixel width alignment and
		 * 4-pixel height alignment for linear render/texture targets; 32-pixel
		 * width alignment also aligns 16bpp/32bpp strides to >= 64 bytes.
		 */
		alloc_width = ALIGN(width, 32);
		alloc_height = ALIGN(height, 4);
		layout_height = alloc_height;
		stride = drv_stride_from_format(format, alloc_width, 0);
		bpp = drv_bytes_per_pixel_from_format(format, 0) * 8;
	}

	drv_bo_from_format(bo, stride, layout_height, format);

	uint32_t handle = 0;
	bool allocated = false;

	/*
	 * Reserve CMA for RGB scanout buffers that fit the 1024x600 panel.
	 * YUV video buffers and oversized layers allocate from normal paged RAM
	 * via etnaviv's IOMMU (DRM_IOCTL_ETNAVIV_GEM_NEW) so the 96 MiB CMA pool
	 * stays available for CODA7542 VPU DPB/capture buffers (which lack an IOMMU).
	 */
	bool allow_cma_scanout = (use_flags & (BO_USE_SCANOUT | BO_USE_CURSOR)) &&
				 !is_planar_yv12 && !is_biplanar_yuv &&
				 width <= 1024 && height <= 600;
	if (allow_cma_scanout) {
		uint32_t cma_pitch = 0;
		size_t cma_size = 0;
		if (etnaviv_alloc_cma_scanout(bo->drv, alloc_width, alloc_height, bpp, &cma_pitch,
					      &cma_size, &handle) == 0) {
			if (cma_pitch != stride)
				drv_bo_from_format(bo, cma_pitch, layout_height, format);
			bo->total_size = cma_size;
			allocated = true;
		} else {
			drv_log("CMA scanout alloc failed (%ux%u fmt=0x%x), falling back to etnaviv GEM\n",
				width, height, format);
		}
	}

	if (!allocated) {
		struct drm_etnaviv_gem_new gem_new = { 0 };
		gem_new.size = ALIGN(bo->total_size, 4096);
		gem_new.flags = (use_flags & BO_USE_SW_READ_OFTEN) ? ETNA_BO_CACHED : ETNA_BO_WC;

		int ret = drmIoctl(bo->drv->fd, DRM_IOCTL_ETNAVIV_GEM_NEW, &gem_new);
		if (ret) {
			drv_log("DRM_IOCTL_ETNAVIV_GEM_NEW failed (size=%llu, err=%d)\n",
				(unsigned long long)gem_new.size, errno);
			return -errno;
		}
		handle = gem_new.handle;
		bo->total_size = (size_t)gem_new.size;
	}

	for (size_t plane = 0; plane < bo->num_planes; plane++)
		bo->handles[plane].u32 = handle;

	return 0;
}

static void *etnaviv_bo_map(struct bo *bo, struct vma *vma, size_t plane, uint32_t map_flags)
{
	int prime_fd = drv_bo_get_plane_fd(bo, plane);
	if (prime_fd < 0) {
		drv_log("drv_bo_get_plane_fd failed (%d)\n", prime_fd);
		return MAP_FAILED;
	}

	vma->length = bo->total_size;
	void *addr = mmap(0, vma->length, drv_get_prot(map_flags), MAP_SHARED, prime_fd, 0);
	close(prime_fd);
	return addr;
}

static int etnaviv_bo_invalidate(struct bo *bo, struct mapping *mapping)
{
	struct drm_etnaviv_gem_cpu_prep prep = { 0 };
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	prep.handle = bo->handles[0].u32;
	prep.op = ETNA_PREP_READ;
	if (mapping->vma->map_flags & BO_MAP_WRITE)
		prep.op |= ETNA_PREP_WRITE;
	prep.timeout.tv_sec = now.tv_sec + 5;
	prep.timeout.tv_nsec = now.tv_nsec;

	if (drmIoctl(bo->drv->fd, DRM_IOCTL_ETNAVIV_GEM_CPU_PREP, &prep) < 0)
		return -errno;

	return 0;
}

static int etnaviv_bo_flush(struct bo *bo, struct mapping *mapping)
{
	struct drm_etnaviv_gem_cpu_fini fini = { 0 };
	fini.handle = bo->handles[0].u32;

	if (drmIoctl(bo->drv->fd, DRM_IOCTL_ETNAVIV_GEM_CPU_FINI, &fini) < 0)
		return -errno;

	return 0;
}

static uint32_t etnaviv_resolve_format(uint32_t format, uint64_t use_flags)
{
	switch (format) {
	case DRM_FORMAT_FLEX_IMPLEMENTATION_DEFINED:
		if (use_flags & (BO_USE_CAMERA_READ | BO_USE_CAMERA_WRITE |
				 BO_USE_HW_VIDEO_DECODER | BO_USE_HW_VIDEO_ENCODER))
			return DRM_FORMAT_YVU420_ANDROID;
		return DRM_FORMAT_XBGR8888;
	case DRM_FORMAT_FLEX_YCbCr_420_888:
		return DRM_FORMAT_YVU420_ANDROID;
	default:
		return format;
	}
}

const struct backend backend_etnaviv = {
	.name = "etnaviv",
	.init = etnaviv_init,
	.close = etnaviv_close,
	.bo_create = etnaviv_bo_create,
	.bo_destroy = drv_gem_bo_destroy,
	.bo_import = drv_prime_bo_import,
	.bo_map = etnaviv_bo_map,
	.bo_unmap = drv_bo_munmap,
	.bo_invalidate = etnaviv_bo_invalidate,
	.bo_flush = etnaviv_bo_flush,
	.resolve_format = etnaviv_resolve_format,
};

#endif /* DRV_ETNAVIV */
