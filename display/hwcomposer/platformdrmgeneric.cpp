/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "hwc-platform-drm-generic"

#include "drmresources.h"
#include "platform.h"
#include "platformdrmgeneric.h"

#include <drm/drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <cutils/log.h>
#include <hardware/gralloc.h>
#include <EGL/eglext.h>

#include <vector>

namespace android {

namespace {

enum {
  GRALLOC_DRM_GET_DIMENSIONS = 2,
  CROS_GRALLOC_DRM_GET_BUFFER_INFO = 4,
};

struct cros_gralloc0_buffer_info {
  uint32_t drm_fourcc;
  int num_fds;
  int fds[4];
  uint64_t modifier;
  int offset[4];
  int stride[4];
};

}  // namespace

#ifdef USE_DRM_GENERIC_IMPORTER
// static
Importer *Importer::CreateInstance(DrmResources *drm) {
  DrmGenericImporter *importer = new DrmGenericImporter(drm);
  if (!importer)
    return NULL;

  int ret = importer->Init();
  if (ret) {
    ALOGE("Failed to initialize the drm generic importer %d", ret);
    delete importer;
    return NULL;
  }
  return importer;
}
#endif

DrmGenericImporter::DrmGenericImporter(DrmResources *drm) : drm_(drm) {
}

DrmGenericImporter::~DrmGenericImporter() {
}

int DrmGenericImporter::Init() {
  int ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID,
                          (const hw_module_t **)&gralloc_);
  if (ret) {
    ALOGE("Failed to open gralloc module");
    return ret;
  }
  if (!gralloc_->perform) {
    ALOGE("Gralloc module lacks perform()");
    return -ENODEV;
  }
  return 0;
}

EGLImageKHR DrmGenericImporter::ImportImage(EGLDisplay egl_display,
                                            buffer_handle_t handle) {
  struct cros_gralloc0_buffer_info info;
  uint32_t width = 0, height = 0;

  if (gralloc_->perform(gralloc_, CROS_GRALLOC_DRM_GET_BUFFER_INFO, handle,
                        &info) != 0) {
    ALOGE("CROS_GRALLOC_DRM_GET_BUFFER_INFO failed for handle %p", handle);
    return EGL_NO_IMAGE_KHR;
  }
  if (gralloc_->perform(gralloc_, GRALLOC_DRM_GET_DIMENSIONS, handle, &width,
                        &height) != 0) {
    ALOGE("GRALLOC_DRM_GET_DIMENSIONS failed for handle %p", handle);
    return EGL_NO_IMAGE_KHR;
  }

  std::vector<EGLint> attr = {
      EGL_WIDTH,
      static_cast<EGLint>(width),
      EGL_HEIGHT,
      static_cast<EGLint>(height),
      EGL_LINUX_DRM_FOURCC_EXT,
      static_cast<EGLint>(info.drm_fourcc),
  };

  if (info.num_fds >= 1) {
    attr.push_back(EGL_DMA_BUF_PLANE0_FD_EXT);
    attr.push_back(info.fds[0]);
    attr.push_back(EGL_DMA_BUF_PLANE0_OFFSET_EXT);
    attr.push_back(info.offset[0]);
    attr.push_back(EGL_DMA_BUF_PLANE0_PITCH_EXT);
    attr.push_back(info.stride[0]);
  }
  if (info.num_fds >= 2) {
    attr.push_back(EGL_DMA_BUF_PLANE1_FD_EXT);
    attr.push_back(info.fds[1]);
    attr.push_back(EGL_DMA_BUF_PLANE1_OFFSET_EXT);
    attr.push_back(info.offset[1]);
    attr.push_back(EGL_DMA_BUF_PLANE1_PITCH_EXT);
    attr.push_back(info.stride[1]);
  }
  if (info.num_fds >= 3) {
    attr.push_back(EGL_DMA_BUF_PLANE2_FD_EXT);
    attr.push_back(info.fds[2]);
    attr.push_back(EGL_DMA_BUF_PLANE2_OFFSET_EXT);
    attr.push_back(info.offset[2]);
    attr.push_back(EGL_DMA_BUF_PLANE2_PITCH_EXT);
    attr.push_back(info.stride[2]);
  }
  attr.push_back(EGL_NONE);

  return eglCreateImageKHR(egl_display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT,
                           NULL, attr.data());
}

int DrmGenericImporter::ImportBuffer(buffer_handle_t handle, hwc_drm_bo_t *bo) {
  struct cros_gralloc0_buffer_info info;
  uint32_t width = 0, height = 0;

  int ret = gralloc_->perform(gralloc_, CROS_GRALLOC_DRM_GET_BUFFER_INFO,
                              handle, &info);
  if (ret) {
    ALOGE("CROS_GRALLOC_DRM_GET_BUFFER_INFO failed (%d)", ret);
    return ret;
  }

  ret = gralloc_->perform(gralloc_, GRALLOC_DRM_GET_DIMENSIONS, handle, &width,
                          &height);
  if (ret) {
    ALOGE("GRALLOC_DRM_GET_DIMENSIONS failed (%d)", ret);
    return ret;
  }

  if (info.num_fds <= 0 || info.num_fds > 4) {
    ALOGE("Invalid num_fds %d from gralloc", info.num_fds);
    return -EINVAL;
  }

  memset(bo, 0, sizeof(hwc_drm_bo_t));
  bo->width = width;
  bo->height = height;
  bo->format = info.drm_fourcc;

  {
    std::lock_guard<std::mutex> lock(gem_handles_lock_);
    for (int i = 0; i < info.num_fds; i++) {
      uint32_t gem_handle = 0;
      ret = drmPrimeFDToHandle(drm_->fd(), info.fds[i], &gem_handle);
      if (ret) {
        ALOGE("failed to import prime fd %d (plane %d) ret=%d", info.fds[i], i,
              ret);
        for (int j = 0; j < i; j++) {
          uint32_t h = bo->gem_handles[j];
          bo->gem_handles[j] = 0;
          auto it = gem_handles_.find(h);
          if (it != gem_handles_.end() && --it->second == 0) {
            gem_handles_.erase(it);
            struct drm_gem_close gem_close = {};
            gem_close.handle = h;
            drmIoctl(drm_->fd(), DRM_IOCTL_GEM_CLOSE, &gem_close);
          }
        }
        return ret;
      }
      gem_handles_[gem_handle]++;
      bo->gem_handles[i] = gem_handle;
      bo->pitches[i] = static_cast<uint32_t>(info.stride[i]);
      bo->offsets[i] = static_cast<uint32_t>(info.offset[i]);
    }
  }

  ret = drmModeAddFB2(drm_->fd(), bo->width, bo->height, bo->format,
                      bo->gem_handles, bo->pitches, bo->offsets, &bo->fb_id, 0);
  if (ret) {
    ALOGE("could not create drm fb %d (w=%u h=%u fmt=0x%x pitch=%u gem=%u)",
          ret, bo->width, bo->height, bo->format, bo->pitches[0],
          bo->gem_handles[0]);
    ReleaseBuffer(bo);
    return ret;
  }

  return 0;
}

int DrmGenericImporter::ReleaseBuffer(hwc_drm_bo_t *bo) {
  if (bo->fb_id) {
    if (drmModeRmFB(drm_->fd(), bo->fb_id))
      ALOGE("Failed to rm fb");
    bo->fb_id = 0;
  }

  struct drm_gem_close gem_close;
  memset(&gem_close, 0, sizeof(gem_close));
  int num_gem_handles = sizeof(bo->gem_handles) / sizeof(bo->gem_handles[0]);
  std::lock_guard<std::mutex> lock(gem_handles_lock_);
  for (int i = 0; i < num_gem_handles; i++) {
    if (!bo->gem_handles[i])
      continue;

    uint32_t handle = bo->gem_handles[i];
    bo->gem_handles[i] = 0;

    auto it = gem_handles_.find(handle);
    if (it != gem_handles_.end() && --it->second > 0)
      continue;
    if (it != gem_handles_.end())
      gem_handles_.erase(it);

    gem_close.handle = handle;
    int ret = drmIoctl(drm_->fd(), DRM_IOCTL_GEM_CLOSE, &gem_close);
    if (ret)
      ALOGE("Failed to close gem handle %d %d", i, ret);
  }
  return 0;
}

#ifdef USE_DRM_GENERIC_IMPORTER
std::unique_ptr<Planner> Planner::CreateInstance(DrmResources *) {
  std::unique_ptr<Planner> planner(new Planner);
  planner->AddStage<PlanStageGreedy>();
  return planner;
}
#endif
}
