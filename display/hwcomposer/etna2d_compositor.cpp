/*
 * Copyright (C) 2026 The LineageOS Project
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

#define LOG_TAG "hwc-etna-2d"

#include "etna2d_compositor.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <vector>

#include <cutils/log.h>
#include <drm/drm_fourcc.h>
extern "C" {
#include <etnaviv_drmif.h>
}
#include <drm/etnaviv_drm.h>
#include <sync/sync.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "drmhwcgralloc.h"
#include "platform.h"
#include "state_2d.xml.h"

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

constexpr uint32_t kChipFeaturePipe2D = 0x00000200;
constexpr uint32_t kChipMinor02DPE20 = 0x00000080;

constexpr uint32_t kFeOpLoadState = 0x08000000;
constexpr uint32_t kFeOpDraw2D = 0x20000000;
constexpr uint32_t kFeOpStall = 0x48000000;

constexpr uint32_t kSyncRecipientFE = 0x1;
constexpr uint32_t kSyncRecipientPE = 0x7;

constexpr uint32_t kRegGlSemaphoreToken = 0x00003808;
constexpr uint32_t kRegGlFlushCache = 0x0000380c;
constexpr uint32_t kGlFlushCachePe2D = 0x00000008;

static inline uint32_t LoadStateHeader(uint32_t reg, uint32_t count) {
  return kFeOpLoadState | ((count & 0x3ff) << 16) | ((reg >> 2) & 0xffff);
}

struct CmdBuilder {
  std::vector<uint32_t> words;
  std::vector<struct drm_etnaviv_gem_submit_bo> bos;
  std::vector<struct drm_etnaviv_gem_submit_reloc> relocs;

  uint32_t AddBo(struct etna_bo *bo, uint32_t flags) {
    uint32_t handle = etna_bo_handle(bo);
    for (size_t i = 0; i < bos.size(); ++i) {
      if (bos[i].handle == handle) {
        bos[i].flags |= flags;
        return static_cast<uint32_t>(i);
      }
    }
    struct drm_etnaviv_gem_submit_bo entry = {};
    entry.flags = flags;
    entry.handle = handle;
    bos.push_back(entry);
    return static_cast<uint32_t>(bos.size() - 1);
  }

  void EmitState(uint32_t reg, uint32_t value) {
    words.push_back(LoadStateHeader(reg, 1));
    words.push_back(value);
  }

  void EmitStateReloc(uint32_t reg, struct etna_bo *bo, uint32_t offset,
                      uint32_t bo_flags) {
    words.push_back(LoadStateHeader(reg, 1));
    uint32_t idx = AddBo(bo, bo_flags);
    struct drm_etnaviv_gem_submit_reloc r = {};
    r.submit_offset =
        static_cast<uint32_t>(words.size() * sizeof(uint32_t));
    r.reloc_idx = idx;
    r.reloc_offset = offset;
    relocs.push_back(r);
    words.push_back(0);
  }

  void EmitDraw2D(uint16_t left, uint16_t top, uint16_t right,
                  uint16_t bottom) {
    words.push_back(kFeOpDraw2D | (1u << 8));
    words.push_back(0);
    words.push_back((static_cast<uint32_t>(top) << 16) | left);
    words.push_back((static_cast<uint32_t>(bottom) << 16) | right);
  }

  void EmitStallPe() {
    uint32_t token = kSyncRecipientFE | (kSyncRecipientPE << 8);
    EmitState(kRegGlSemaphoreToken, token);
    words.push_back(kFeOpStall);
    words.push_back(token);
  }
};

struct Etna2dFormatDesc {
  uint32_t de_format;
  uint32_t de_opaque_format;
  uint32_t de_swizzle;
};

static bool MapDrmFormatToDe(uint32_t fourcc, Etna2dFormatDesc *out) {
  switch (fourcc) {
    case DRM_FORMAT_ARGB8888:
      *out = {DE_FORMAT_A8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_ARGB};
      return true;
    case DRM_FORMAT_XRGB8888:
      *out = {DE_FORMAT_X8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_ARGB};
      return true;
    case DRM_FORMAT_ABGR8888:
      *out = {DE_FORMAT_A8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_ABGR};
      return true;
    case DRM_FORMAT_XBGR8888:
      *out = {DE_FORMAT_X8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_ABGR};
      return true;
    case DRM_FORMAT_RGBA8888:
      *out = {DE_FORMAT_A8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_RGBA};
      return true;
    case DRM_FORMAT_BGRA8888:
      *out = {DE_FORMAT_A8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_BGRA};
      return true;
    case DRM_FORMAT_RGB565:
      *out = {DE_FORMAT_R5G6B5, DE_FORMAT_R5G6B5, DE_SWIZZLE_ARGB};
      return true;
    case DRM_FORMAT_BGR565:
      *out = {DE_FORMAT_R5G6B5, DE_FORMAT_R5G6B5, DE_SWIZZLE_ABGR};
      return true;
    default:
      return false;
  }
}

static uint32_t ComputeStretchFactor(uint32_t src_size, uint32_t dst_size) {
  if (dst_size <= 1 || src_size <= 1)
    return 0;
  return ((src_size - 1) << 16) / (dst_size - 1);
}

// Persistent CMA target slots are owned by Etna2dCompositor, so DrmHwcBuffer
// receives a no-op Importer to keep DrmHwcBuffer::operator->() valid without
// destroying the cached FB on frame retirement.
class UnownedFbImporter : public Importer {
 public:
  EGLImageKHR ImportImage(EGLDisplay /*egl_display*/,
                          buffer_handle_t /*handle*/) override {
    return EGL_NO_IMAGE_KHR;
  }
  int ImportBuffer(buffer_handle_t /*handle*/, hwc_drm_bo_t * /*bo*/) override {
    return -ENOTSUP;
  }
  int ReleaseBuffer(hwc_drm_bo_t * /*bo*/) override {
    return 0;
  }
};

static UnownedFbImporter &GetUnownedImporter() {
  static UnownedFbImporter instance;
  return instance;
}

static void MergeFence(UniqueFd *acc, int fd) {
  if (fd < 0)
    return;
  if (acc->get() < 0) {
    acc->Set(dup(fd));
    return;
  }
  int merged = sync_merge("hwc_etna2d", acc->get(), fd);
  if (merged >= 0) {
    acc->Set(merged);
  } else {
    sync_wait(acc->get(), 100);
    acc->Set(dup(fd));
  }
}

struct RectI {
  int l, t, r, b;
  bool operator==(const RectI &o) const {
    return l == o.l && t == o.t && r == o.r && b == o.b;
  }
  bool operator!=(const RectI &o) const {
    return !(*this == o);
  }
};

static void SubtractRect(const RectI &r, const RectI &occ,
                         std::vector<RectI> *out) {
  if (occ.r <= r.l || occ.l >= r.r || occ.b <= r.t || occ.t >= r.b) {
    out->push_back(r);
    return;
  }
  int mid_t = std::max(r.t, occ.t);
  int mid_b = std::min(r.b, occ.b);
  if (occ.t > r.t)
    out->push_back({r.l, r.t, r.r, occ.t});
  if (occ.b < r.b)
    out->push_back({r.l, occ.b, r.r, r.b});
  if (occ.l > r.l)
    out->push_back({r.l, mid_t, occ.l, mid_b});
  if (occ.r < r.r)
    out->push_back({occ.r, mid_t, r.r, mid_b});
}

}  // namespace

Etna2dCompositor::~Etna2dCompositor() {
  Shutdown();
}

void Etna2dCompositor::Shutdown() {
  for (CachedBo &c : bo_cache_) {
    if (c.bo)
      etna_bo_del(c.bo);
  }
  bo_cache_.clear();
  cache_seq_ = 0;

  for (size_t i = 0; i < kNumTargetSlots; ++i)
    DestroyTargetSlot(&slots_[i]);
  active_slot_ = -1;

  if (solid_bo_) {
    etna_bo_del(solid_bo_);
    solid_bo_ = nullptr;
    solid_map_ = nullptr;
    solid_cursor_ = 0;
  }

  if (stream_) {
    etna_cmd_stream_del(stream_);
    stream_ = nullptr;
  }
  if (etna_pipe_) {
    etna_pipe_del(etna_pipe_);
    etna_pipe_ = nullptr;
  }
  if (etna_gpu_) {
    etna_gpu_del(etna_gpu_);
    etna_gpu_ = nullptr;
  }
  if (etna_dev_) {
    etna_device_del(etna_dev_);
    etna_dev_ = nullptr;
  }
  if (etna_fd_ >= 0) {
    close(etna_fd_);
    etna_fd_ = -1;
  }
  available_ = false;
}

int Etna2dCompositor::Init(int drm_fd, const gralloc_module_t *gralloc) {
  Shutdown();
  drm_fd_ = drm_fd;
  gralloc_ = gralloc;
  if (drm_fd_ < 0 || !gralloc_ || !gralloc_->perform)
    return -EINVAL;

  for (int i = 128; i < 192; ++i) {
    char node[64];
    snprintf(node, sizeof(node), "/dev/dri/renderD%d", i);
    int fd = open(node, O_RDWR | O_CLOEXEC);
    if (fd < 0)
      continue;

    drmVersionPtr ver = drmGetVersion(fd);
    bool is_etnaviv = ver && ver->name && strcmp(ver->name, "etnaviv") == 0;
    if (ver)
      drmFreeVersion(ver);
    if (!is_etnaviv) {
      close(fd);
      continue;
    }

    etna_fd_ = fd;
    break;
  }

  if (etna_fd_ < 0) {
    ALOGW("No etnaviv render node found for 2D composition");
    return -ENODEV;
  }

  etna_dev_ = etna_device_new(etna_fd_);
  if (!etna_dev_) {
    ALOGE("Failed to create etna_device");
    Shutdown();
    return -ENODEV;
  }

  etna_gpu_ = etna_gpu_new(etna_dev_, 0);
  if (!etna_gpu_) {
    ALOGE("Failed to open etna_gpu core 0");
    Shutdown();
    return -ENODEV;
  }

  uint64_t feat0 = 0, feat1 = 0, feat2 = 0, feat3 = 0;
  if (etna_gpu_get_param(etna_gpu_, ETNA_GPU_FEATURES_0, &feat0) ||
      etna_gpu_get_param(etna_gpu_, ETNA_GPU_FEATURES_1, &feat1)) {
    ALOGE("Failed to query etnaviv GPU features");
    Shutdown();
    return -ENODEV;
  }
  etna_gpu_get_param(etna_gpu_, ETNA_GPU_FEATURES_2, &feat2);
  etna_gpu_get_param(etna_gpu_, ETNA_GPU_FEATURES_3, &feat3);

  if (!(feat0 & kChipFeaturePipe2D) || !(feat1 & kChipMinor02DPE20)) {
    ALOGW("Vivante GPU lacks PIPE_2D/2DPE20 (f0=0x%llx f1=0x%llx)",
          static_cast<unsigned long long>(feat0),
          static_cast<unsigned long long>(feat1));
    Shutdown();
    return -ENOTSUP;
  }

  etna_pipe_ = etna_pipe_new(etna_gpu_, static_cast<enum etna_pipe_id>(1));
  if (!etna_pipe_) {
    ALOGE("Failed to create ETNA_PIPE_2D");
    Shutdown();
    return -ENODEV;
  }

  stream_ = etna_cmd_stream_new(etna_pipe_, 0x2000, nullptr, nullptr);
  if (!stream_) {
    ALOGE("Failed to create etna_cmd_stream");
    Shutdown();
    return -ENOMEM;
  }

  solid_bo_ = etna_bo_new(etna_dev_, kSolidRingSize * sizeof(uint32_t),
                          DRM_ETNA_GEM_CACHE_WC);
  if (solid_bo_) {
    solid_map_ = static_cast<uint32_t *>(etna_bo_map(solid_bo_));
    if (!solid_map_) {
      etna_bo_del(solid_bo_);
      solid_bo_ = nullptr;
    }
  }
  if (!solid_bo_)
    ALOGW("Failed to allocate solid color ring BO; SolidColor layers disabled");

  available_ = true;
  ALOGI("Initialized Vivante ETNA_PIPE_2D (2DPE20) HWC compositor "
        "(f0=0x%llx f1=0x%llx f2=0x%llx f3=0x%llx)",
        static_cast<unsigned long long>(feat0),
        static_cast<unsigned long long>(feat1),
        static_cast<unsigned long long>(feat2),
        static_cast<unsigned long long>(feat3));
  return 0;
}

void Etna2dCompositor::DestroyTargetSlot(TargetSlot *slot) {
  if (!slot)
    return;
  if (slot->busy_fence.get() >= 0) {
    sync_wait(slot->busy_fence.get(), 100);
    slot->busy_fence.Close();
  }
  if (slot->bo) {
    etna_bo_del(slot->bo);
    slot->bo = nullptr;
  }
  if (slot->fb_id_xbgr) {
    drmModeRmFB(drm_fd_, slot->fb_id_xbgr);
    slot->fb_id_xbgr = 0;
  }
  if (slot->fb_id_abgr) {
    drmModeRmFB(drm_fd_, slot->fb_id_abgr);
    slot->fb_id_abgr = 0;
  }
  if (slot->gem_handle) {
    struct drm_mode_destroy_dumb destroy = {};
    destroy.handle = slot->gem_handle;
    drmIoctl(drm_fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    slot->gem_handle = 0;
  }
  slot->width = 0;
  slot->height = 0;
  slot->pitch = 0;
  slot->size = 0;
}

int Etna2dCompositor::EnsureTargetSlot(size_t slot_index, uint32_t width,
                                       uint32_t height) {
  TargetSlot &slot = slots_[slot_index];
  if (slot.width == width && slot.height == height && slot.bo &&
      slot.fb_id_xbgr && slot.fb_id_abgr)
    return 0;

  DestroyTargetSlot(&slot);

  struct drm_mode_create_dumb create = {};
  create.width = width;
  create.height = height;
  create.bpp = 32;
  if (drmIoctl(drm_fd_, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
    ALOGE("Failed to create dumb buffer (%ux%u): %d", width, height, errno);
    return -errno;
  }

  slot.gem_handle = create.handle;
  slot.pitch = create.pitch;
  slot.size = static_cast<uint32_t>(create.size);
  slot.width = width;
  slot.height = height;

  uint32_t handles[4] = {slot.gem_handle, 0, 0, 0};
  uint32_t pitches[4] = {slot.pitch, 0, 0, 0};
  uint32_t offsets[4] = {0, 0, 0, 0};

  int ret = drmModeAddFB2(drm_fd_, width, height, DRM_FORMAT_XBGR8888, handles,
                          pitches, offsets, &slot.fb_id_xbgr, 0);
  if (ret) {
    ALOGE("Failed to add XBGR FB2 for slot %zu: %d", slot_index, ret);
    DestroyTargetSlot(&slot);
    return ret;
  }

  ret = drmModeAddFB2(drm_fd_, width, height, DRM_FORMAT_ABGR8888, handles,
                      pitches, offsets, &slot.fb_id_abgr, 0);
  if (ret) {
    ALOGE("Failed to add ABGR FB2 for slot %zu: %d", slot_index, ret);
    DestroyTargetSlot(&slot);
    return ret;
  }

  struct drm_prime_handle prime = {};
  prime.handle = slot.gem_handle;
  prime.flags = DRM_CLOEXEC | DRM_RDWR;
  if (drmIoctl(drm_fd_, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime) < 0) {
    prime.flags = DRM_CLOEXEC;
    if (drmIoctl(drm_fd_, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime) < 0) {
      ret = -errno;
      ALOGE("Failed to export prime fd for slot %zu: %d", slot_index, ret);
      DestroyTargetSlot(&slot);
      return ret;
    }
  }

  slot.bo = etna_bo_from_dmabuf(etna_dev_, prime.fd);
  close(prime.fd);
  if (!slot.bo) {
    ALOGE("Failed to import target CMA buffer into etnaviv");
    DestroyTargetSlot(&slot);
    return -ENOMEM;
  }

  return 0;
}

struct etna_bo *Etna2dCompositor::GetOrImportSourceBo(int dmabuf_fd,
                                                      uint32_t buf_w,
                                                      uint32_t buf_h,
                                                      uint32_t fourcc,
                                                      uint32_t offset,
                                                      uint32_t stride,
                                                      bool *out_owned) {
  *out_owned = false;
  struct stat st = {};
  bool has_inode = (fstat(dmabuf_fd, &st) == 0 && st.st_ino != 0);
  uint64_t dev = static_cast<uint64_t>(st.st_dev);
  uint64_t ino = static_cast<uint64_t>(st.st_ino);

  if (has_inode) {
    for (CachedBo &c : bo_cache_) {
      if (c.dev == dev && c.ino == ino) {
        c.last_used = ++cache_seq_;
        ++cache_hits_;
        return c.bo;
      }
    }
  }
  ++cache_misses_;

  struct etna_bo *bo = etna_bo_from_dmabuf(etna_dev_, dmabuf_fd);
  if (!bo)
    return nullptr;

  if (!has_inode) {
    *out_owned = true;
    return bo;
  }

  if (bo_cache_.size() >= kMaxCachedBos) {
    auto victim = std::min_element(
        bo_cache_.begin(), bo_cache_.end(),
        [](const CachedBo &a, const CachedBo &b) {
          return a.last_used < b.last_used;
        });
    if (victim->bo)
      etna_bo_del(victim->bo);
    victim->dev = dev;
    victim->ino = ino;
    victim->width = buf_w;
    victim->height = buf_h;
    victim->fourcc = fourcc;
    victim->offset = offset;
    victim->stride = stride;
    victim->bo = bo;
    victim->last_used = ++cache_seq_;
  } else {
    CachedBo entry;
    entry.dev = dev;
    entry.ino = ino;
    entry.width = buf_w;
    entry.height = buf_h;
    entry.fourcc = fourcc;
    entry.offset = offset;
    entry.stride = stride;
    entry.bo = bo;
    entry.last_used = ++cache_seq_;
    bo_cache_.push_back(entry);
  }
  return bo;
}

bool Etna2dCompositor::IsLayerSupported(HWC2::Composition sf_type,
                                        buffer_handle_t handle,
                                        HWC2::BlendMode blending,
                                        HWC2::Transform transform,
                                        const hwc_rect_t &display_frame,
                                        const hwc_frect_t &source_crop,
                                        uint32_t display_w,
                                        uint32_t display_h) const {
  if (!available_)
    return false;

  if (blending != HWC2::BlendMode::None &&
      blending != HWC2::BlendMode::Premultiplied &&
      blending != HWC2::BlendMode::Coverage)
    return false;

  int dst_l = std::max(0, display_frame.left);
  int dst_t = std::max(0, display_frame.top);
  int dst_r = std::min(static_cast<int>(display_w), display_frame.right);
  int dst_b = std::min(static_cast<int>(display_h), display_frame.bottom);
  if (dst_r <= dst_l || dst_b <= dst_t)
    return false;

  if (sf_type == HWC2::Composition::SolidColor)
    return solid_bo_ != nullptr && solid_map_ != nullptr;

  if (!handle)
    return false;

  if (sf_type != HWC2::Composition::Device &&
      sf_type != HWC2::Composition::Cursor)
    return false;

  // PE 2.0 supports 0/90/180/270 rotations and H/V mirrors; fall back to
  // Client only for combined flip + 90-degree transforms.
  int32_t tr = static_cast<int32_t>(transform);
  if ((tr & static_cast<int32_t>(HWC2::Transform::Rotate90)) &&
      transform != HWC2::Transform::Rotate90 &&
      transform != HWC2::Transform::Rotate270)
    return false;

  if (source_crop.right <= source_crop.left ||
      source_crop.bottom <= source_crop.top)
    return false;

  struct cros_gralloc0_buffer_info info = {};
  if (gralloc_->perform(gralloc_, CROS_GRALLOC_DRM_GET_BUFFER_INFO, handle,
                        &info) != 0)
    return false;
  if (info.num_fds != 1 || info.fds[0] < 0 || info.modifier != 0)
    return false;

  Etna2dFormatDesc fmt_desc = {};
  if (!MapDrmFormatToDe(info.drm_fourcc, &fmt_desc))
    return false;

  return true;
}

int Etna2dCompositor::Composite(const std::vector<Etna2dLayerInput> &layers,
                                uint32_t width, uint32_t height,
                                bool transparent_bg, DrmHwcLayer *out_layer,
                                int *out_fence_fd, size_t *out_slot_index) {
  if (out_fence_fd)
    *out_fence_fd = -1;
  if (!available_ || layers.empty() || !out_layer || !width || !height)
    return -EINVAL;

  if (active_slot_ >= 0 &&
      next_slot_ == static_cast<size_t>(active_slot_)) {
    next_slot_ = (next_slot_ + 1) % kNumTargetSlots;
  }
  size_t slot_idx = next_slot_;
  next_slot_ = (next_slot_ + 1) % kNumTargetSlots;

  int ret = EnsureTargetSlot(slot_idx, width, height);
  if (ret)
    return ret;

  TargetSlot &slot = slots_[slot_idx];
  UniqueFd merged_in_fence;
  if (slot.busy_fence.get() >= 0) {
    MergeFence(&merged_in_fence, slot.busy_fence.get());
    slot.busy_fence.Close();
  }

  std::vector<struct etna_bo *> temp_bos;
  temp_bos.reserve(layers.size());
  struct PreparedLayer {
    struct etna_bo *bo;
    uint32_t offset;
    uint32_t stride;
    Etna2dFormatDesc fmt;
    HWC2::BlendMode blending;
    uint8_t alpha;
    HWC2::Transform transform;
    bool is_solid_color;
    hwc_color_t color;
    int dst_l, dst_t, dst_r, dst_b;
    int src_l, src_t, src_w, src_h;
    std::vector<RectI> visible_rects;
  };
  std::vector<PreparedLayer> prepared;
  prepared.reserve(layers.size());

  auto cleanup_bos = [&]() {
    for (struct etna_bo *bo : temp_bos)
      etna_bo_del(bo);
    temp_bos.clear();
  };

  for (const Etna2dLayerInput &l : layers) {
    int raw_dw = l.display_frame.right - l.display_frame.left;
    int raw_dh = l.display_frame.bottom - l.display_frame.top;
    if (raw_dw <= 0 || raw_dh <= 0)
      continue;

    int dst_l = std::max(0, l.display_frame.left);
    int dst_t = std::max(0, l.display_frame.top);
    int dst_r = std::min(static_cast<int>(width), l.display_frame.right);
    int dst_b = std::min(static_cast<int>(height), l.display_frame.bottom);
    if (dst_r <= dst_l || dst_b <= dst_t)
      continue;

    if (l.is_solid_color) {
      if (!solid_bo_ || !solid_map_)
        continue;
      size_t idx = solid_cursor_;
      solid_cursor_ = (solid_cursor_ + 1) % kSolidRingSize;

      uint8_t pr = l.color.r;
      uint8_t pg = l.color.g;
      uint8_t pb = l.color.b;
      uint8_t pa = l.color.a;
      if (l.blending == HWC2::BlendMode::Premultiplied && pa < 255) {
        pr = static_cast<uint8_t>((static_cast<uint32_t>(pr) * pa + 127) / 255);
        pg = static_cast<uint8_t>((static_cast<uint32_t>(pg) * pa + 127) / 255);
        pb = static_cast<uint8_t>((static_cast<uint32_t>(pb) * pa + 127) / 255);
      } else if (l.blending == HWC2::BlendMode::None) {
        pa = 0xFF;
      }
      solid_map_[idx] = (static_cast<uint32_t>(pa) << 24) |
                        (static_cast<uint32_t>(pb) << 16) |
                        (static_cast<uint32_t>(pg) << 8) |
                        static_cast<uint32_t>(pr);
      __sync_synchronize();

      PreparedLayer p = {};
      p.bo = solid_bo_;
      p.offset = static_cast<uint32_t>(idx * sizeof(uint32_t));
      p.stride = 4;
      p.fmt = {DE_FORMAT_A8R8G8B8, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_ABGR};
      p.blending = l.blending;
      p.alpha = l.alpha;
      p.transform = HWC2::Transform::None;
      p.is_solid_color = true;
      p.color = l.color;
      p.dst_l = dst_l;
      p.dst_t = dst_t;
      p.dst_r = dst_r;
      p.dst_b = dst_b;
      p.src_l = 0;
      p.src_t = 0;
      p.src_w = 1;
      p.src_h = 1;
      prepared.push_back(std::move(p));
      continue;
    }

    struct cros_gralloc0_buffer_info info = {};
    if (gralloc_->perform(gralloc_, CROS_GRALLOC_DRM_GET_BUFFER_INFO, l.handle,
                          &info) != 0 ||
        info.num_fds < 1 || info.fds[0] < 0) {
      ALOGE("Composite: CROS_GRALLOC_DRM_GET_BUFFER_INFO failed handle=%p",
            l.handle);
      cleanup_bos();
      return -EINVAL;
    }

    uint32_t buf_w = 0, buf_h = 0;
    if (gralloc_->perform(gralloc_, GRALLOC_DRM_GET_DIMENSIONS, l.handle,
                          &buf_w, &buf_h) != 0 ||
        !buf_w || !buf_h) {
      ALOGE("Composite: GRALLOC_DRM_GET_DIMENSIONS failed handle=%p",
            l.handle);
      cleanup_bos();
      return -EINVAL;
    }

    Etna2dFormatDesc fmt = {};
    if (!MapDrmFormatToDe(info.drm_fourcc, &fmt)) {
      ALOGE("Composite: unsupported fourcc 0x%x", info.drm_fourcc);
      cleanup_bos();
      return -EINVAL;
    }

    float crop_w = l.source_crop.right - l.source_crop.left;
    float crop_h = l.source_crop.bottom - l.source_crop.top;
    bool is_rot90 = (static_cast<int32_t>(l.transform) &
                     static_cast<int32_t>(HWC2::Transform::Rotate90)) != 0;
    float sx_ratio = crop_w / static_cast<float>(is_rot90 ? raw_dh : raw_dw);
    float sy_ratio = crop_h / static_cast<float>(is_rot90 ? raw_dw : raw_dh);

    int clip_l = dst_l - l.display_frame.left;
    int clip_t = dst_t - l.display_frame.top;
    int clip_r = l.display_frame.right - dst_r;
    int clip_b = l.display_frame.bottom - dst_b;

    float clipped_sl, clipped_st, clipped_sr, clipped_sb;
    if (l.transform == HWC2::Transform::Rotate270) {
      clipped_sl = l.source_crop.left + clip_b * sx_ratio;
      clipped_sr = l.source_crop.right - clip_t * sx_ratio;
      clipped_st = l.source_crop.top + clip_l * sy_ratio;
      clipped_sb = l.source_crop.bottom - clip_r * sy_ratio;
    } else if (l.transform == HWC2::Transform::Rotate90) {
      clipped_sl = l.source_crop.left + clip_t * sx_ratio;
      clipped_sr = l.source_crop.right - clip_b * sx_ratio;
      clipped_st = l.source_crop.top + clip_r * sy_ratio;
      clipped_sb = l.source_crop.bottom - clip_l * sy_ratio;
    } else {
      bool fh = (static_cast<int32_t>(l.transform) &
                 static_cast<int32_t>(HWC2::Transform::FlipH)) != 0;
      bool fv = (static_cast<int32_t>(l.transform) &
                 static_cast<int32_t>(HWC2::Transform::FlipV)) != 0;
      clipped_sl = l.source_crop.left + (fh ? clip_r : clip_l) * sx_ratio;
      clipped_sr = l.source_crop.right - (fh ? clip_l : clip_r) * sx_ratio;
      clipped_st = l.source_crop.top + (fv ? clip_b : clip_t) * sy_ratio;
      clipped_sb = l.source_crop.bottom - (fv ? clip_t : clip_b) * sy_ratio;
    }

    int src_l, src_t, src_r, src_b;
    float exp_w = static_cast<float>(is_rot90 ? raw_dh : raw_dw);
    float exp_h = static_cast<float>(is_rot90 ? raw_dw : raw_dh);
    if (fabsf(crop_w - exp_w) < 0.01f && fabsf(crop_h - exp_h) < 0.01f) {
      src_l = std::max(0, static_cast<int>(lroundf(clipped_sl)));
      src_t = std::max(0, static_cast<int>(lroundf(clipped_st)));
      int span_w = is_rot90 ? (dst_b - dst_t) : (dst_r - dst_l);
      int span_h = is_rot90 ? (dst_r - dst_l) : (dst_b - dst_t);
      src_r = std::min(static_cast<int>(buf_w), src_l + span_w);
      src_b = std::min(static_cast<int>(buf_h), src_t + span_h);
    } else {
      src_l = std::max(0, static_cast<int>(floorf(clipped_sl)));
      src_t = std::max(0, static_cast<int>(floorf(clipped_st)));
      src_r = std::min(static_cast<int>(buf_w),
                       static_cast<int>(ceilf(clipped_sr)));
      src_b = std::min(static_cast<int>(buf_h),
                       static_cast<int>(ceilf(clipped_sb)));
    }
    if (src_r <= src_l || src_b <= src_t)
      continue;

    bool owned_bo = false;
    struct etna_bo *bo = GetOrImportSourceBo(
        info.fds[0], buf_w, buf_h, info.drm_fourcc,
        static_cast<uint32_t>(info.offset[0]),
        static_cast<uint32_t>(info.stride[0]), &owned_bo);
    if (!bo) {
      ALOGE("Failed to import layer dma-buf fd %d into etnaviv", info.fds[0]);
      cleanup_bos();
      return -ENOMEM;
    }
    if (owned_bo)
      temp_bos.push_back(bo);
    MergeFence(&merged_in_fence, l.acquire_fence);

    PreparedLayer p = {};
    p.bo = bo;
    p.offset = static_cast<uint32_t>(info.offset[0]);
    p.stride = static_cast<uint32_t>(info.stride[0]);
    p.fmt = fmt;
    p.blending = l.blending;
    p.alpha = l.alpha;
    p.transform = l.transform;
    p.is_solid_color = false;
    p.dst_l = dst_l;
    p.dst_t = dst_t;
    p.dst_r = dst_r;
    p.dst_b = dst_b;
    p.src_l = src_l;
    p.src_t = src_t;
    p.src_w = src_r - src_l;
    p.src_h = src_b - src_t;
    prepared.push_back(std::move(p));
  }

  // Front-to-back opaque occlusion culling & partial blit rect splitting.
  bool skip_clear = false;
  std::vector<RectI> occluders;
  for (size_t ri = prepared.size(); ri-- > 0;) {
    PreparedLayer &p = prepared[ri];
    RectI full = {p.dst_l, p.dst_t, p.dst_r, p.dst_b};
    std::vector<RectI> visible = {full};
    for (const RectI &occ : occluders) {
      std::vector<RectI> next;
      next.reserve(visible.size() * 2);
      for (const RectI &r : visible)
        SubtractRect(r, occ, &next);
      visible.swap(next);
      if (visible.empty() || visible.size() > 32)
        break;
    }
    if (visible.empty()) {
      ++culled_layers_;
      continue;
    }
    if (visible.size() > 32 ||
        (p.transform != HWC2::Transform::None &&
         (visible.size() > 1 || visible[0] != full))) {
      p.visible_rects = {full};
    } else {
      p.visible_rects = std::move(visible);
    }

    bool is_opaque =
        (p.blending == HWC2::BlendMode::None && p.alpha == 0xFF) ||
        (p.is_solid_color && p.color.a == 0xFF && p.alpha == 0xFF);
    if (is_opaque) {
      occluders.push_back(full);
      if (full.l == 0 && full.t == 0 &&
          full.r == static_cast<int>(width) &&
          full.b == static_cast<int>(height)) {
        skip_clear = true;
      }
    }
  }

  CmdBuilder cmd;
  cmd.words.reserve(256);

  if (!skip_clear) {
    uint32_t clear_val = transparent_bg ? 0x00000000u : 0xFF000000u;
    cmd.EmitStateReloc(VIVS_DE_DEST_ADDRESS, slot.bo, 0,
                       ETNA_SUBMIT_BO_WRITE);
    cmd.EmitState(VIVS_DE_DEST_STRIDE,
                  VIVS_DE_DEST_STRIDE_STRIDE(slot.pitch));
    cmd.EmitState(VIVS_DE_DEST_ROTATION_CONFIG, 0);
    cmd.EmitState(VIVS_DE_DEST_CONFIG,
                  VIVS_DE_DEST_CONFIG_FORMAT(DE_FORMAT_A8R8G8B8) |
                      VIVS_DE_DEST_CONFIG_COMMAND_CLEAR |
                      VIVS_DE_DEST_CONFIG_SWIZZLE(DE_SWIZZLE_ABGR) |
                      VIVS_DE_DEST_CONFIG_TILED_DISABLE);
    cmd.EmitState(VIVS_DE_PATTERN_CONFIG,
                  VIVS_DE_PATTERN_CONFIG_TYPE_SOLID_COLOR);
    cmd.EmitState(VIVS_DE_ROP,
                  VIVS_DE_ROP_TYPE_ROP2_PATTERN | VIVS_DE_ROP_ROP_FG(0xF0) |
                      VIVS_DE_ROP_ROP_BG(0xF0));
    cmd.EmitState(VIVS_DE_CLIP_TOP_LEFT,
                  VIVS_DE_CLIP_TOP_LEFT_X(0) | VIVS_DE_CLIP_TOP_LEFT_Y(0));
    cmd.EmitState(VIVS_DE_CLIP_BOTTOM_RIGHT,
                  VIVS_DE_CLIP_BOTTOM_RIGHT_X(width) |
                      VIVS_DE_CLIP_BOTTOM_RIGHT_Y(height));
    cmd.EmitState(VIVS_DE_CONFIG, 0);
    cmd.EmitState(VIVS_DE_SRC_ROTATION_HEIGHT, 0);
    cmd.EmitState(VIVS_DE_DEST_ROTATION_HEIGHT, 0);
    cmd.EmitState(VIVS_DE_ROT_ANGLE, 0);
    cmd.EmitState(VIVS_DE_PE_CONFIG,
                  VIVS_DE_PE_CONFIG_DESTINATION_FETCH_DISABLE);
    cmd.EmitState(VIVS_DE_ALPHA_CONTROL, VIVS_DE_ALPHA_CONTROL_ENABLE_OFF);
    cmd.EmitState(VIVS_DE_COLOR_MULTIPLY_MODES, 0);
    cmd.EmitState(VIVS_DE_CLEAR_BYTE_MASK, 0xFF);
    cmd.EmitState(VIVS_DE_CLEAR_PIXEL_VALUE32, clear_val);
    cmd.EmitDraw2D(0, 0, static_cast<uint16_t>(width),
                   static_cast<uint16_t>(height));
  }

  for (const PreparedLayer &p : prepared) {
    if (p.visible_rects.empty())
      continue;

    uint32_t full_dw = static_cast<uint32_t>(p.dst_r - p.dst_l);
    uint32_t full_dh = static_cast<uint32_t>(p.dst_b - p.dst_t);
    int32_t tr = static_cast<int32_t>(p.transform);
    bool is_rot90 =
        (tr & static_cast<int32_t>(HWC2::Transform::Rotate90)) != 0;
    uint32_t target_w = is_rot90 ? full_dh : full_dw;
    uint32_t target_h = is_rot90 ? full_dw : full_dh;
    bool is_stretch = (static_cast<uint32_t>(p.src_w) != target_w ||
                       static_cast<uint32_t>(p.src_h) != target_h);

    bool need_blend = (p.blending == HWC2::BlendMode::Premultiplied) ||
                      (p.blending == HWC2::BlendMode::Coverage) ||
                      (p.alpha != 0xFF);
    uint32_t src_fmt =
        (p.blending == HWC2::BlendMode::None) ? p.fmt.de_opaque_format
                                              : p.fmt.de_format;

    uint32_t de_cfg = 0;
    uint32_t rot_angle = 0;
    uint32_t dst_rot_cfg = 0;
    uint32_t dst_rot_h = 0;
    if (is_rot90) {
      uint32_t rot_mode = (tr == static_cast<int32_t>(HWC2::Transform::Rotate270))
                              ? DE_ROT_MODE_ROT270
                              : DE_ROT_MODE_ROT90;
      rot_angle = VIVS_DE_ROT_ANGLE_SRC(DE_ROT_MODE_ROT0) |
                  VIVS_DE_ROT_ANGLE_DST(rot_mode);
      // On PE 2.0 full-rotation cores, VIVS_DE_ROT_ANGLE controls the angle;
      // DEST_ROTATION_CONFIG supplies the unrotated surface width with legacy
      // 90-degree ROTATION bit disabled (enabling both cancels 90+270=0).
      dst_rot_cfg = VIVS_DE_DEST_ROTATION_CONFIG_ROTATION_DISABLE |
                    VIVS_DE_DEST_ROTATION_CONFIG_WIDTH(width);
      dst_rot_h = VIVS_DE_DEST_ROTATION_HEIGHT_HEIGHT(height);
    } else {
      bool fh = (tr & static_cast<int32_t>(HWC2::Transform::FlipH)) != 0;
      bool fv = (tr & static_cast<int32_t>(HWC2::Transform::FlipV)) != 0;
      if (fh && fv) {
        de_cfg = VIVS_DE_CONFIG_MIRROR_BLT_ENABLE_ON |
                 VIVS_DE_CONFIG_MIRROR_BLT_MODE_FULL_MIRROR;
      } else if (fh) {
        de_cfg = VIVS_DE_CONFIG_MIRROR_BLT_ENABLE_ON |
                 VIVS_DE_CONFIG_MIRROR_BLT_MODE_HMIRROR;
      } else if (fv) {
        de_cfg = VIVS_DE_CONFIG_MIRROR_BLT_ENABLE_ON |
                 VIVS_DE_CONFIG_MIRROR_BLT_MODE_VMIRROR;
      }
    }

    bool is_partial =
        (p.visible_rects.size() > 1 ||
         p.visible_rects[0] != RectI{p.dst_l, p.dst_t, p.dst_r, p.dst_b});
    if (is_partial)
      partial_blits_ += p.visible_rects.size();

    for (const RectI &vr : p.visible_rects) {
      int sub_src_l = p.src_l;
      int sub_src_t = p.src_t;
      int sub_src_w = p.src_w;
      int sub_src_h = p.src_h;
      int vr_w = vr.r - vr.l;
      int vr_h = vr.b - vr.t;
      if (!p.is_solid_color &&
          (vr.l != p.dst_l || vr.t != p.dst_t || vr.r != p.dst_r ||
           vr.b != p.dst_b)) {
        if (!is_stretch) {
          sub_src_l = p.src_l + (vr.l - p.dst_l);
          sub_src_t = p.src_t + (vr.t - p.dst_t);
          sub_src_w = vr_w;
          sub_src_h = vr_h;
        } else {
          float sx = static_cast<float>(p.src_w) / static_cast<float>(full_dw);
          float sy = static_cast<float>(p.src_h) / static_cast<float>(full_dh);
          sub_src_l = p.src_l + static_cast<int>(floorf((vr.l - p.dst_l) * sx));
          sub_src_t = p.src_t + static_cast<int>(floorf((vr.t - p.dst_t) * sy));
          int sub_src_r =
              p.src_l + static_cast<int>(ceilf((vr.r - p.dst_l) * sx));
          int sub_src_b =
              p.src_t + static_cast<int>(ceilf((vr.b - p.dst_t) * sy));
          sub_src_w = std::max(1, sub_src_r - sub_src_l);
          sub_src_h = std::max(1, sub_src_b - sub_src_t);
        }
      }

      cmd.EmitStateReloc(VIVS_DE_SRC_ADDRESS, p.bo, p.offset,
                         ETNA_SUBMIT_BO_READ);
      cmd.EmitState(VIVS_DE_SRC_STRIDE,
                    VIVS_DE_SRC_STRIDE_STRIDE(p.stride));
      cmd.EmitState(VIVS_DE_SRC_ROTATION_CONFIG, 0);
      cmd.EmitState(VIVS_DE_SRC_EX_CONFIG, 0);
      cmd.EmitState(VIVS_DE_SRC_CONFIG,
                    VIVS_DE_SRC_CONFIG_PE10_SOURCE_FORMAT(src_fmt & 0xF) |
                        VIVS_DE_SRC_CONFIG_SOURCE_FORMAT(src_fmt) |
                        VIVS_DE_SRC_CONFIG_SWIZZLE(p.fmt.de_swizzle) |
                        VIVS_DE_SRC_CONFIG_SRC_RELATIVE_ABSOLUTE |
                        VIVS_DE_SRC_CONFIG_TILED_DISABLE |
                        VIVS_DE_SRC_CONFIG_LOCATION_MEMORY |
                        VIVS_DE_SRC_CONFIG_PACK_PACKED8);

      cmd.EmitState(VIVS_DE_SRC_ORIGIN,
                    VIVS_DE_SRC_ORIGIN_X(sub_src_l) |
                        VIVS_DE_SRC_ORIGIN_Y(sub_src_t));
      cmd.EmitState(VIVS_DE_SRC_SIZE,
                    VIVS_DE_SRC_SIZE_X(sub_src_w) |
                        VIVS_DE_SRC_SIZE_Y(sub_src_h));
      cmd.EmitState(VIVS_DE_SRC_ORIGIN_FRACTION, 0);

      if (is_stretch) {
        uint32_t dst_sw = is_rot90 ? static_cast<uint32_t>(vr_h)
                                   : static_cast<uint32_t>(vr_w);
        uint32_t dst_sh = is_rot90 ? static_cast<uint32_t>(vr_w)
                                   : static_cast<uint32_t>(vr_h);
        cmd.EmitState(VIVS_DE_STRETCH_FACTOR_LOW,
                      VIVS_DE_STRETCH_FACTOR_LOW_X(
                          ComputeStretchFactor(sub_src_w, dst_sw)));
        cmd.EmitState(VIVS_DE_STRETCH_FACTOR_HIGH,
                      VIVS_DE_STRETCH_FACTOR_HIGH_Y(
                          ComputeStretchFactor(sub_src_h, dst_sh)));
      }

      cmd.EmitStateReloc(VIVS_DE_DEST_ADDRESS, slot.bo, 0,
                         ETNA_SUBMIT_BO_WRITE);
      cmd.EmitState(VIVS_DE_DEST_STRIDE,
                    VIVS_DE_DEST_STRIDE_STRIDE(slot.pitch));
      cmd.EmitState(VIVS_DE_DEST_ROTATION_CONFIG, dst_rot_cfg);
      cmd.EmitState(VIVS_DE_SRC_ROTATION_HEIGHT, 0);
      cmd.EmitState(VIVS_DE_DEST_ROTATION_HEIGHT, dst_rot_h);
      cmd.EmitState(VIVS_DE_DEST_CONFIG,
                    VIVS_DE_DEST_CONFIG_FORMAT(DE_FORMAT_A8R8G8B8) |
                        (is_stretch ? VIVS_DE_DEST_CONFIG_COMMAND_STRETCH_BLT
                                    : VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT) |
                        VIVS_DE_DEST_CONFIG_SWIZZLE(DE_SWIZZLE_ABGR) |
                        VIVS_DE_DEST_CONFIG_TILED_DISABLE);

      int draw_l = vr.l;
      int draw_t = vr.t;
      int draw_r = vr.r;
      int draw_b = vr.b;
      if (p.transform == HWC2::Transform::Rotate270) {
        draw_l = static_cast<int>(height) - vr.b;
        draw_t = vr.l;
        draw_r = static_cast<int>(height) - vr.t;
        draw_b = vr.r;
      } else if (p.transform == HWC2::Transform::Rotate90) {
        draw_l = vr.t;
        draw_t = static_cast<int>(width) - vr.r;
        draw_r = vr.b;
        draw_b = static_cast<int>(width) - vr.l;
      }

      cmd.EmitState(VIVS_DE_CLIP_TOP_LEFT,
                    VIVS_DE_CLIP_TOP_LEFT_X(draw_l) |
                        VIVS_DE_CLIP_TOP_LEFT_Y(draw_t));
      cmd.EmitState(VIVS_DE_CLIP_BOTTOM_RIGHT,
                    VIVS_DE_CLIP_BOTTOM_RIGHT_X(draw_r) |
                        VIVS_DE_CLIP_BOTTOM_RIGHT_Y(draw_b));
      cmd.EmitState(VIVS_DE_CONFIG, de_cfg);
      cmd.EmitState(VIVS_DE_ROT_ANGLE, rot_angle);
      cmd.EmitState(VIVS_DE_ROP,
                    VIVS_DE_ROP_TYPE_ROP3 | VIVS_DE_ROP_ROP_FG(0xCC) |
                        VIVS_DE_ROP_ROP_BG(0xCC));

      if (!need_blend) {
        cmd.EmitState(VIVS_DE_PE_CONFIG,
                      VIVS_DE_PE_CONFIG_DESTINATION_FETCH_DISABLE);
        cmd.EmitState(VIVS_DE_ALPHA_CONTROL,
                      VIVS_DE_ALPHA_CONTROL_ENABLE_OFF);
        cmd.EmitState(VIVS_DE_COLOR_MULTIPLY_MODES, 0);
      } else {
        uint32_t global_src_mode =
            (p.alpha == 0xFF)
                ? VIVS_DE_ALPHA_MODES_GLOBAL_SRC_ALPHA_MODE_NORMAL
                : (p.blending == HWC2::BlendMode::None
                       ? VIVS_DE_ALPHA_MODES_GLOBAL_SRC_ALPHA_MODE_GLOBAL
                       : VIVS_DE_ALPHA_MODES_GLOBAL_SRC_ALPHA_MODE_SCALED);
        uint32_t mul_modes = 0;
        if (p.blending == HWC2::BlendMode::Coverage)
          mul_modes |= VIVS_DE_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_ENABLE;
        if (p.alpha != 0xFF)
          mul_modes |=
              VIVS_DE_COLOR_MULTIPLY_MODES_SRC_GLOBAL_PREMULTIPLY_ALPHA;

        cmd.EmitState(VIVS_DE_PE_CONFIG,
                      VIVS_DE_PE_CONFIG_DESTINATION_FETCH_DEFAULT);
        cmd.EmitState(VIVS_DE_ALPHA_CONTROL,
                      VIVS_DE_ALPHA_CONTROL_ENABLE_ON);
        cmd.EmitState(VIVS_DE_GLOBAL_SRC_COLOR,
                      (static_cast<uint32_t>(p.alpha) << 24) | 0x00FFFFFFu);
        cmd.EmitState(VIVS_DE_GLOBAL_DEST_COLOR, 0xFFFFFFFFu);
        cmd.EmitState(VIVS_DE_COLOR_MULTIPLY_MODES, mul_modes);
        cmd.EmitState(VIVS_DE_ALPHA_MODES,
                      VIVS_DE_ALPHA_MODES_SRC_ALPHA_MODE_NORMAL |
                          VIVS_DE_ALPHA_MODES_DST_ALPHA_MODE_NORMAL |
                          global_src_mode |
                          VIVS_DE_ALPHA_MODES_GLOBAL_DST_ALPHA_MODE_NORMAL |
                          VIVS_DE_ALPHA_MODES_SRC_BLENDING_MODE(
                              DE_BLENDMODE_ONE) |
                          VIVS_DE_ALPHA_MODES_SRC_ALPHA_FACTOR_DISABLE |
                          VIVS_DE_ALPHA_MODES_DST_BLENDING_MODE(
                              DE_BLENDMODE_INVERSED) |
                          VIVS_DE_ALPHA_MODES_DST_ALPHA_FACTOR_DISABLE);
      }

      cmd.EmitDraw2D(static_cast<uint16_t>(draw_l),
                     static_cast<uint16_t>(draw_t),
                     static_cast<uint16_t>(draw_r),
                     static_cast<uint16_t>(draw_b));
    }
  }

  cmd.EmitState(kRegGlFlushCache, kGlFlushCachePe2D);
  cmd.EmitStallPe();

  struct drm_etnaviv_gem_submit req = {};
  req.pipe = 0;
  req.exec_state = ETNA_PIPE_2D;
  req.nr_bos = static_cast<uint32_t>(cmd.bos.size());
  req.nr_relocs = static_cast<uint32_t>(cmd.relocs.size());
  req.stream_size = static_cast<uint32_t>(cmd.words.size() * sizeof(uint32_t));
  req.bos = reinterpret_cast<uintptr_t>(cmd.bos.data());
  req.relocs = reinterpret_cast<uintptr_t>(cmd.relocs.data());
  req.stream = reinterpret_cast<uintptr_t>(cmd.words.data());
  req.flags = ETNA_SUBMIT_FENCE_FD_OUT |
              (merged_in_fence.get() >= 0 ? ETNA_SUBMIT_FENCE_FD_IN : 0);
  req.fence_fd = merged_in_fence.get();

  int submit_ret =
      drmCommandWriteRead(etna_fd_, DRM_ETNAVIV_GEM_SUBMIT, &req, sizeof(req));
  merged_in_fence.Close();
  cleanup_bos();

  if (submit_ret != 0 || req.fence_fd < 0) {
    ALOGE("DRM_ETNA_GEM_SUBMIT failed: ret=%d errno=%d fence_fd=%d",
          submit_ret, errno, req.fence_fd);
    return submit_ret ? -errno : -EIO;
  }
  int submit_out_fence = req.fence_fd;

  hwc_drm_bo_t bo = {};
  bo.width = width;
  bo.height = height;
  bo.format = transparent_bg ? DRM_FORMAT_ABGR8888 : DRM_FORMAT_XBGR8888;
  bo.pitches[0] = slot.pitch;
  bo.gem_handles[0] = slot.gem_handle;
  bo.fb_id = transparent_bg ? slot.fb_id_abgr : slot.fb_id_xbgr;
  bo.acquire_fence_fd = -1;

  out_layer->buffer = DrmHwcBuffer(bo, &GetUnownedImporter());
  out_layer->blending =
      transparent_bg ? DrmHwcBlending::kPreMult : DrmHwcBlending::kNone;
  out_layer->alpha = 0xFF;
  out_layer->transform = DrmHwcTransform::kIdentity;
  out_layer->source_crop =
      DrmHwcRect<float>(0.0f, 0.0f, static_cast<float>(width),
                        static_cast<float>(height));
  out_layer->display_frame =
      DrmHwcRect<int>(0, 0, static_cast<int>(width), static_cast<int>(height));
  out_layer->acquire_fence.Set(dup(submit_out_fence));

  if (out_fence_fd)
    *out_fence_fd = submit_out_fence;
  else
    close(submit_out_fence);

  if (out_slot_index)
    *out_slot_index = slot_idx;

  ++frames_composited_;
  return 0;
}

void Etna2dCompositor::SetSlotBusyFence(size_t slot_index, int fence_fd) {
  if (slot_index >= kNumTargetSlots)
    return;
  /*
   * |fence_fd| is the CRTC out_fence for the commit that puts |slot_index|
   * onto the display, which is also the exact VSYNC instant when the
   * previously displayed slot finishes scanning out.
   */
  if (active_slot_ >= 0 &&
      static_cast<size_t>(active_slot_) < kNumTargetSlots &&
      static_cast<size_t>(active_slot_) != slot_index && fence_fd >= 0) {
    MergeFence(&slots_[active_slot_].busy_fence, fence_fd);
  }
  if (fence_fd >= 0)
    MergeFence(&slots_[slot_index].busy_fence, fence_fd);
  active_slot_ = static_cast<int>(slot_index);
}

void Etna2dCompositor::ReleaseActiveSlot(int fence_fd) {
  if (active_slot_ >= 0 &&
      static_cast<size_t>(active_slot_) < kNumTargetSlots && fence_fd >= 0) {
    MergeFence(&slots_[active_slot_].busy_fence, fence_fd);
  }
  active_slot_ = -1;
}

void Etna2dCompositor::DumpStats(std::string *out) const {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "Etna2d: avail=%d frames=%llu cache_hit=%llu cache_miss=%llu "
           "culled_layers=%llu partial_blits=%llu\n",
           available_ ? 1 : 0,
           static_cast<unsigned long long>(frames_composited_),
           static_cast<unsigned long long>(cache_hits_),
           static_cast<unsigned long long>(cache_misses_),
           static_cast<unsigned long long>(culled_layers_),
           static_cast<unsigned long long>(partial_blits_));
  *out += buf;
}

}  // namespace android
