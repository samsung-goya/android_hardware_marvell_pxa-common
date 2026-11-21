/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 samsung-goya
 */

#ifndef ANDROID_ETNA2D_COMPOSITOR_H_
#define ANDROID_ETNA2D_COMPOSITOR_H_

#include <stdint.h>
#include <string>
#include <vector>

#include <hardware/gralloc.h>
#include <hardware/hwcomposer2.h>

#include "autofd.h"
#include "drmhwcomposer.h"

struct etna_bo;
struct etna_cmd_stream;
struct etna_device;
struct etna_gpu;
struct etna_pipe;

namespace android {

struct Etna2dLayerInput {
  buffer_handle_t handle = nullptr;
  HWC2::BlendMode blending = HWC2::BlendMode::None;
  uint8_t alpha = 0xFF;
  HWC2::Transform transform = HWC2::Transform::None;
  hwc_rect_t display_frame = {0, 0, 0, 0};
  hwc_frect_t source_crop = {0.0f, 0.0f, 0.0f, 0.0f};
  int acquire_fence = -1;
  bool is_solid_color = false;
  hwc_color_t color = {0, 0, 0, 0};
};

class Etna2dCompositor {
 public:
  Etna2dCompositor() = default;
  ~Etna2dCompositor();

  int Init(int drm_fd, const gralloc_module_t *gralloc);
  bool IsAvailable() const {
    return available_;
  }

  bool IsLayerSupported(HWC2::Composition sf_type, buffer_handle_t handle,
                        HWC2::BlendMode blending, HWC2::Transform transform,
                        const hwc_rect_t &display_frame,
                        const hwc_frect_t &source_crop, uint32_t display_w,
                        uint32_t display_h) const;

  // Composites |layers| back-to-front into an internal CMA scanout buffer and
  // populates |out_layer| for presentation on the primary GRA plane.
  int Composite(const std::vector<Etna2dLayerInput> &layers, uint32_t width,
                uint32_t height, bool transparent_bg, DrmHwcLayer *out_layer,
                int *out_fence_fd, size_t *out_slot_index);

  void SetSlotBusyFence(size_t slot_index, int fence_fd);
  void ReleaseActiveSlot(int fence_fd);
  void DumpStats(std::string *out) const;

 private:
  struct TargetSlot {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pitch = 0;
    uint32_t size = 0;
    uint32_t gem_handle = 0;
    uint32_t fb_id_xbgr = 0;
    uint32_t fb_id_abgr = 0;
    struct etna_bo *bo = nullptr;
    UniqueFd busy_fence;
  };

  struct CachedBo {
    uint64_t dev = 0;
    uint64_t ino = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t fourcc = 0;
    uint32_t offset = 0;
    uint32_t stride = 0;
    struct etna_bo *bo = nullptr;
    uint64_t last_used = 0;
  };

  static constexpr size_t kNumTargetSlots = 3;
  static constexpr size_t kMaxCachedBos = 8;
  static constexpr size_t kSolidRingSize = 1024;

  int EnsureTargetSlot(size_t slot_index, uint32_t width, uint32_t height);
  void DestroyTargetSlot(TargetSlot *slot);
  struct etna_bo *GetOrImportSourceBo(int dmabuf_fd, uint32_t buf_w,
                                      uint32_t buf_h, uint32_t fourcc,
                                      uint32_t offset, uint32_t stride,
                                      bool *out_owned);
  void Shutdown();

  int drm_fd_ = -1;
  int etna_fd_ = -1;
  const gralloc_module_t *gralloc_ = nullptr;
  bool available_ = false;

  struct etna_device *etna_dev_ = nullptr;
  struct etna_gpu *etna_gpu_ = nullptr;
  struct etna_pipe *etna_pipe_ = nullptr;
  struct etna_cmd_stream *stream_ = nullptr;
  struct etna_bo *solid_bo_ = nullptr;
  uint32_t *solid_map_ = nullptr;
  size_t solid_cursor_ = 0;

  TargetSlot slots_[kNumTargetSlots];
  size_t next_slot_ = 0;
  int active_slot_ = -1;
  std::vector<CachedBo> bo_cache_;
  uint64_t cache_seq_ = 0;

  mutable uint64_t frames_composited_ = 0;
  mutable uint64_t cache_hits_ = 0;
  mutable uint64_t cache_misses_ = 0;
  mutable uint64_t culled_layers_ = 0;
  mutable uint64_t partial_blits_ = 0;
};

}  // namespace android

#endif  // ANDROID_ETNA2D_COMPOSITOR_H_
