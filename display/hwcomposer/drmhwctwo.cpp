/*
 * Copyright (C) 2016 The Android Open Source Project
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

#define ATRACE_TAG ATRACE_TAG_GRAPHICS
#define LOG_TAG "hwc-drm-two"

#include "drmdisplaycomposition.h"
#include "drmhwcomposer.h"
#include "drmhwctwo.h"
#include "platform.h"
#include "vsyncworker.h"

#include <algorithm>
#include <inttypes.h>
#include <string>
#include <unordered_set>
#include <vector>

#include <cutils/log.h>
#include <cutils/properties.h>
#include <drm/drm_fourcc.h>
#include <hardware/hardware.h>
#include <hardware/hwcomposer2.h>
#include <sync/sync.h>

namespace android {

static bool IsYuvDrmFormat(uint32_t fourcc) {
  switch (fourcc) {
    case DRM_FORMAT_YUV420:
    case DRM_FORMAT_YVU420:
    case DRM_FORMAT_YUV422:
    case DRM_FORMAT_YVU422:
    case DRM_FORMAT_NV12:
    case DRM_FORMAT_NV21:
    case DRM_FORMAT_UYVY:
    case DRM_FORMAT_VYUY:
    case DRM_FORMAT_YUYV:
    case DRM_FORMAT_YVYU:
      return true;
    default:
      return false;
  }
}

static bool IsBufferYuv(const gralloc_module_t *gralloc,
                        buffer_handle_t handle) {
  if (!gralloc || !gralloc->perform || !handle)
    return false;
  struct cros_gralloc_info {
    uint32_t drm_fourcc;
    int num_fds;
    int fds[4];
    uint64_t modifier;
    int offset[4];
    int stride[4];
  } info = {};
  constexpr int kCrosGrallocGetBufferInfo = 4;
  if (gralloc->perform(gralloc, kCrosGrallocGetBufferInfo, handle, &info) != 0)
    return false;
  return IsYuvDrmFormat(info.drm_fourcc);
}

static bool HasAlphaDrmFormat(uint32_t fourcc) {
  switch (fourcc) {
    case DRM_FORMAT_ABGR8888:
    case DRM_FORMAT_ARGB8888:
    case DRM_FORMAT_RGBA8888:
    case DRM_FORMAT_BGRA8888:
    case DRM_FORMAT_ABGR4444:
    case DRM_FORMAT_ARGB4444:
    case DRM_FORMAT_RGBA4444:
    case DRM_FORMAT_BGRA4444:
    case DRM_FORMAT_ABGR1555:
    case DRM_FORMAT_ARGB1555:
    case DRM_FORMAT_RGBA5551:
    case DRM_FORMAT_BGRA5551:
      return true;
    default:
      return false;
  }
}

static bool IsDrmLayerEligibleForMmpPlane(HWC2::Composition sf_type,
                                          const DrmHwcLayer &drm_layer,
                                          bool is_overlay, bool has_underlay) {
  if (sf_type != HWC2::Composition::Device &&
      sf_type != HWC2::Composition::Cursor)
    return false;

  if (!drm_layer.buffer)
    return false;

  // mmp-drm has no hardware rotation/reflection or coverage blend support.
  if (drm_layer.transform != DrmHwcTransform::kIdentity ||
      drm_layer.blending == DrmHwcBlending::kCoverage)
    return false;

  if (drm_layer.display_frame.right <= drm_layer.display_frame.left ||
      drm_layer.display_frame.bottom <= drm_layer.display_frame.top ||
      drm_layer.source_crop.right <= drm_layer.source_crop.left ||
      drm_layer.source_crop.bottom <= drm_layer.source_crop.top)
    return false;

  uint32_t fmt = drm_layer.buffer->format;
  bool is_yuv = IsYuvDrmFormat(fmt);
  bool has_alpha = HasAlphaDrmFormat(fmt);

  if (is_overlay) {
    // PXA986 A0 DMA overlay single-plane packed RGB fetch corrupts x >= ~700
    // on scanlines (visible as a right-edge cutoff/smear in Launcher & Settings);
    // match Marvell's BSP HWOverlayComposer by restricting DMA to YUV layers.
    if (!is_yuv)
      return false;
    if (drm_layer.blending == DrmHwcBlending::kPreMult &&
        drm_layer.alpha != 0xFF)
      return false;
  } else {
    // GRA primary plane does not support YUV formats.
    if (is_yuv)
      return false;
    if (drm_layer.blending == DrmHwcBlending::kPreMult &&
        drm_layer.alpha != 0xFF && has_alpha)
      return false;
    if (has_underlay && has_alpha && drm_layer.blending == DrmHwcBlending::kNone)
      return false;
  }

  return true;
}

class DrmVsyncCallback : public VsyncCallback {
 public:
  DrmVsyncCallback(hwc2_callback_data_t data, hwc2_function_pointer_t hook)
      : data_(data), hook_(hook) {
  }

  void Callback(int display, int64_t timestamp) {
    auto hook = reinterpret_cast<HWC2_PFN_VSYNC>(hook_);
    hook(data_, display, timestamp);
  }

 private:
  hwc2_callback_data_t data_;
  hwc2_function_pointer_t hook_;
};

DrmHwcTwo::DrmHwcTwo() {
  common.tag = HARDWARE_DEVICE_TAG;
  common.version = HWC_DEVICE_API_VERSION_2_0;
  common.close = HookDevClose;
  getCapabilities = HookDevGetCapabilities;
  getFunction = HookDevGetFunction;
}

HWC2::Error DrmHwcTwo::Init() {
  int ret = drm_.Init();
  if (ret) {
    ALOGE("Can't initialize drm object %d", ret);
    return HWC2::Error::NoResources;
  }

  importer_.reset(Importer::CreateInstance(&drm_));
  if (!importer_) {
    ALOGE("Failed to create importer instance");
    return HWC2::Error::NoResources;
  }

  ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID,
                      (const hw_module_t **)&gralloc_);
  if (ret) {
    ALOGE("Failed to open gralloc module %d", ret);
    return HWC2::Error::NoResources;
  }

  displays_.emplace(std::piecewise_construct,
                    std::forward_as_tuple(HWC_DISPLAY_PRIMARY),
                    std::forward_as_tuple(&drm_, importer_, gralloc_,
                                          HWC_DISPLAY_PRIMARY,
                                          HWC2::DisplayType::Physical));

  DrmCrtc *crtc = drm_.GetCrtcForDisplay(static_cast<int>(HWC_DISPLAY_PRIMARY));
  if (!crtc) {
    ALOGE("Failed to get crtc for display %d",
          static_cast<int>(HWC_DISPLAY_PRIMARY));
    return HWC2::Error::BadDisplay;
  }

  std::vector<DrmPlane *> display_planes;
  for (auto &plane : drm_.planes()) {
    if (plane->GetCrtcSupported(*crtc))
      display_planes.push_back(plane.get());
  }
  displays_.at(HWC_DISPLAY_PRIMARY).Init(&display_planes);
  return HWC2::Error::None;
}

template <typename... Args>
static inline HWC2::Error unsupported(char const *func, Args... /*args*/) {
  ALOGV("Unsupported function: %s", func);
  return HWC2::Error::Unsupported;
}

static inline void supported(char const *func) {
  ALOGV("Supported function: %s", func);
}

HWC2::Error DrmHwcTwo::CreateVirtualDisplay(uint32_t width, uint32_t height,
                                            int32_t *format,
                                            hwc2_display_t *display) {
  // TODO: Implement virtual display
  return unsupported(__func__, width, height, format, display);
}

HWC2::Error DrmHwcTwo::DestroyVirtualDisplay(hwc2_display_t display) {
  // TODO: Implement virtual display
  return unsupported(__func__, display);
}

void DrmHwcTwo::Dump(uint32_t *size, char *buffer) {
  supported(__func__);
  std::string out;
  auto it = displays_.find(HWC_DISPLAY_PRIMARY);
  if (it != displays_.end())
    it->second.DumpEtnaStats(&out);

  if (!buffer) {
    *size = static_cast<uint32_t>(out.size());
    return;
  }
  uint32_t copy_len =
      std::min(*size, static_cast<uint32_t>(out.size()));
  if (copy_len > 0)
    memcpy(buffer, out.data(), copy_len);
  *size = copy_len;
}

uint32_t DrmHwcTwo::GetMaxVirtualDisplayCount() {
  // TODO: Implement virtual display
  unsupported(__func__);
  return 0;
}

HWC2::Error DrmHwcTwo::RegisterCallback(int32_t descriptor,
                                        hwc2_callback_data_t data,
                                        hwc2_function_pointer_t function) {
  supported(__func__);
  auto callback = static_cast<HWC2::Callback>(descriptor);
  callbacks_.emplace(callback, HwcCallback(data, function));

  switch (callback) {
    case HWC2::Callback::Hotplug: {
      auto hotplug = reinterpret_cast<HWC2_PFN_HOTPLUG>(function);
      hotplug(data, HWC_DISPLAY_PRIMARY,
              static_cast<int32_t>(HWC2::Connection::Connected));
      break;
    }
    case HWC2::Callback::Vsync: {
      for (std::pair<const hwc2_display_t, DrmHwcTwo::HwcDisplay> &d :
           displays_)
        d.second.RegisterVsyncCallback(data, function);
      break;
    }
    default:
      break;
  }
  return HWC2::Error::None;
}

DrmHwcTwo::HwcDisplay::HwcDisplay(DrmResources *drm,
                                  std::shared_ptr<Importer> importer,
                                  const gralloc_module_t *gralloc,
                                  hwc2_display_t handle, HWC2::DisplayType type)
    : drm_(drm),
      importer_(importer),
      gralloc_(gralloc),
      handle_(handle),
      type_(type) {
  supported(__func__);
}

HWC2::Error DrmHwcTwo::HwcDisplay::Init(std::vector<DrmPlane *> *planes) {
  supported(__func__);
  planner_ = Planner::CreateInstance(drm_);
  if (!planner_) {
    ALOGE("Failed to create planner instance for composition");
    return HWC2::Error::NoResources;
  }

  int display = static_cast<int>(handle_);
  int ret = compositor_.Init(drm_, display);
  if (ret) {
    ALOGE("Failed display compositor init for display %d (%d)", display, ret);
    return HWC2::Error::NoResources;
  }

  if (etna2d_.Init(drm_->fd(), gralloc_) != 0)
    ALOGW("Vivante 2D compositor unavailable on display %d; using Client fallback",
          display);

  // Split up the given display planes into primary and overlay to properly
  // interface with the composition
  char use_overlay_planes_prop[PROPERTY_VALUE_MAX];
  property_get("hwc.drm.use_overlay_planes", use_overlay_planes_prop, "1");
  bool use_overlay_planes = atoi(use_overlay_planes_prop);
  for (auto &plane : *planes) {
    if (plane->type() == DRM_PLANE_TYPE_PRIMARY)
      primary_planes_.push_back(plane);
    else if (use_overlay_planes && (plane)->type() == DRM_PLANE_TYPE_OVERLAY)
      overlay_planes_.push_back(plane);
  }

  crtc_ = drm_->GetCrtcForDisplay(display);
  if (!crtc_) {
    ALOGE("Failed to get crtc for display %d", display);
    return HWC2::Error::BadDisplay;
  }

  connector_ = drm_->GetConnectorForDisplay(display);
  if (!connector_) {
    ALOGE("Failed to get connector for display %d", display);
    return HWC2::Error::BadDisplay;
  }

  // Fetch the number of modes from the display
  uint32_t num_configs;
  HWC2::Error err = GetDisplayConfigs(&num_configs, NULL);
  if (err != HWC2::Error::None || !num_configs)
    return err;

  // Prefer the panel's preferred mode if present, otherwise fall back to first.
  hwc2_config_t default_config;
  num_configs = 1;
  err = GetDisplayConfigs(&num_configs, &default_config);
  if (err != HWC2::Error::None)
    return err;
  for (const DrmMode &mode : connector_->modes()) {
    if (mode.type() & DRM_MODE_TYPE_PREFERRED) {
      default_config = mode.id();
      break;
    }
  }

  ret = vsync_worker_.Init(drm_, display);
  if (ret) {
    ALOGE("Failed to create event worker for d=%d %d\n", display, ret);
    return HWC2::Error::BadDisplay;
  }

  err = SetActiveConfig(default_config);
  if (err != HWC2::Error::None)
    return err;

  return SetPowerMode(static_cast<int32_t>(HWC2::PowerMode::On));
}

HWC2::Error DrmHwcTwo::HwcDisplay::RegisterVsyncCallback(
    hwc2_callback_data_t data, hwc2_function_pointer_t func) {
  supported(__func__);
  auto callback = std::make_shared<DrmVsyncCallback>(data, func);
  vsync_worker_.RegisterCallback(std::move(callback));
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::AcceptDisplayChanges() {
  supported(__func__);
  uint32_t num_changes = 0;
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_)
    l.second.accept_type_change();
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::CreateLayer(hwc2_layer_t *layer) {
  supported(__func__);
  layers_.emplace(static_cast<hwc2_layer_t>(layer_idx_), HwcLayer());
  *layer = static_cast<hwc2_layer_t>(layer_idx_);
  ++layer_idx_;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::DestroyLayer(hwc2_layer_t layer) {
  supported(__func__);
  layers_.erase(layer);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetActiveConfig(hwc2_config_t *config) {
  supported(__func__);
  DrmMode const &mode = connector_->active_mode();
  if (mode.id() == 0)
    return HWC2::Error::BadConfig;

  *config = mode.id();
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetChangedCompositionTypes(
    uint32_t *num_elements, hwc2_layer_t *layers, int32_t *types) {
  supported(__func__);
  uint32_t num_changes = 0;
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    if (l.second.type_changed()) {
      if (layers && num_changes < *num_elements)
        layers[num_changes] = l.first;
      if (types && num_changes < *num_elements)
        types[num_changes] = static_cast<int32_t>(l.second.validated_type());
      ++num_changes;
    }
  }
  if (!layers && !types)
    *num_elements = num_changes;
  else
    *num_elements = std::min(*num_elements, num_changes);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetClientTargetSupport(uint32_t width,
                                                          uint32_t height,
                                                          int32_t /*format*/,
                                                          int32_t dataspace) {
  supported(__func__);
  std::pair<uint32_t, uint32_t> min = drm_->min_resolution();
  std::pair<uint32_t, uint32_t> max = drm_->max_resolution();

  if (width < min.first || height < min.second)
    return HWC2::Error::Unsupported;

  if (width > max.first || height > max.second)
    return HWC2::Error::Unsupported;

  if (dataspace != HAL_DATASPACE_UNKNOWN &&
      dataspace != HAL_DATASPACE_STANDARD_UNSPECIFIED &&
      dataspace != HAL_DATASPACE_V0_SRGB)
    return HWC2::Error::Unsupported;

  // TODO: Validate format can be handled by either GL or planes
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetColorModes(uint32_t *num_modes,
                                                 int32_t *modes) {
  supported(__func__);
  if (!modes)
    *num_modes = 1;

  if (modes)
    *modes = HAL_COLOR_MODE_NATIVE;

  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayAttribute(hwc2_config_t config,
                                                       int32_t attribute_in,
                                                       int32_t *value) {
  supported(__func__);
  auto mode =
      std::find_if(connector_->modes().begin(), connector_->modes().end(),
                   [config](DrmMode const &m) { return m.id() == config; });
  if (mode == connector_->modes().end()) {
    ALOGE("Could not find active mode for %d", config);
    return HWC2::Error::BadConfig;
  }

  static const int32_t kUmPerInch = 25400;
  uint32_t mm_width = connector_->mm_width();
  uint32_t mm_height = connector_->mm_height();
  auto attribute = static_cast<HWC2::Attribute>(attribute_in);
  switch (attribute) {
    case HWC2::Attribute::Width:
      *value = mode->h_display();
      break;
    case HWC2::Attribute::Height:
      *value = mode->v_display();
      break;
    case HWC2::Attribute::VsyncPeriod:
      // in nanoseconds
      *value = 1000 * 1000 * 1000 / mode->v_refresh();
      break;
    case HWC2::Attribute::DpiX:
      // Dots per 1000 inches
      *value = mm_width ? (mode->h_display() * kUmPerInch) / mm_width : -1;
      break;
    case HWC2::Attribute::DpiY:
      // Dots per 1000 inches
      *value = mm_height ? (mode->v_display() * kUmPerInch) / mm_height : -1;
      break;
    default:
      *value = -1;
      return HWC2::Error::BadConfig;
  }
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayConfigs(uint32_t *num_configs,
                                                     hwc2_config_t *configs) {
  supported(__func__);
  // Since this callback is normally invoked twice (once to get the count, and
  // once to populate configs), we don't really want to read the edid
  // redundantly. Instead, only update the modes on the first invocation. While
  // it's possible this will result in stale modes, it'll all come out in the
  // wash when we try to set the active config later.
  if (!configs) {
    int ret = connector_->UpdateModes();
    if (ret) {
      ALOGE("Failed to update display modes %d", ret);
      return HWC2::Error::BadDisplay;
    }
  }

  auto num_modes = static_cast<uint32_t>(connector_->modes().size());
  if (!configs) {
    *num_configs = num_modes;
    return HWC2::Error::None;
  }

  uint32_t idx = 0;
  for (const DrmMode &mode : connector_->modes()) {
    if (idx >= *num_configs)
      break;
    configs[idx++] = mode.id();
  }
  *num_configs = idx;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayName(uint32_t *size, char *name) {
  supported(__func__);
  std::ostringstream stream;
  stream << "display-" << connector_->id();
  std::string string = stream.str();
  size_t length = string.length();
  if (!name) {
    *size = length;
    return HWC2::Error::None;
  }

  *size = std::min<uint32_t>(static_cast<uint32_t>(length), *size);
  strncpy(name, string.c_str(), *size);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayRequests(int32_t *display_requests,
                                                      uint32_t *num_elements,
                                                      hwc2_layer_t *layers,
                                                      int32_t *layer_requests) {
  supported(__func__);
  // TODO: I think virtual display should request
  //      HWC2_DISPLAY_REQUEST_WRITE_CLIENT_TARGET_TO_OUTPUT here
  unsupported(__func__, display_requests, num_elements, layers, layer_requests);
  *num_elements = 0;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayType(int32_t *type) {
  supported(__func__);
  *type = static_cast<int32_t>(type_);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDozeSupport(int32_t *support) {
  supported(__func__);
  *support = 0;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetHdrCapabilities(
    uint32_t *num_types, int32_t */*types*/, float */*max_luminance*/,
    float */*max_average_luminance*/, float */*min_luminance*/) {
  supported(__func__);
  *num_types = 0;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetReleaseFences(uint32_t *num_elements,
                                                    hwc2_layer_t *layers,
                                                    int32_t *fences) {
  supported(__func__);
  uint32_t num_layers = 0;

  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    ++num_layers;
    if (layers == NULL || fences == NULL) {
      continue;
    } else if (num_layers > *num_elements) {
      ALOGW("Overflow num_elements %d/%d", num_layers, *num_elements);
      return HWC2::Error::None;
    }

    layers[num_layers - 1] = l.first;
    fences[num_layers - 1] = l.second.take_release_fence();
  }
  *num_elements = num_layers;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::PresentDisplay(int32_t *retire_fence) {
  supported(__func__);
  std::vector<DrmCompositionDisplayLayersMap> layers_map;
  layers_map.emplace_back();
  DrmCompositionDisplayLayersMap &map = layers_map.back();

  map.display = static_cast<int>(handle_);
  map.geometry_changed = true;  // TODO: Fix this

  // order the layers by z-order
  bool use_client_layer = false;
  uint32_t client_z_order = 0;
  std::map<uint32_t, DrmHwcTwo::HwcLayer *> z_map;
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    switch (l.second.validated_type()) {
      case HWC2::Composition::Device:
      case HWC2::Composition::SolidColor:
        z_map.emplace(std::make_pair(l.second.z_order(), &l.second));
        break;
      case HWC2::Composition::Client:
        // Place it at the z_order of the highest client layer
        use_client_layer = true;
        client_z_order = std::max(client_z_order, l.second.z_order());
        break;
      default:
        continue;
    }
  }
  if (use_client_layer)
    z_map.emplace(std::make_pair(client_z_order, &client_layer_));

  // now that they're ordered by z, add them to the composition
  bool used_etna2d = false;
  int etna_out_fence = -1;
  size_t etna_slot_idx = 0;
  DrmHwcTwo::HwcLayer *dma_hwc_layer = nullptr;

  if (use_etna2d_ && !use_client_layer && !z_map.empty() &&
      etna2d_.IsAvailable() &&
      (!etna2d_has_dma_underlay_ || z_map.size() >= 2)) {
    std::vector<Etna2dLayerInput> etna_inputs;
    for (std::pair<const uint32_t, DrmHwcTwo::HwcLayer *> &l : z_map) {
      if (etna2d_has_dma_underlay_ && !dma_hwc_layer) {
        dma_hwc_layer = l.second;
        DrmHwcLayer layer;
        dma_hwc_layer->PopulateDrmLayer(&layer);
        int ret = layer.ImportBuffer(importer_.get(), gralloc_);
        if (ret) {
          ALOGE("Failed to import DMA underlay layer, ret=%d", ret);
          return HWC2::Error::NoResources;
        }
        map.layers.emplace_back(std::move(layer));
      } else {
        Etna2dLayerInput in;
        l.second->PopulateEtna2dLayer(&in);
        etna_inputs.emplace_back(in);
      }
    }

    DrmHwcLayer gra_layer;
    uint32_t disp_w = connector_->active_mode().h_display();
    uint32_t disp_h = connector_->active_mode().v_display();
    int etna_ret = etna2d_.Composite(etna_inputs, disp_w, disp_h,
                                     etna2d_has_dma_underlay_, &gra_layer,
                                     &etna_out_fence, &etna_slot_idx);
    for (Etna2dLayerInput &in : etna_inputs) {
      if (in.acquire_fence >= 0) {
        close(in.acquire_fence);
        in.acquire_fence = -1;
      }
    }
    if (etna_ret) {
      ALOGE("Etna2d Composite failed ret=%d", etna_ret);
      if (etna_out_fence >= 0)
        close(etna_out_fence);
      return HWC2::Error::BadParameter;
    }
    map.layers.emplace_back(std::move(gra_layer));
    used_etna2d = true;
  } else {
    for (std::pair<const uint32_t, DrmHwcTwo::HwcLayer *> &l : z_map) {
      DrmHwcLayer layer;
      l.second->PopulateDrmLayer(&layer);
      int ret = layer.ImportBuffer(importer_.get(), gralloc_);
      if (ret) {
        ALOGE("Failed to import layer, ret=%d", ret);
        return HWC2::Error::NoResources;
      }
      map.layers.emplace_back(std::move(layer));
    }
  }
  if (map.layers.empty()) {
    if (etna_out_fence >= 0)
      close(etna_out_fence);
    *retire_fence = -1;
    return HWC2::Error::None;
  }

  std::unique_ptr<DrmDisplayComposition> composition =
      compositor_.CreateComposition();
  composition->Init(drm_, crtc_, importer_.get(), planner_.get(), frame_no_);

  int ret = composition->SetLayers(map.layers.data(), map.layers.size(), true);
  if (ret) {
    ALOGE("Failed to set layers in the composition ret=%d", ret);
    if (etna_out_fence >= 0)
      close(etna_out_fence);
    return HWC2::Error::BadLayer;
  }

  std::vector<DrmPlane *> primary_planes(primary_planes_);
  std::vector<DrmPlane *> overlay_planes(overlay_planes_);
  ret = composition->Plan(compositor_.squash_state(), &primary_planes,
                          &overlay_planes);
  if (ret) {
    ALOGE("Failed to plan the composition ret=%d", ret);
    if (etna_out_fence >= 0)
      close(etna_out_fence);
    return HWC2::Error::BadConfig;
  }

  // Disable the planes we're not using
  for (auto i = primary_planes.begin(); i != primary_planes.end();) {
    composition->AddPlaneDisable(*i);
    i = primary_planes.erase(i);
  }
  for (auto i = overlay_planes.begin(); i != overlay_planes.end();) {
    composition->AddPlaneDisable(*i);
    i = overlay_planes.erase(i);
  }

  int out_fence = -1;
  ret = compositor_.ExecuteComposition(std::move(composition), &out_fence);
  if (ret) {
    ALOGE("Failed to apply the frame composition ret=%d", ret);
    if (out_fence >= 0)
      close(out_fence);
    if (etna_out_fence >= 0)
      close(etna_out_fence);
    return HWC2::Error::BadParameter;
  }

  if (used_etna2d)
    etna2d_.SetSlotBusyFence(etna_slot_idx, out_fence);
  else
    etna2d_.ReleaseActiveSlot(out_fence);

  int merged_etna_release_fence = -1;
  if (used_etna2d) {
    if (etna_out_fence >= 0 && out_fence >= 0) {
      merged_etna_release_fence =
          sync_merge("hwc_etna_rel", etna_out_fence, out_fence);
      if (merged_etna_release_fence < 0)
        merged_etna_release_fence = dup(out_fence);
    } else if (out_fence >= 0) {
      merged_etna_release_fence = dup(out_fence);
    } else if (etna_out_fence >= 0) {
      merged_etna_release_fence = dup(etna_out_fence);
    }
  }

  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    bool is_device = (l.second.validated_type() == HWC2::Composition::Device);
    if (used_etna2d && is_device) {
      int fence = (&l.second == dma_hwc_layer) ? out_fence
                                               : merged_etna_release_fence;
      l.second.set_release_fence(fence >= 0 ? dup(fence) : -1);
    } else if (out_fence >= 0 && (is_device || l.second.was_device())) {
      l.second.set_release_fence(dup(out_fence));
    } else {
      l.second.set_release_fence(-1);
    }
    l.second.set_was_device(is_device);
  }
  if (merged_etna_release_fence >= 0)
    close(merged_etna_release_fence);
  if (etna_out_fence >= 0)
    close(etna_out_fence);
  client_layer_.set_release_fence(-1);

  *retire_fence = out_fence;

  ++frame_no_;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetActiveConfig(hwc2_config_t config) {
  supported(__func__);
  auto mode =
      std::find_if(connector_->modes().begin(), connector_->modes().end(),
                   [config](DrmMode const &m) { return m.id() == config; });
  if (mode == connector_->modes().end()) {
    ALOGE("Could not find active mode for %d", config);
    return HWC2::Error::BadConfig;
  }

  int ret = compositor_.SetDisplayMode(*mode);
  if (ret) {
    ALOGE("Failed to set display mode %d", ret);
    return HWC2::Error::BadConfig;
  }
  if (connector_->active_mode().id() == 0)
    connector_->set_active_mode(*mode);

  // Setup the client layer's dimensions
  hwc_rect_t display_frame = {.left = 0,
                              .top = 0,
                              .right = static_cast<int>(mode->h_display()),
                              .bottom = static_cast<int>(mode->v_display())};
  client_layer_.SetLayerDisplayFrame(display_frame);
  hwc_frect_t source_crop = {.left = 0.0f,
                             .top = 0.0f,
                             .right = mode->h_display() + 0.0f,
                             .bottom = mode->v_display() + 0.0f};
  client_layer_.SetLayerSourceCrop(source_crop);

  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetClientTarget(buffer_handle_t target,
                                                   int32_t acquire_fence,
                                                   int32_t dataspace,
                                                   hwc_region_t damage) {
  supported(__func__);
  UniqueFd uf(acquire_fence);

  client_layer_.set_buffer(target);
  client_layer_.set_acquire_fence(uf.get());
  client_layer_.SetLayerDataspace(dataspace);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetColorMode(int32_t mode) {
  supported(__func__);

  if (mode != HAL_COLOR_MODE_NATIVE)
    return HWC2::Error::Unsupported;

  color_mode_ = mode;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetColorTransform(const float *matrix,
                                                     int32_t hint) {
  supported(__func__);
  // TODO: Force client composition if we get this
  return unsupported(__func__, matrix, hint);
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetOutputBuffer(buffer_handle_t buffer,
                                                   int32_t release_fence) {
  supported(__func__);
  // TODO: Need virtual display support
  return unsupported(__func__, buffer, release_fence);
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetPowerMode(int32_t mode_in) {
  supported(__func__);
  uint64_t dpms_value = 0;
  auto mode = static_cast<HWC2::PowerMode>(mode_in);
  switch (mode) {
    case HWC2::PowerMode::Off:
      dpms_value = DRM_MODE_DPMS_OFF;
      break;
    case HWC2::PowerMode::On:
      dpms_value = DRM_MODE_DPMS_ON;
      break;
    default:
      ALOGI("Power mode %d is unsupported\n", mode);
      return HWC2::Error::Unsupported;
  };

  int ret = compositor_.SetDpmsMode(dpms_value);
  if (ret) {
    ALOGE("Failed to apply the dpms composition ret=%d", ret);
    return HWC2::Error::BadParameter;
  }
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetVsyncEnabled(int32_t enabled) {
  supported(__func__);
  vsync_worker_.VSyncControl(enabled);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::ValidateDisplay(uint32_t *num_types,
                                                   uint32_t *num_requests) {
  supported(__func__);
  *num_types = 0;
  *num_requests = 0;

  use_etna2d_ = false;
  etna2d_has_dma_underlay_ = false;

  if (layers_.empty())
    return HWC2::Error::None;

  std::vector<std::pair<hwc2_layer_t, HwcLayer *>> ordered_layers;
  ordered_layers.reserve(layers_.size());
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_)
    ordered_layers.emplace_back(l.first, &l.second);

  std::sort(ordered_layers.begin(), ordered_layers.end(),
            [](const std::pair<hwc2_layer_t, HwcLayer *> &a,
               const std::pair<hwc2_layer_t, HwcLayer *> &b) {
              return a.second->z_order() < b.second->z_order();
            });

  auto try_layers = [&](const std::vector<HwcLayer *> &hw_layers,
                        bool overlay_only) -> bool {
    if (hw_layers.empty())
      return false;

    std::vector<DrmHwcLayer> drm_layers(hw_layers.size());
    for (size_t i = 0; i < hw_layers.size(); ++i) {
      if (!hw_layers[i]->buffer()) {
        ALOGD("ValidateDisplay: layer %zu has null buffer", i);
        return false;
      }
      if (hw_layers[i]->transform() != HWC2::Transform::None ||
          hw_layers[i]->blending() == HWC2::BlendMode::Coverage)
        return false;
      hw_layers[i]->PopulateDrmLayerInfo(&drm_layers[i]);
      if (drm_layers[i].ImportBuffer(importer_.get(), gralloc_) != 0) {
        ALOGD("ValidateDisplay: layer %zu ImportBuffer failed", i);
        return false;
      }

      bool is_overlay = overlay_only;
      bool has_underlay = false;
      if (!overlay_only) {
        if (hw_layers.size() == 2) {
          is_overlay = (i == 0);
          has_underlay = (i == 1);
        } else {
          is_overlay = IsYuvDrmFormat(drm_layers[i].buffer->format);
        }
      }

      if (!IsDrmLayerEligibleForMmpPlane(hw_layers[i]->sf_type(), drm_layers[i],
                                         is_overlay, has_underlay)) {
        ALOGV("ValidateDisplay: layer %zu ineligible (overlay=%d fmt=0x%x "
              "blend=%d alpha=%u tr=%d)",
              i, is_overlay, drm_layers[i].buffer->format,
              static_cast<int>(drm_layers[i].blending), drm_layers[i].alpha,
              static_cast<int>(drm_layers[i].transform));
        return false;
      }
    }

    std::unique_ptr<DrmDisplayComposition> test_comp =
        compositor_.CreateComposition();
    if (test_comp->Init(drm_, crtc_, importer_.get(), planner_.get(),
                        frame_no_) != 0)
      return false;

    if (test_comp->SetLayers(drm_layers.data(), drm_layers.size(), true) != 0)
      return false;

    std::vector<DrmPlane *> primaries;
    if (!overlay_only)
      primaries = primary_planes_;
    std::vector<DrmPlane *> overlays(overlay_planes_);

    if (test_comp->Plan(compositor_.squash_state(), &primaries, &overlays) != 0) {
      ALOGD("ValidateDisplay: Plan failed");
      return false;
    }

    if (overlay_only) {
      for (DrmPlane *plane : primary_planes_)
        test_comp->AddPlaneDisable(plane);
    } else {
      for (DrmPlane *plane : primaries)
        test_comp->AddPlaneDisable(plane);
    }
    for (DrmPlane *plane : overlays)
      test_comp->AddPlaneDisable(plane);

    return compositor_.TestComposition(test_comp.get()) == 0;
  };

  uint32_t disp_w = connector_->active_mode().h_display();
  uint32_t disp_h = connector_->active_mode().v_display();
  auto is_etna2d_eligible = [&](const HwcLayer *l) -> bool {
    return etna2d_.IsAvailable() && disp_w > 0 && disp_h > 0 &&
           etna2d_.IsLayerSupported(l->sf_type(), l->buffer(), l->blending(),
                                    l->transform(), l->display_frame(),
                                    l->source_crop(), disp_w, disp_h);
  };
  auto all_etna2d_eligible = [&](size_t start_idx) -> bool {
    if (ordered_layers.size() <= start_idx || !etna2d_.IsAvailable() ||
        disp_w == 0 || disp_h == 0)
      return false;
    for (size_t i = start_idx; i < ordered_layers.size(); ++i) {
      if (!is_etna2d_eligible(ordered_layers[i].second))
        return false;
    }
    return true;
  };

  std::unordered_set<hwc2_layer_t> device_layers;
  if (ordered_layers.size() == 1) {
    if (try_layers({ordered_layers[0].second}, false)) {
      device_layers.insert(ordered_layers[0].first);
    } else if (is_etna2d_eligible(ordered_layers[0].second)) {
      device_layers.insert(ordered_layers[0].first);
      use_etna2d_ = true;
    }
  } else if (ordered_layers.size() == 2) {
    bool first_is_yuv = IsBufferYuv(gralloc_, ordered_layers[0].second->buffer());
    if (first_is_yuv && !overlay_planes_.empty() && !primary_planes_.empty() &&
        try_layers({ordered_layers[0].second, ordered_layers[1].second},
                   false)) {
      device_layers.insert(ordered_layers[0].first);
      device_layers.insert(ordered_layers[1].first);
    } else if (all_etna2d_eligible(0)) {
      device_layers.insert(ordered_layers[0].first);
      device_layers.insert(ordered_layers[1].first);
      use_etna2d_ = true;
    } else if (first_is_yuv && !overlay_planes_.empty() &&
               !primary_planes_.empty() &&
               ordered_layers[0].second->z_order() <
                   ordered_layers[1].second->z_order() &&
               try_layers({ordered_layers[0].second}, true)) {
      device_layers.insert(ordered_layers[0].first);
    }
  } else if (ordered_layers.size() >= 3) {
    bool first_is_yuv = IsBufferYuv(gralloc_, ordered_layers[0].second->buffer());
    if (first_is_yuv && !overlay_planes_.empty() && !primary_planes_.empty() &&
        ordered_layers[0].second->z_order() <
            ordered_layers[1].second->z_order() &&
        try_layers({ordered_layers[0].second}, true)) {
      device_layers.insert(ordered_layers[0].first);
      if (all_etna2d_eligible(1)) {
        for (size_t i = 1; i < ordered_layers.size(); ++i)
          device_layers.insert(ordered_layers[i].first);
        use_etna2d_ = true;
        etna2d_has_dma_underlay_ = true;
      }
    } else if (all_etna2d_eligible(0)) {
      for (size_t i = 0; i < ordered_layers.size(); ++i)
        device_layers.insert(ordered_layers[i].first);
      use_etna2d_ = true;
    }
  }

  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    DrmHwcTwo::HwcLayer &layer = l.second;
    HWC2::Composition target = HWC2::Composition::Client;
    if (device_layers.count(l.first)) {
      target = (layer.sf_type() == HWC2::Composition::SolidColor)
                   ? HWC2::Composition::SolidColor
                   : HWC2::Composition::Device;
    }
    layer.set_validated_type(target);
    if (layer.validated_type() != layer.sf_type())
      ++*num_types;
  }
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetCursorPosition(int32_t x, int32_t y) {
  supported(__func__);
  cursor_x_ = x;
  cursor_y_ = y;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerBlendMode(int32_t mode) {
  supported(__func__);
  blending_ = static_cast<HWC2::BlendMode>(mode);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerBuffer(buffer_handle_t buffer,
                                                int32_t acquire_fence) {
  supported(__func__);
  UniqueFd uf(acquire_fence);

  // The buffer and acquire_fence are handled elsewhere
  if (sf_type_ == HWC2::Composition::Client ||
      sf_type_ == HWC2::Composition::Sideband ||
      sf_type_ == HWC2::Composition::SolidColor)
    return HWC2::Error::None;

  set_buffer(buffer);
  set_acquire_fence(uf.get());
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerColor(hwc_color_t color) {
  supported(__func__);
  color_ = color;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerCompositionType(int32_t type) {
  sf_type_ = static_cast<HWC2::Composition>(type);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerDataspace(int32_t dataspace) {
  supported(__func__);
  dataspace_ = static_cast<android_dataspace_t>(dataspace);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerDisplayFrame(hwc_rect_t frame) {
  supported(__func__);
  display_frame_ = frame;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerPlaneAlpha(float alpha) {
  supported(__func__);
  alpha_ = alpha;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerSidebandStream(
    const native_handle_t *stream) {
  supported(__func__);
  // TODO: We don't support sideband
  return unsupported(__func__, stream);
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerSourceCrop(hwc_frect_t crop) {
  supported(__func__);
  source_crop_ = crop;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerSurfaceDamage(hwc_region_t damage) {
  supported(__func__);
  // TODO: We don't use surface damage, marking as unsupported
  unsupported(__func__, damage);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerTransform(int32_t transform) {
  supported(__func__);
  transform_ = static_cast<HWC2::Transform>(transform);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerVisibleRegion(hwc_region_t visible) {
  supported(__func__);
  // TODO: We don't use this information, marking as unsupported
  unsupported(__func__, visible);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerZOrder(uint32_t order) {
  supported(__func__);
  z_order_ = order;
  return HWC2::Error::None;
}

void DrmHwcTwo::HwcLayer::PopulateDrmLayerInfo(DrmHwcLayer *layer) const {
  switch (blending_) {
    case HWC2::BlendMode::None:
      layer->blending = DrmHwcBlending::kNone;
      break;
    case HWC2::BlendMode::Premultiplied:
      layer->blending = DrmHwcBlending::kPreMult;
      break;
    case HWC2::BlendMode::Coverage:
      layer->blending = DrmHwcBlending::kCoverage;
      break;
    default:
      ALOGE("Unknown blending mode b=%d", blending_);
      layer->blending = DrmHwcBlending::kNone;
      break;
  }

  layer->sf_handle = buffer_;
  layer->SetDisplayFrame(display_frame_);
  layer->alpha = static_cast<uint8_t>(255.0f * alpha_ + 0.5f);
  layer->SetSourceCrop(source_crop_);
  layer->SetTransform(static_cast<int32_t>(transform_));
}

void DrmHwcTwo::HwcLayer::PopulateDrmLayer(DrmHwcLayer *layer) {
  supported(__func__);
  PopulateDrmLayerInfo(layer);
  OutputFd release_fence = release_fence_output();
  layer->acquire_fence = acquire_fence_.Release();
  layer->release_fence = std::move(release_fence);
}

void DrmHwcTwo::HwcDisplay::DumpEtnaStats(std::string *out) const {
  const char *mode_str = use_etna2d_
                             ? (etna2d_has_dma_underlay_ ? "etna2d+dma"
                                                         : "etna2d")
                             : "zero-copy/client";
  *out += "DrmHwcTwo display 0 mode: ";
  *out += mode_str;
  *out += "\n";
  etna2d_.DumpStats(out);
}

void DrmHwcTwo::HwcLayer::PopulateEtna2dLayer(Etna2dLayerInput *layer) {
  supported(__func__);
  layer->handle = buffer_;
  layer->blending = blending_;
  layer->alpha = static_cast<uint8_t>(255.0f * alpha_ + 0.5f);
  layer->transform = transform_;
  layer->display_frame = display_frame_;
  layer->source_crop = source_crop_;
  layer->acquire_fence = acquire_fence_.Release();
  layer->is_solid_color = (sf_type_ == HWC2::Composition::SolidColor);
  layer->color = color_;
}

// static
int DrmHwcTwo::HookDevClose(hw_device_t * /*dev*/) {
  unsupported(__func__);
  return 0;
}

// static
void DrmHwcTwo::HookDevGetCapabilities(hwc2_device_t * /*dev*/,
                                       uint32_t *out_count,
                                       int32_t * /*out_capabilities*/) {
  supported(__func__);
  *out_count = 0;
}

// static
hwc2_function_pointer_t DrmHwcTwo::HookDevGetFunction(
    struct hwc2_device * /*dev*/, int32_t descriptor) {
  supported(__func__);
  auto func = static_cast<HWC2::FunctionDescriptor>(descriptor);
  switch (func) {
    // Device functions
    case HWC2::FunctionDescriptor::CreateVirtualDisplay:
      return ToHook<HWC2_PFN_CREATE_VIRTUAL_DISPLAY>(
          DeviceHook<int32_t, decltype(&DrmHwcTwo::CreateVirtualDisplay),
                     &DrmHwcTwo::CreateVirtualDisplay, uint32_t, uint32_t,
                     int32_t*, hwc2_display_t *>);
    case HWC2::FunctionDescriptor::DestroyVirtualDisplay:
      return ToHook<HWC2_PFN_DESTROY_VIRTUAL_DISPLAY>(
          DeviceHook<int32_t, decltype(&DrmHwcTwo::DestroyVirtualDisplay),
                     &DrmHwcTwo::DestroyVirtualDisplay, hwc2_display_t>);
    case HWC2::FunctionDescriptor::Dump:
      return ToHook<HWC2_PFN_DUMP>(
          DeviceHook<void, decltype(&DrmHwcTwo::Dump), &DrmHwcTwo::Dump,
                     uint32_t *, char *>);
    case HWC2::FunctionDescriptor::GetMaxVirtualDisplayCount:
      return ToHook<HWC2_PFN_GET_MAX_VIRTUAL_DISPLAY_COUNT>(
          DeviceHook<uint32_t, decltype(&DrmHwcTwo::GetMaxVirtualDisplayCount),
                     &DrmHwcTwo::GetMaxVirtualDisplayCount>);
    case HWC2::FunctionDescriptor::RegisterCallback:
      return ToHook<HWC2_PFN_REGISTER_CALLBACK>(
          DeviceHook<int32_t, decltype(&DrmHwcTwo::RegisterCallback),
                     &DrmHwcTwo::RegisterCallback, int32_t,
                     hwc2_callback_data_t, hwc2_function_pointer_t>);

    // Display functions
    case HWC2::FunctionDescriptor::AcceptDisplayChanges:
      return ToHook<HWC2_PFN_ACCEPT_DISPLAY_CHANGES>(
          DisplayHook<decltype(&HwcDisplay::AcceptDisplayChanges),
                      &HwcDisplay::AcceptDisplayChanges>);
    case HWC2::FunctionDescriptor::CreateLayer:
      return ToHook<HWC2_PFN_CREATE_LAYER>(
          DisplayHook<decltype(&HwcDisplay::CreateLayer),
                      &HwcDisplay::CreateLayer, hwc2_layer_t *>);
    case HWC2::FunctionDescriptor::DestroyLayer:
      return ToHook<HWC2_PFN_DESTROY_LAYER>(
          DisplayHook<decltype(&HwcDisplay::DestroyLayer),
                      &HwcDisplay::DestroyLayer, hwc2_layer_t>);
    case HWC2::FunctionDescriptor::GetActiveConfig:
      return ToHook<HWC2_PFN_GET_ACTIVE_CONFIG>(
          DisplayHook<decltype(&HwcDisplay::GetActiveConfig),
                      &HwcDisplay::GetActiveConfig, hwc2_config_t *>);
    case HWC2::FunctionDescriptor::GetChangedCompositionTypes:
      return ToHook<HWC2_PFN_GET_CHANGED_COMPOSITION_TYPES>(
          DisplayHook<decltype(&HwcDisplay::GetChangedCompositionTypes),
                      &HwcDisplay::GetChangedCompositionTypes, uint32_t *,
                      hwc2_layer_t *, int32_t *>);
    case HWC2::FunctionDescriptor::GetClientTargetSupport:
      return ToHook<HWC2_PFN_GET_CLIENT_TARGET_SUPPORT>(
          DisplayHook<decltype(&HwcDisplay::GetClientTargetSupport),
                      &HwcDisplay::GetClientTargetSupport, uint32_t, uint32_t,
                      int32_t, int32_t>);
    case HWC2::FunctionDescriptor::GetColorModes:
      return ToHook<HWC2_PFN_GET_COLOR_MODES>(
          DisplayHook<decltype(&HwcDisplay::GetColorModes),
                      &HwcDisplay::GetColorModes, uint32_t *, int32_t *>);
    case HWC2::FunctionDescriptor::GetDisplayAttribute:
      return ToHook<HWC2_PFN_GET_DISPLAY_ATTRIBUTE>(DisplayHook<
          decltype(&HwcDisplay::GetDisplayAttribute),
          &HwcDisplay::GetDisplayAttribute, hwc2_config_t, int32_t, int32_t *>);
    case HWC2::FunctionDescriptor::GetDisplayConfigs:
      return ToHook<HWC2_PFN_GET_DISPLAY_CONFIGS>(DisplayHook<
          decltype(&HwcDisplay::GetDisplayConfigs),
          &HwcDisplay::GetDisplayConfigs, uint32_t *, hwc2_config_t *>);
    case HWC2::FunctionDescriptor::GetDisplayName:
      return ToHook<HWC2_PFN_GET_DISPLAY_NAME>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayName),
                      &HwcDisplay::GetDisplayName, uint32_t *, char *>);
    case HWC2::FunctionDescriptor::GetDisplayRequests:
      return ToHook<HWC2_PFN_GET_DISPLAY_REQUESTS>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayRequests),
                      &HwcDisplay::GetDisplayRequests, int32_t *, uint32_t *,
                      hwc2_layer_t *, int32_t *>);
    case HWC2::FunctionDescriptor::GetDisplayType:
      return ToHook<HWC2_PFN_GET_DISPLAY_TYPE>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayType),
                      &HwcDisplay::GetDisplayType, int32_t *>);
    case HWC2::FunctionDescriptor::GetDozeSupport:
      return ToHook<HWC2_PFN_GET_DOZE_SUPPORT>(
          DisplayHook<decltype(&HwcDisplay::GetDozeSupport),
                      &HwcDisplay::GetDozeSupport, int32_t *>);
    case HWC2::FunctionDescriptor::GetHdrCapabilities:
      return ToHook<HWC2_PFN_GET_HDR_CAPABILITIES>(
          DisplayHook<decltype(&HwcDisplay::GetHdrCapabilities),
                      &HwcDisplay::GetHdrCapabilities, uint32_t *, int32_t *,
                      float *, float *, float *>);
    case HWC2::FunctionDescriptor::GetReleaseFences:
      return ToHook<HWC2_PFN_GET_RELEASE_FENCES>(
          DisplayHook<decltype(&HwcDisplay::GetReleaseFences),
                      &HwcDisplay::GetReleaseFences, uint32_t *, hwc2_layer_t *,
                      int32_t *>);
    case HWC2::FunctionDescriptor::PresentDisplay:
      return ToHook<HWC2_PFN_PRESENT_DISPLAY>(
          DisplayHook<decltype(&HwcDisplay::PresentDisplay),
                      &HwcDisplay::PresentDisplay, int32_t *>);
    case HWC2::FunctionDescriptor::SetActiveConfig:
      return ToHook<HWC2_PFN_SET_ACTIVE_CONFIG>(
          DisplayHook<decltype(&HwcDisplay::SetActiveConfig),
                      &HwcDisplay::SetActiveConfig, hwc2_config_t>);
    case HWC2::FunctionDescriptor::SetClientTarget:
      return ToHook<HWC2_PFN_SET_CLIENT_TARGET>(DisplayHook<
          decltype(&HwcDisplay::SetClientTarget), &HwcDisplay::SetClientTarget,
          buffer_handle_t, int32_t, int32_t, hwc_region_t>);
    case HWC2::FunctionDescriptor::SetColorMode:
      return ToHook<HWC2_PFN_SET_COLOR_MODE>(
          DisplayHook<decltype(&HwcDisplay::SetColorMode),
                      &HwcDisplay::SetColorMode, int32_t>);
    case HWC2::FunctionDescriptor::SetColorTransform:
      return ToHook<HWC2_PFN_SET_COLOR_TRANSFORM>(
          DisplayHook<decltype(&HwcDisplay::SetColorTransform),
                      &HwcDisplay::SetColorTransform, const float *, int32_t>);
    case HWC2::FunctionDescriptor::SetOutputBuffer:
      return ToHook<HWC2_PFN_SET_OUTPUT_BUFFER>(
          DisplayHook<decltype(&HwcDisplay::SetOutputBuffer),
                      &HwcDisplay::SetOutputBuffer, buffer_handle_t, int32_t>);
    case HWC2::FunctionDescriptor::SetPowerMode:
      return ToHook<HWC2_PFN_SET_POWER_MODE>(
          DisplayHook<decltype(&HwcDisplay::SetPowerMode),
                      &HwcDisplay::SetPowerMode, int32_t>);
    case HWC2::FunctionDescriptor::SetVsyncEnabled:
      return ToHook<HWC2_PFN_SET_VSYNC_ENABLED>(
          DisplayHook<decltype(&HwcDisplay::SetVsyncEnabled),
                      &HwcDisplay::SetVsyncEnabled, int32_t>);
    case HWC2::FunctionDescriptor::ValidateDisplay:
      return ToHook<HWC2_PFN_VALIDATE_DISPLAY>(
          DisplayHook<decltype(&HwcDisplay::ValidateDisplay),
                      &HwcDisplay::ValidateDisplay, uint32_t *, uint32_t *>);

    // Layer functions
    case HWC2::FunctionDescriptor::SetCursorPosition:
      return ToHook<HWC2_PFN_SET_CURSOR_POSITION>(
          LayerHook<decltype(&HwcLayer::SetCursorPosition),
                    &HwcLayer::SetCursorPosition, int32_t, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerBlendMode:
      return ToHook<HWC2_PFN_SET_LAYER_BLEND_MODE>(
          LayerHook<decltype(&HwcLayer::SetLayerBlendMode),
                    &HwcLayer::SetLayerBlendMode, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerBuffer:
      return ToHook<HWC2_PFN_SET_LAYER_BUFFER>(
          LayerHook<decltype(&HwcLayer::SetLayerBuffer),
                    &HwcLayer::SetLayerBuffer, buffer_handle_t, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerColor:
      return ToHook<HWC2_PFN_SET_LAYER_COLOR>(
          LayerHook<decltype(&HwcLayer::SetLayerColor),
                    &HwcLayer::SetLayerColor, hwc_color_t>);
    case HWC2::FunctionDescriptor::SetLayerCompositionType:
      return ToHook<HWC2_PFN_SET_LAYER_COMPOSITION_TYPE>(
          LayerHook<decltype(&HwcLayer::SetLayerCompositionType),
                    &HwcLayer::SetLayerCompositionType, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerDataspace:
      return ToHook<HWC2_PFN_SET_LAYER_DATASPACE>(
          LayerHook<decltype(&HwcLayer::SetLayerDataspace),
                    &HwcLayer::SetLayerDataspace, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerDisplayFrame:
      return ToHook<HWC2_PFN_SET_LAYER_DISPLAY_FRAME>(
          LayerHook<decltype(&HwcLayer::SetLayerDisplayFrame),
                    &HwcLayer::SetLayerDisplayFrame, hwc_rect_t>);
    case HWC2::FunctionDescriptor::SetLayerPlaneAlpha:
      return ToHook<HWC2_PFN_SET_LAYER_PLANE_ALPHA>(
          LayerHook<decltype(&HwcLayer::SetLayerPlaneAlpha),
                    &HwcLayer::SetLayerPlaneAlpha, float>);
    case HWC2::FunctionDescriptor::SetLayerSidebandStream:
      return ToHook<HWC2_PFN_SET_LAYER_SIDEBAND_STREAM>(LayerHook<
          decltype(&HwcLayer::SetLayerSidebandStream),
          &HwcLayer::SetLayerSidebandStream, const native_handle_t *>);
    case HWC2::FunctionDescriptor::SetLayerSourceCrop:
      return ToHook<HWC2_PFN_SET_LAYER_SOURCE_CROP>(
          LayerHook<decltype(&HwcLayer::SetLayerSourceCrop),
                    &HwcLayer::SetLayerSourceCrop, hwc_frect_t>);
    case HWC2::FunctionDescriptor::SetLayerSurfaceDamage:
      return ToHook<HWC2_PFN_SET_LAYER_SURFACE_DAMAGE>(
          LayerHook<decltype(&HwcLayer::SetLayerSurfaceDamage),
                    &HwcLayer::SetLayerSurfaceDamage, hwc_region_t>);
    case HWC2::FunctionDescriptor::SetLayerTransform:
      return ToHook<HWC2_PFN_SET_LAYER_TRANSFORM>(
          LayerHook<decltype(&HwcLayer::SetLayerTransform),
                    &HwcLayer::SetLayerTransform, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerVisibleRegion:
      return ToHook<HWC2_PFN_SET_LAYER_VISIBLE_REGION>(
          LayerHook<decltype(&HwcLayer::SetLayerVisibleRegion),
                    &HwcLayer::SetLayerVisibleRegion, hwc_region_t>);
    case HWC2::FunctionDescriptor::SetLayerZOrder:
      return ToHook<HWC2_PFN_SET_LAYER_Z_ORDER>(
          LayerHook<decltype(&HwcLayer::SetLayerZOrder),
                    &HwcLayer::SetLayerZOrder, uint32_t>);
    case HWC2::FunctionDescriptor::Invalid:
    default:
      return NULL;
  }
}

// static
int DrmHwcTwo::HookDevOpen(const struct hw_module_t *module, const char *name,
                           struct hw_device_t **dev) {
  supported(__func__);
  if (strcmp(name, HWC_HARDWARE_COMPOSER)) {
    ALOGE("Invalid module name- %s", name);
    return -EINVAL;
  }

  std::unique_ptr<DrmHwcTwo> ctx(new DrmHwcTwo());
  if (!ctx) {
    ALOGE("Failed to allocate DrmHwcTwo");
    return -ENOMEM;
  }

  HWC2::Error err = ctx->Init();
  if (err != HWC2::Error::None) {
    ALOGE("Failed to initialize DrmHwcTwo err=%d\n", err);
    return -EINVAL;
  }

  ctx->common.module = const_cast<hw_module_t *>(module);
  *dev = &ctx->common;
  ctx.release();
  return 0;
}
}

static struct hw_module_methods_t hwc2_module_methods = {
    .open = android::DrmHwcTwo::HookDevOpen,
};

hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .module_api_version = HARDWARE_MODULE_API_VERSION(2, 0),
    .id = HWC_HARDWARE_MODULE_ID,
    .name = "DrmHwcTwo module",
    .author = "The Android Open Source Project",
    .methods = &hwc2_module_methods,
    .dso = NULL,
    .reserved = {0},
};
