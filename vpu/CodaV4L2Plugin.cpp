// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * Stateful V4L2 Memory-to-Memory OpenMAX IL Hardware Plugin (libstagefrighthw)
 * for Marvell PXA988 / PXA986 Chips&Media CODA7542 VPU (Linux 4.19 coda.ko).
 *
 * Implements all hardware quirks documented in kernel/samsung/goyawifi/docs/VPU.md:
 *  1. Dynamic V4L2 node discovery by driver=="coda" and card=="coda-decoder"/"coda-encoder".
 *  2. Macroblock-aligned CAPTURE buffers + visible crop via VIDIOC_G_SELECTION and
 *     row-by-row stride-aware copy (bytesperline != width on non-16-aligned widths).
 *  3. Decoder CAPTURE DPB buffer count >= 12 to prevent H.264 DPB reordering stalls.
 *  4. Encoder OUTPUT queue configured as V4L2_PIX_FMT_NV12 (interleaved UV stride == luma
 *     bytesperline, avoiding YU12 chroma stride unalignment on non-32-aligned widths).
 *  5. Encoder GOP size (60) and bitrate floor (512 kbps) to avoid CODA7542 CBR skip-frames.
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "CodaV4L2Plugin"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#include <cutils/native_handle.h>
#include <cutils/properties.h>
#include <drm/drm.h>
#include <drm/etnaviv_drm.h>
#include <hardware/gralloc.h>
#include "state_2d.xml.h"
#include <media/hardware/HardwareAPI.h>
#include <media/hardware/OMXPluginBase.h>
#include <media/openmax/OMX_Component.h>
#include <media/openmax/OMX_Core.h>
#include <media/openmax/OMX_Index.h>
#include <media/openmax/OMX_IndexExt.h>
#include <media/openmax/OMX_Video.h>
#include <media/openmax/OMX_VideoExt.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/MediaDefs.h>
#include <media/stagefright/omx/SoftVideoDecoderOMXComponent.h>
#include <media/stagefright/omx/SoftVideoEncoderOMXComponent.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Rect.h>
#include <utils/Log.h>

namespace android {

namespace {

constexpr uint32_t alignUp(uint32_t val, uint32_t align) {
    return (val + align - 1) & ~(align - 1);
}

template <class T>
void InitOMXParams(T *params) {
    memset(params, 0, sizeof(T));
    params->nSize = sizeof(T);
    params->nVersion.s.nVersionMajor = 1;
    params->nVersion.s.nVersionMinor = 0;
    params->nVersion.s.nRevision = 0;
    params->nVersion.s.nStep = 0;
}

int xioctl(int fd, unsigned long request, void *arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

/*
 * Scan /dev/video0..15 for a V4L2 M2M device with driver == "coda" whose
 * OUTPUT queue (decoder) or CAPTURE queue (encoder) supports targetPixFmt.
 */
int openCodaDevice(bool isEncoder, uint32_t targetPixFmt) {
    char path[64];
    enum v4l2_buf_type enumType = isEncoder
            ? V4L2_BUF_TYPE_VIDEO_CAPTURE
            : V4L2_BUF_TYPE_VIDEO_OUTPUT;
    for (int i = 0; i < 16; ++i) {
        snprintf(path, sizeof(path), "/dev/video%d", i);
        int fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            continue;
        }
        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                                    ? cap.device_caps
                                    : cap.capabilities;
            if ((caps & V4L2_CAP_VIDEO_M2M) &&
                strcmp(reinterpret_cast<const char *>(cap.driver), "coda") == 0) {
                struct v4l2_fmtdesc fdesc;
                memset(&fdesc, 0, sizeof(fdesc));
                fdesc.type = enumType;
                while (xioctl(fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {
                    if (fdesc.pixelformat == targetPixFmt) {
                        ALOGI("Opened CODA %s (%s) at %s",
                              isEncoder ? "encoder" : "decoder",
                              reinterpret_cast<const char *>(cap.card), path);
                        return fd;
                    }
                    ++fdesc.index;
                }
            }
        }
        close(fd);
    }
    ALOGE("Could not find CODA V4L2 %s supporting fourcc 0x%08x",
          isEncoder ? "encoder" : "decoder", targetPixFmt);
    return -1;
}

struct MmapBuffer {
    void *start = nullptr;
    size_t length = 0;
    bool queued = false;
    uint32_t gemHandle = 0;
};

/*
 * Hardware Vivante GC1000 2D (DE2D) YUV420 (YU12) -> RGBA_8888 blitter over
 * /dev/dri/renderD128 (etnaviv). Converts V4L2 CAPTURE DMA-BUF directly into
 * the destination RGBA_8888 GraphicBuffer with 0% CPU copy/CSC and 0 CMA.
 */
class Etna2dYuvBlitter {
public:
    Etna2dYuvBlitter() : mDrmFd(-1), mAvailable(false) {}

    ~Etna2dYuvBlitter() {
        closeDevice();
    }

    bool init() {
        if (mAvailable) {
            return true;
        }
        mDrmFd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
        if (mDrmFd < 0) {
            ALOGW("Etna2dYuvBlitter: failed to open /dev/dri/renderD128: %s", strerror(errno));
            return false;
        }
        struct drm_etnaviv_param param;
        memset(&param, 0, sizeof(param));
        param.pipe = 0;
        param.param = ETNAVIV_PARAM_GPU_FEATURES_0;
        if (xioctl(mDrmFd, DRM_IOCTL_ETNAVIV_GET_PARAM, &param) < 0 ||
            (param.value & 0x00000200u) == 0) {
            ALOGW("Etna2dYuvBlitter: GPU has no 2D pipe (features0=0x%" PRIx64 ")",
                  static_cast<uint64_t>(param.value));
            closeDevice();
            return false;
        }
        mAvailable = true;
        char prop[PROPERTY_VALUE_MAX];
        property_get("debug.coda.blit", prop, "0");
        mBlitFlags = static_cast<uint32_t>(strtoul(prop, nullptr, 0));
        property_get("debug.coda.blit.unity", prop, "4000");
        mUnity = static_cast<uint32_t>(strtoul(prop, nullptr, 16));
        ALOGI("Etna2dYuvBlitter: initialized Vivante GC1000 2D VR filter blitter "
              "(debug.coda.blit=0x%x unity=0x%x)", mBlitFlags, mUnity);
        return true;
    }

    bool isAvailable() const { return mAvailable; }

    uint32_t importPrimeFd(int primeFd) {
        if (!mAvailable || primeFd < 0) {
            return 0;
        }
        struct drm_prime_handle prime;
        memset(&prime, 0, sizeof(prime));
        prime.fd = primeFd;
        if (xioctl(mDrmFd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &prime) < 0) {
            ALOGW("Etna2dYuvBlitter: PRIME_FD_TO_HANDLE(%d) failed: %s", primeFd, strerror(errno));
            return 0;
        }
        return prime.handle;
    }

    void closeGemHandle(uint32_t handle) {
        if (mDrmFd >= 0 && handle != 0) {
            struct drm_gem_close closeArg;
            memset(&closeArg, 0, sizeof(closeArg));
            closeArg.handle = handle;
            xioctl(mDrmFd, DRM_IOCTL_GEM_CLOSE, &closeArg);
        }
    }

    bool blitYuv420ToRgba(uint32_t srcHandle, uint32_t yOffset, uint32_t uOffset, uint32_t vOffset,
                          uint32_t srcYStride, uint32_t srcUVStride,
                          const void *dstKey, int dstPrimeFd, uint32_t dstStride,
                          uint32_t width, uint32_t height) {
        if (!mAvailable || srcHandle == 0 || dstPrimeFd < 0) {
            return false;
        }
        uint32_t dstHandle = getOrImportDstHandle(dstKey, dstPrimeFd);
        if (dstHandle == 0) {
            return false;
        }

        constexpr uint32_t kFeOpLoadState = 0x08000000u;
        constexpr uint32_t kFeOpDraw2D = 0x20000000u;
        constexpr uint32_t kFeOpStall = 0x48000000u;
        constexpr uint32_t kRegGlSemaphoreToken = 0x00003808u;
        constexpr uint32_t kRegGlFlushCache = 0x0000380cu;
        constexpr uint32_t kGlFlushCachePe2D = 0x00000008u;
        constexpr uint32_t kSyncTokenFePe = 0x1u | (0x7u << 8);

        auto loadStateHdr = [](uint32_t reg) -> uint32_t {
            return kFeOpLoadState | (1u << 16) | ((reg >> 2) & 0xffffu);
        };

        std::vector<uint32_t> words;
        words.reserve(80);
        std::vector<struct drm_etnaviv_gem_submit_reloc> relocs;
        relocs.reserve(4);

        auto emitState = [&](uint32_t reg, uint32_t val) {
            words.push_back(loadStateHdr(reg));
            words.push_back(val);
        };
        auto emitReloc = [&](uint32_t reg, uint32_t boIdx, uint32_t offset) {
            words.push_back(loadStateHdr(reg));
            struct drm_etnaviv_gem_submit_reloc r;
            memset(&r, 0, sizeof(r));
            r.submit_offset = static_cast<uint32_t>(words.size() * sizeof(uint32_t));
            r.reloc_idx = boIdx;
            r.reloc_offset = offset;
            relocs.push_back(r);
            words.push_back(0);
        };

        // BO 0 = srcHandle (READ), BO 1 = dstHandle (WRITE)
        // Stall FE->PE at the start of the job so any VIVS_GL_FLUSH_MMU emitted
        // by the kernel's etnaviv_buffer_queue() completes before PE fetches.
        emitState(kRegGlSemaphoreToken, kSyncTokenFePe);
        words.push_back(kFeOpStall);
        words.push_back(kSyncTokenFePe);

        /*
         * Planar YUV420 is only consumed by the Vivante 2D "video resampler" (VR)
         * one-pass filter blit; a plain BIT_BLT/DRAW_2D never starts it (which is
         * why the destination stayed all-zero). Sequence per galcore's FilterBlit:
         *   1) load the 9-tap x 17-phase filter kernel
         *   2) program Y/U/V source planes + VR source image/origin/target window
         *   3) COMMAND = ONE_PASS_FILTER_BLT, then kick VR_CONFIG.START = ONE_PASS_BLIT
         * Register-field "_MASK" bits are write-enables that must be 0 to update the
         * field (the working etna2d compositor writes ROT_ANGLE the same way).
         *
         * Uncertain details are runtime-tunable (no rebuild) via
         *   setprop debug.coda.blit <bitmask>   bit0: swap U/V (UV_SWIZZLE)
         *                                       bit1: also emit DRAW_2D after VR start
         *                                       bit2: inclusive (w-1,h-1) window coords
         *   setprop debug.coda.blit.unity <hex>  filter coefficient meaning 1.0 (def 4000)
         */
        const uint32_t kUnity = mUnity;
        const bool use9Tap = (mBlitFlags & 8u) != 0;
        std::vector<uint32_t> kernelWords((17 * 9 + 1) / 2, 0);
        for (uint32_t phase = 0; phase < 17; ++phase) {
            uint32_t idx = phase * 9 + 4;
            kernelWords[idx >> 1] |= (kUnity & 0xffffu) << ((idx & 1) ? 16 : 0);
        }
        auto emitStates = [&](uint32_t reg, const std::vector<uint32_t> &data) {
            uint32_t n = static_cast<uint32_t>(data.size());
            words.push_back(kFeOpLoadState | (n << 16) | ((reg >> 2) & 0xffffu));
            for (uint32_t w : data) {
                words.push_back(w);
            }
            if ((words.size() & 1) != 0) {
                words.push_back(0);  // keep 64-bit command alignment
            }
        };
        emitStates(0x00001800u, kernelWords);  // VIVS_DE_FILTER_KERNEL
        emitStates(0x00002800u, kernelWords);  // VIVS_DE_HORI_FILTER_KERNEL
        emitStates(0x00002a00u, kernelWords);  // VIVS_DE_VERTI_FILTER_KERNEL
        emitState(VIVS_DE_VR_CONFIG_EX,
                  VIVS_DE_VR_CONFIG_EX_FILTER_TAP(use9Tap ? 9 : 1));

        // Etna2d state is not saved/restored between GPU users, and the HWC 2D
        // compositor shares this pipe. Reset anything it (or an earlier job) may
        // have left that could redirect fetches (the first job after open faults).
        emitState(VIVS_DE_DE_MULTI_SOURCE, 0);
        emitState(VIVS_DE_DEYUV_CONVERSION, VIVS_DE_DEYUV_CONVERSION_ENABLE_OFF);
        emitState(VIVS_DE_PATTERN_CONFIG, VIVS_DE_PATTERN_CONFIG_TYPE_SOLID_COLOR);
        emitState(VIVS_DE_SRC_COLOR_BG, 0);
        emitState(VIVS_DE_SRC_COLOR_FG, 0);

        emitReloc(VIVS_DE_SRC_ADDRESS, 0, yOffset);
        emitState(VIVS_DE_SRC_STRIDE, VIVS_DE_SRC_STRIDE_STRIDE(srcYStride));
        emitReloc(VIVS_DE_UPLANE_ADDRESS, 0, uOffset);
        emitState(VIVS_DE_UPLANE_STRIDE, VIVS_DE_UPLANE_STRIDE_STRIDE(srcUVStride));
        emitReloc(VIVS_DE_VPLANE_ADDRESS, 0, vOffset);
        emitState(VIVS_DE_VPLANE_STRIDE, VIVS_DE_VPLANE_STRIDE_STRIDE(srcUVStride));

        emitState(VIVS_DE_SRC_ROTATION_CONFIG, 0);
        emitState(VIVS_DE_SRC_EX_CONFIG, 0);
        emitState(VIVS_DE_SRC_CONFIG,
                  VIVS_DE_SRC_CONFIG_PE10_SOURCE_FORMAT(DE_FORMAT_YV12 & 0xF) |
                  VIVS_DE_SRC_CONFIG_SOURCE_FORMAT(DE_FORMAT_YV12) |
                  VIVS_DE_SRC_CONFIG_SWIZZLE(DE_SWIZZLE_ARGB) |
                  VIVS_DE_SRC_CONFIG_SRC_RELATIVE_ABSOLUTE |
                  VIVS_DE_SRC_CONFIG_TILED_DISABLE |
                  VIVS_DE_SRC_CONFIG_LOCATION_MEMORY |
                  VIVS_DE_SRC_CONFIG_PACK_PACKED8 |
                  VIVS_DE_SRC_CONFIG_DISABLE420_L2_CACHE);
        emitState(VIVS_DE_SRC_ORIGIN, 0);
        emitState(VIVS_DE_SRC_SIZE,
                  VIVS_DE_SRC_SIZE_X(width) | VIVS_DE_SRC_SIZE_Y(height));
        emitState(VIVS_DE_SRC_ORIGIN_FRACTION, 0);
        emitState(VIVS_DE_STRETCH_FACTOR_LOW,
                  VIVS_DE_STRETCH_FACTOR_LOW_X(0x10000u));
        emitState(VIVS_DE_STRETCH_FACTOR_HIGH,
                  VIVS_DE_STRETCH_FACTOR_HIGH_Y(0x10000u));

        emitReloc(VIVS_DE_DEST_ADDRESS, 1, 0);
        emitState(VIVS_DE_DEST_STRIDE, VIVS_DE_DEST_STRIDE_STRIDE(dstStride));
        emitState(VIVS_DE_DEST_ROTATION_CONFIG, 0);
        emitState(VIVS_DE_SRC_ROTATION_HEIGHT, 0);
        emitState(VIVS_DE_DEST_ROTATION_HEIGHT, 0);
        emitState(VIVS_DE_DEST_CONFIG,
                  VIVS_DE_DEST_CONFIG_FORMAT(DE_FORMAT_X8R8G8B8) |
                  VIVS_DE_DEST_CONFIG_COMMAND_ONE_PASS_FILTER_BLT |
                  VIVS_DE_DEST_CONFIG_SWIZZLE(DE_SWIZZLE_ABGR) |
                  VIVS_DE_DEST_CONFIG_TILED_DISABLE);

        emitState(VIVS_DE_CLIP_TOP_LEFT, 0);
        emitState(VIVS_DE_CLIP_BOTTOM_RIGHT,
                  VIVS_DE_CLIP_BOTTOM_RIGHT_X(width) |
                  VIVS_DE_CLIP_BOTTOM_RIGHT_Y(height));
        emitState(VIVS_DE_CONFIG, 0);
        emitState(VIVS_DE_ROT_ANGLE, 0);
        emitState(VIVS_DE_ROP,
                  VIVS_DE_ROP_TYPE_ROP3 |
                  VIVS_DE_ROP_ROP_FG(0xCC) |
                  VIVS_DE_ROP_ROP_BG(0xCC));
        emitState(VIVS_DE_PE_CONFIG, VIVS_DE_PE_CONFIG_DESTINATION_FETCH_DISABLE);
        // Mask bits left 0 => fields are written (see comment above).
        emitState(VIVS_DE_PE_CONTROL,
                  VIVS_DE_PE_CONTROL_YUV_601 |
                  ((mBlitFlags & 1u) ? VIVS_DE_PE_CONTROL_UV_SWIZZLE_VU
                                     : VIVS_DE_PE_CONTROL_UV_SWIZZLE_UV) |
                  VIVS_DE_PE_CONTROL_YUVRGB_ENABLE);
        emitState(VIVS_DE_ALPHA_CONTROL, VIVS_DE_ALPHA_CONTROL_ENABLE_OFF);
        emitState(VIVS_DE_COLOR_MULTIPLY_MODES, 0);

        const uint32_t wEnd = (mBlitFlags & 4u) ? width - 1 : width;
        const uint32_t hEnd = (mBlitFlags & 4u) ? height - 1 : height;
        emitState(VIVS_DE_VR_SOURCE_IMAGE_LOW,
                  VIVS_DE_VR_SOURCE_IMAGE_LOW_LEFT(0) | VIVS_DE_VR_SOURCE_IMAGE_LOW_TOP(0));
        emitState(VIVS_DE_VR_SOURCE_IMAGE_HIGH,
                  VIVS_DE_VR_SOURCE_IMAGE_HIGH_RIGHT(wEnd) |
                  VIVS_DE_VR_SOURCE_IMAGE_HIGH_BOTTOM(hEnd));
        emitState(VIVS_DE_VR_SOURCE_ORIGIN_LOW, 0);
        emitState(VIVS_DE_VR_SOURCE_ORIGIN_HIGH, 0);
        emitState(VIVS_DE_VR_TARGET_WINDOW_LOW,
                  VIVS_DE_VR_TARGET_WINDOW_LOW_LEFT(0) | VIVS_DE_VR_TARGET_WINDOW_LOW_TOP(0));
        emitState(VIVS_DE_VR_TARGET_WINDOW_HIGH,
                  VIVS_DE_VR_TARGET_WINDOW_HIGH_RIGHT(wEnd) |
                  VIVS_DE_VR_TARGET_WINDOW_HIGH_BOTTOM(hEnd));

        // Kick the resampler (write-triggered).
        emitState(VIVS_DE_VR_CONFIG, VIVS_DE_VR_CONFIG_START_ONE_PASS_BLIT);

        if (mBlitFlags & 2u) {
            // Draw2D (0, 0) -> (width, height)
            words.push_back(kFeOpDraw2D | (1u << 8));
            words.push_back(0);
            words.push_back(0);
            words.push_back((static_cast<uint32_t>(height) << 16) | static_cast<uint32_t>(width));
        }

        // Flush PE2D cache & stall FE->PE, then restore YUVRGB=disabled for shared 2D pipe
        emitState(kRegGlFlushCache, kGlFlushCachePe2D);
        emitState(kRegGlSemaphoreToken, kSyncTokenFePe);
        words.push_back(kFeOpStall);
        words.push_back(kSyncTokenFePe);
        emitState(VIVS_DE_PE_CONTROL,
                  VIVS_DE_PE_CONTROL_YUVRGB_DISABLE);

        struct drm_etnaviv_gem_submit_bo bos[2];
        memset(bos, 0, sizeof(bos));
        bos[0].handle = srcHandle;
        bos[0].flags = ETNA_SUBMIT_BO_READ;
        bos[1].handle = dstHandle;
        bos[1].flags = ETNA_SUBMIT_BO_WRITE;

        struct drm_etnaviv_gem_submit req;
        memset(&req, 0, sizeof(req));
        req.pipe = 0;
        req.exec_state = ETNA_PIPE_2D;
        req.nr_bos = 2;
        req.nr_relocs = static_cast<uint32_t>(relocs.size());
        req.stream_size = static_cast<uint32_t>(words.size() * sizeof(uint32_t));
        req.bos = reinterpret_cast<uintptr_t>(bos);
        req.relocs = reinterpret_cast<uintptr_t>(relocs.data());
        req.stream = reinterpret_cast<uintptr_t>(words.data());
        req.flags = ETNA_SUBMIT_FENCE_FD_OUT;
        req.fence_fd = -1;

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int ret = xioctl(mDrmFd, DRM_IOCTL_ETNAVIV_GEM_SUBMIT, &req);
        bool done = false;
        if (ret == 0 && req.fence_fd >= 0) {
            struct pollfd fpfd;
            fpfd.fd = req.fence_fd;
            fpfd.events = POLLIN;
            fpfd.revents = 0;
            // A hung job is only signalled after the kernel's ~1 s hang timeout, so a
            // wait this long means the frame was NOT produced: report failure so the
            // caller converts it on the CPU instead of showing stale/green data.
            int pr = poll(&fpfd, 1, 120);
            done = (pr > 0);
            close(req.fence_fd);
        } else {
            ALOGW("Etna2dYuvBlitter: GEM_SUBMIT failed: ret=%d errno=%s", ret, strerror(errno));
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        long us = (t1.tv_sec - t0.tv_sec) * 1000000L + (t1.tv_nsec - t0.tv_nsec) / 1000L;
        ++mBlitCount;
        mBlitUsTotal += us;
        if (!done) {
            ALOGW("Etna2dYuvBlitter: blit #%u TIMED OUT/FAILED in %ld us",
                  mBlitCount, us);
        }

        return ret == 0 && done;
    }

    void clearDstCache() {
        for (const auto &entry : mDstCache) {
            closeGemHandle(entry.second);
        }
        mDstCache.clear();
    }

    void closeDevice() {
        clearDstCache();
        if (mDrmFd >= 0) {
            close(mDrmFd);
            mDrmFd = -1;
        }
        mAvailable = false;
    }

private:
    uint32_t getOrImportDstHandle(const void *key, int primeFd) {
        for (const auto &entry : mDstCache) {
            if (entry.first == key) {
                return entry.second;
            }
        }
        uint32_t h = importPrimeFd(primeFd);
        if (h != 0) {
            mDstCache.push_back({key, h});
        }
        return h;
    }

    int mDrmFd;
    bool mAvailable;
    uint32_t mBlitFlags = 0;
    uint32_t mUnity = 0x4000;
    uint32_t mBlitCount = 0;
    long mBlitUsTotal = 0;
    std::vector<std::pair<const void *, uint32_t>> mDstCache;
};

/*
 * Fast BT.601 limited-range YUV420 (YU12) -> RGBA_8888 conversion for a row range.
 * Uses ARM NEON SIMD when available to process 16 pixels per iteration with
 * 8-byte U/V loads and 32-byte vst4_u8 burst stores to write-combined gralloc memory.
 */
void yuv420ToRgba8888Slice(const uint8_t *srcY, const uint8_t *srcU, const uint8_t *srcV,
                           size_t srcYStride, size_t srcUVStride,
                           uint8_t *dstBase, size_t dstStride,
                           size_t width, size_t rowStart, size_t rowEnd) {
    auto clampU8 = [](int v) -> uint8_t {
        return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
    };

#ifdef __ARM_NEON
    const uint8x8_t vYBias = vdup_n_u8(16);
    const int16x8_t vUVBias = vdupq_n_s16(128);
    const int32x4_t vRound = vdupq_n_s32(128);
    const uint8x8_t vAlpha = vdup_n_u8(0xFF);
#endif

    for (size_t row = rowStart; row < rowEnd; row += 2) {
        const uint8_t *yRow0 = srcY + row * srcYStride;
        const uint8_t *yRow1 = (row + 1 < rowEnd)
                ? (srcY + (row + 1) * srcYStride)
                : nullptr;
        const uint8_t *uRow = srcU + (row >> 1) * srcUVStride;
        const uint8_t *vRow = srcV + (row >> 1) * srcUVStride;
        uint8_t *dRow0 = dstBase + row * dstStride;
        uint8_t *dRow1 = yRow1 ? (dstBase + (row + 1) * dstStride) : nullptr;

        size_t col = 0;
#ifdef __ARM_NEON
        for (; col + 16 <= width; col += 16) {
            uint8x8_t u8Raw = vld1_u8(uRow + (col >> 1));
            uint8x8_t v8Raw = vld1_u8(vRow + (col >> 1));
            int16x8_t uSub = vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(u8Raw)), vUVBias);
            int16x8_t vSub = vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(v8Raw)), vUVBias);

            int16x4x2_t uDup0 = vzip_s16(vget_low_s16(uSub), vget_low_s16(uSub));
            int16x4x2_t vDup0 = vzip_s16(vget_low_s16(vSub), vget_low_s16(vSub));
            int16x4x2_t uDup1 = vzip_s16(vget_high_s16(uSub), vget_high_s16(uSub));
            int16x4x2_t vDup1 = vzip_s16(vget_high_s16(vSub), vget_high_s16(vSub));

            int32x4_t rAdd0 = vmulq_n_s32(vmovl_s16(vDup0.val[0]), 409);
            int32x4_t rAdd1 = vmulq_n_s32(vmovl_s16(vDup0.val[1]), 409);
            int32x4_t rAdd2 = vmulq_n_s32(vmovl_s16(vDup1.val[0]), 409);
            int32x4_t rAdd3 = vmulq_n_s32(vmovl_s16(vDup1.val[1]), 409);

            int32x4_t gAdd0 = vmlsq_n_s32(vmulq_n_s32(vmovl_s16(uDup0.val[0]), -100),
                                          vmovl_s16(vDup0.val[0]), 208);
            int32x4_t gAdd1 = vmlsq_n_s32(vmulq_n_s32(vmovl_s16(uDup0.val[1]), -100),
                                          vmovl_s16(vDup0.val[1]), 208);
            int32x4_t gAdd2 = vmlsq_n_s32(vmulq_n_s32(vmovl_s16(uDup1.val[0]), -100),
                                          vmovl_s16(vDup1.val[0]), 208);
            int32x4_t gAdd3 = vmlsq_n_s32(vmulq_n_s32(vmovl_s16(uDup1.val[1]), -100),
                                          vmovl_s16(vDup1.val[1]), 208);

            int32x4_t bAdd0 = vmulq_n_s32(vmovl_s16(uDup0.val[0]), 516);
            int32x4_t bAdd1 = vmulq_n_s32(vmovl_s16(uDup0.val[1]), 516);
            int32x4_t bAdd2 = vmulq_n_s32(vmovl_s16(uDup1.val[0]), 516);
            int32x4_t bAdd3 = vmulq_n_s32(vmovl_s16(uDup1.val[1]), 516);

            auto convert16 = [&](const uint8_t *yPtr, uint8_t *dPtr) {
                uint8x16_t yRaw = vld1q_u8(yPtr);
                int16x8_t ySubLo = vreinterpretq_s16_u16(vsubl_u8(vget_low_u8(yRaw), vYBias));
                int16x8_t ySubHi = vreinterpretq_s16_u16(vsubl_u8(vget_high_u8(yRaw), vYBias));

                int32x4_t c0 = vmlal_n_s16(vRound, vget_low_s16(ySubLo), 298);
                int32x4_t c1 = vmlal_n_s16(vRound, vget_high_s16(ySubLo), 298);
                int32x4_t c2 = vmlal_n_s16(vRound, vget_low_s16(ySubHi), 298);
                int32x4_t c3 = vmlal_n_s16(vRound, vget_high_s16(ySubHi), 298);

                uint8x8x4_t rgba0;
                rgba0.val[0] = vqmovn_u16(vcombine_u16(
                        vqshrun_n_s32(vaddq_s32(c0, rAdd0), 8),
                        vqshrun_n_s32(vaddq_s32(c1, rAdd1), 8)));
                rgba0.val[1] = vqmovn_u16(vcombine_u16(
                        vqshrun_n_s32(vaddq_s32(c0, gAdd0), 8),
                        vqshrun_n_s32(vaddq_s32(c1, gAdd1), 8)));
                rgba0.val[2] = vqmovn_u16(vcombine_u16(
                        vqshrun_n_s32(vaddq_s32(c0, bAdd0), 8),
                        vqshrun_n_s32(vaddq_s32(c1, bAdd1), 8)));
                rgba0.val[3] = vAlpha;
                vst4_u8(dPtr, rgba0);

                uint8x8x4_t rgba1;
                rgba1.val[0] = vqmovn_u16(vcombine_u16(
                        vqshrun_n_s32(vaddq_s32(c2, rAdd2), 8),
                        vqshrun_n_s32(vaddq_s32(c3, rAdd3), 8)));
                rgba1.val[1] = vqmovn_u16(vcombine_u16(
                        vqshrun_n_s32(vaddq_s32(c2, gAdd2), 8),
                        vqshrun_n_s32(vaddq_s32(c3, gAdd3), 8)));
                rgba1.val[2] = vqmovn_u16(vcombine_u16(
                        vqshrun_n_s32(vaddq_s32(c2, bAdd2), 8),
                        vqshrun_n_s32(vaddq_s32(c3, bAdd3), 8)));
                rgba1.val[3] = vAlpha;
                vst4_u8(dPtr + 32, rgba1);
            };
            convert16(yRow0 + col, dRow0 + col * 4);
            if (dRow1) {
                convert16(yRow1 + col, dRow1 + col * 4);
            }
        }
#endif
        for (; col < width; col += 2) {
            int u = static_cast<int>(uRow[col >> 1]) - 128;
            int v = static_cast<int>(vRow[col >> 1]) - 128;
            int rAdd = (409 * v + 128);
            int gAdd = (-100 * u - 208 * v + 128);
            int bAdd = (516 * u + 128);

            int c00 = 298 * (static_cast<int>(yRow0[col]) - 16);
            dRow0[col * 4 + 0] = clampU8((c00 + rAdd) >> 8);
            dRow0[col * 4 + 1] = clampU8((c00 + gAdd) >> 8);
            dRow0[col * 4 + 2] = clampU8((c00 + bAdd) >> 8);
            dRow0[col * 4 + 3] = 0xFF;

            if (col + 1 < width) {
                int c01 = 298 * (static_cast<int>(yRow0[col + 1]) - 16);
                dRow0[(col + 1) * 4 + 0] = clampU8((c01 + rAdd) >> 8);
                dRow0[(col + 1) * 4 + 1] = clampU8((c01 + gAdd) >> 8);
                dRow0[(col + 1) * 4 + 2] = clampU8((c01 + bAdd) >> 8);
                dRow0[(col + 1) * 4 + 3] = 0xFF;
            }

            if (dRow1) {
                int c10 = 298 * (static_cast<int>(yRow1[col]) - 16);
                dRow1[col * 4 + 0] = clampU8((c10 + rAdd) >> 8);
                dRow1[col * 4 + 1] = clampU8((c10 + gAdd) >> 8);
                dRow1[col * 4 + 2] = clampU8((c10 + bAdd) >> 8);
                dRow1[col * 4 + 3] = 0xFF;

                if (col + 1 < width) {
                    int c11 = 298 * (static_cast<int>(yRow1[col + 1]) - 16);
                    dRow1[(col + 1) * 4 + 0] = clampU8((c11 + rAdd) >> 8);
                    dRow1[(col + 1) * 4 + 1] = clampU8((c11 + gAdd) >> 8);
                    dRow1[(col + 1) * 4 + 2] = clampU8((c11 + bAdd) >> 8);
                    dRow1[(col + 1) * 4 + 3] = 0xFF;
                }
            }
        }
    }
}

class YuvConvertWorker {
public:
    YuvConvertWorker() : mStop(false), mHasJob(false) {
        mThread = std::thread([this]() { threadLoop(); });
    }

    ~YuvConvertWorker() {
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mStop = true;
            mCv.notify_one();
        }
        if (mThread.joinable()) {
            mThread.join();
        }
    }

    void convert(const uint8_t *srcY, const uint8_t *srcU, const uint8_t *srcV,
                 size_t srcYStride, size_t srcUVStride,
                 uint8_t *dstBase, size_t dstStride,
                 size_t width, size_t height) {
        if (height < 240) {
            yuv420ToRgba8888Slice(srcY, srcU, srcV, srcYStride, srcUVStride,
                                  dstBase, dstStride, width, 0, height);
            return;
        }
        size_t midRow = (height >> 1) & ~static_cast<size_t>(1);
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mJob = {srcY, srcU, srcV, srcYStride, srcUVStride,
                    dstBase, dstStride, width, 0, midRow};
            mHasJob = true;
            mCv.notify_one();
        }
        yuv420ToRgba8888Slice(srcY, srcU, srcV, srcYStride, srcUVStride,
                              dstBase, dstStride, width, midRow, height);
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mDoneCv.wait(lock, [this]() { return !mHasJob; });
        }
    }

private:
    struct Job {
        const uint8_t *srcY;
        const uint8_t *srcU;
        const uint8_t *srcV;
        size_t srcYStride;
        size_t srcUVStride;
        uint8_t *dstBase;
        size_t dstStride;
        size_t width;
        size_t rowStart;
        size_t rowEnd;
    };

    void threadLoop() {
        for (;;) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mMutex);
                mCv.wait(lock, [this]() { return mStop || mHasJob; });
                if (mStop && !mHasJob) {
                    return;
                }
                job = mJob;
            }
            yuv420ToRgba8888Slice(job.srcY, job.srcU, job.srcV,
                                  job.srcYStride, job.srcUVStride,
                                  job.dstBase, job.dstStride,
                                  job.width, job.rowStart, job.rowEnd);
            {
                std::lock_guard<std::mutex> lock(mMutex);
                mHasJob = false;
                mDoneCv.notify_one();
            }
        }
    }

    std::mutex mMutex;
    std::condition_variable mCv;
    std::condition_variable mDoneCv;
    std::thread mThread;
    Job mJob{};
    bool mStop;
    bool mHasJob;
};

void yuv420ToRgba8888(const uint8_t *srcY, const uint8_t *srcU, const uint8_t *srcV,
                      size_t srcYStride, size_t srcUVStride,
                      uint8_t *dstBase, size_t dstStride,
                      size_t width, size_t height) {
    static YuvConvertWorker worker;
    worker.convert(srcY, srcU, srcV, srcYStride, srcUVStride,
                   dstBase, dstStride, width, height);
}

/*
 * Bitstream reader for lightweight H.264 SPS resolution & crop extraction.
 */
class BitReader {
public:
    BitReader(const uint8_t *data, size_t size) : mData(data), mSize(size), mBitPos(0) {}

    uint32_t getBits(size_t n) {
        uint32_t val = 0;
        for (size_t i = 0; i < n; ++i) {
            size_t bytePos = mBitPos >> 3;
            if (bytePos >= mSize) {
                return 0;
            }
            size_t bitInByte = 7 - (mBitPos & 7);
            val = (val << 1) | ((mData[bytePos] >> bitInByte) & 1u);
            ++mBitPos;
        }
        return val;
    }

    uint32_t getUE() {
        size_t leadingZeroBits = 0;
        while (mBitPos < (mSize << 3) && getBits(1) == 0) {
            ++leadingZeroBits;
            if (leadingZeroBits > 31) {
                return 0;
            }
        }
        if (leadingZeroBits == 0) {
            return 0;
        }
        return (1u << leadingZeroBits) - 1u + getBits(leadingZeroBits);
    }

    int32_t getSE() {
        uint32_t ue = getUE();
        int32_t val = static_cast<int32_t>((ue + 1) >> 1);
        return (ue & 1) ? val : -val;
    }

private:
    const uint8_t *mData;
    size_t mSize;
    size_t mBitPos;
};

bool parseH264SpsDimensions(const uint8_t *buf, size_t size,
                            uint32_t *visibleWidth, uint32_t *visibleHeight) {
    // Locate NAL type 7 (SPS) in Annex-B stream
    for (size_t i = 0; i + 4 < size; ++i) {
        size_t scLen = 0;
        if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1) {
            scLen = 3;
        } else if (i + 5 < size && buf[i] == 0 && buf[i + 1] == 0 &&
                   buf[i + 2] == 0 && buf[i + 3] == 1) {
            scLen = 4;
        }
        if (scLen == 0) {
            continue;
        }
        size_t nalStart = i + scLen;
        uint8_t nalType = buf[nalStart] & 0x1F;
        if (nalType != 7) {
            continue;
        }
        // Find end of NAL
        size_t nalEnd = size;
        for (size_t j = nalStart + 1; j + 3 <= size; ++j) {
            if (buf[j] == 0 && buf[j + 1] == 0 &&
                (buf[j + 2] == 1 || (j + 4 <= size && buf[j + 2] == 0 && buf[j + 3] == 1))) {
                nalEnd = j;
                break;
            }
        }
        // Unescape emulation_prevention_three_byte (00 00 03)
        std::vector<uint8_t> rbsp;
        rbsp.reserve(nalEnd - nalStart);
        for (size_t j = nalStart + 1; j < nalEnd; ++j) {
            if (j + 2 < nalEnd && buf[j] == 0 && buf[j + 1] == 0 && buf[j + 2] == 3) {
                rbsp.push_back(0);
                rbsp.push_back(0);
                j += 2;
            } else {
                rbsp.push_back(buf[j]);
            }
        }
        if (rbsp.size() < 4) {
            return false;
        }

        BitReader br(rbsp.data(), rbsp.size());
        uint32_t profile_idc = br.getBits(8);
        br.getBits(8); // constraint_set_flags + reserved
        br.getBits(8); // level_idc
        br.getUE();    // seq_parameter_set_id

        uint32_t chroma_format_idc = 1;
        if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
            profile_idc == 244 || profile_idc == 44 || profile_idc == 83 ||
            profile_idc == 86 || profile_idc == 118 || profile_idc == 128 ||
            profile_idc == 138 || profile_idc == 139 || profile_idc == 134) {
            chroma_format_idc = br.getUE();
            if (chroma_format_idc == 3) {
                br.getBits(1); // separate_colour_plane_flag
            }
            br.getUE(); // bit_depth_luma_minus8
            br.getUE(); // bit_depth_chroma_minus8
            br.getBits(1); // qpprime_y_zero_transform_bypass_flag
            uint32_t seq_scaling_matrix_present_flag = br.getBits(1);
            if (seq_scaling_matrix_present_flag) {
                size_t numLists = (chroma_format_idc != 3) ? 8 : 12;
                for (size_t k = 0; k < numLists; ++k) {
                    if (br.getBits(1)) {
                        size_t listSize = (k < 6) ? 16 : 64;
                        int32_t lastScale = 8, nextScale = 8;
                        for (size_t s = 0; s < listSize; ++s) {
                            if (nextScale != 0) {
                                int32_t delta = br.getSE();
                                nextScale = (lastScale + delta + 256) % 256;
                            }
                            lastScale = (nextScale == 0) ? lastScale : nextScale;
                        }
                    }
                }
            }
        }

        br.getUE(); // log2_max_frame_num_minus4
        uint32_t pic_order_cnt_type = br.getUE();
        if (pic_order_cnt_type == 0) {
            br.getUE(); // log2_max_pic_order_cnt_lsb_minus4
        } else if (pic_order_cnt_type == 1) {
            br.getBits(1); // delta_pic_order_always_zero_flag
            br.getSE();    // offset_for_non_ref_pic
            br.getSE();    // offset_for_top_to_bottom_field
            uint32_t num_ref = br.getUE();
            for (uint32_t k = 0; k < num_ref && k < 256; ++k) {
                br.getSE();
            }
        }
        br.getUE();    // max_num_ref_frames
        br.getBits(1); // gaps_in_frame_num_value_allowed_flag

        uint32_t pic_width_in_mbs_minus1 = br.getUE();
        uint32_t pic_height_in_map_units_minus1 = br.getUE();
        uint32_t frame_mbs_only_flag = br.getBits(1);
        if (!frame_mbs_only_flag) {
            br.getBits(1); // mb_adaptive_frame_field_flag
        }
        br.getBits(1); // direct_8x8_inference_flag

        uint32_t frame_cropping_flag = br.getBits(1);
        uint32_t crop_left = 0, crop_right = 0, crop_top = 0, crop_bottom = 0;
        if (frame_cropping_flag) {
            crop_left = br.getUE();
            crop_right = br.getUE();
            crop_top = br.getUE();
            crop_bottom = br.getUE();
        }

        uint32_t width = (pic_width_in_mbs_minus1 + 1) * 16;
        uint32_t height = (2 - frame_mbs_only_flag) * (pic_height_in_map_units_minus1 + 1) * 16;

        uint32_t cropUnitX = (chroma_format_idc == 0) ? 1 : 2;
        uint32_t cropUnitY = ((chroma_format_idc == 0) ? 1 : 2) * (2 - frame_mbs_only_flag);

        uint32_t subX = (crop_left + crop_right) * cropUnitX;
        uint32_t subY = (crop_top + crop_bottom) * cropUnitY;
        if (width > subX && height > subY && width <= 1920 && height <= 1088) {
            *visibleWidth = width - subX;
            *visibleHeight = height - subY;
            return true;
        }
        return false;
    }
    return false;
}

/*
 * Convert AVCC / HVCC (or raw Annex-B) buffer into Annex-B start-code format.
 */
void appendAsAnnexB(const uint8_t *data, size_t size, std::vector<uint8_t> *out) {
    if (size == 0) {
        return;
    }
    // Already Annex-B?
    if ((size >= 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1) ||
        (size >= 3 && data[0] == 0 && data[1] == 0 && data[2] == 1)) {
        out->insert(out->end(), data, data + size);
        return;
    }
    // AVCDecoderConfigurationRecord (configurationVersion == 1)
    if (size >= 7 && data[0] == 1) {
        size_t offset = 5;
        uint8_t numSps = data[offset++] & 0x1F;
        for (uint8_t i = 0; i < numSps && offset + 2 <= size; ++i) {
            uint16_t spsLen = (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
            offset += 2;
            if (offset + spsLen <= size) {
                const uint8_t sc[4] = {0, 0, 0, 1};
                out->insert(out->end(), sc, sc + 4);
                out->insert(out->end(), data + offset, data + offset + spsLen);
                offset += spsLen;
            }
        }
        if (offset < size) {
            uint8_t numPps = data[offset++];
            for (uint8_t i = 0; i < numPps && offset + 2 <= size; ++i) {
                uint16_t ppsLen = (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
                offset += 2;
                if (offset + ppsLen <= size) {
                    const uint8_t sc[4] = {0, 0, 0, 1};
                    out->insert(out->end(), sc, sc + 4);
                    out->insert(out->end(), data + offset, data + offset + ppsLen);
                    offset += ppsLen;
                }
            }
        }
        return;
    }
    // 4-byte length-prefixed NAL units
    size_t offset = 0;
    bool validLengthPrefixed = false;
    while (offset + 4 <= size) {
        uint32_t nalLen = (static_cast<uint32_t>(data[offset]) << 24) |
                          (static_cast<uint32_t>(data[offset + 1]) << 16) |
                          (static_cast<uint32_t>(data[offset + 2]) << 8) |
                          static_cast<uint32_t>(data[offset + 3]);
        if (nalLen == 0 || offset + 4 + nalLen > size) {
            validLengthPrefixed = false;
            break;
        }
        validLengthPrefixed = true;
        offset += 4 + nalLen;
    }
    if (validLengthPrefixed && offset == size) {
        offset = 0;
        while (offset + 4 <= size) {
            uint32_t nalLen = (static_cast<uint32_t>(data[offset]) << 24) |
                              (static_cast<uint32_t>(data[offset + 1]) << 16) |
                              (static_cast<uint32_t>(data[offset + 2]) << 8) |
                              static_cast<uint32_t>(data[offset + 3]);
            offset += 4;
            const uint8_t sc[4] = {0, 0, 0, 1};
            out->insert(out->end(), sc, sc + 4);
            out->insert(out->end(), data + offset, data + offset + nalLen);
            offset += nalLen;
        }
        return;
    }
    // Fallback: copy as-is
    out->insert(out->end(), data, data + size);
}

const CodecProfileLevel kAvcDecProfileLevels[] = {
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel4},
    {OMX_VIDEO_AVCProfileConstrainedBaseline, OMX_VIDEO_AVCLevel4},
    {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel4},
    {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel4},
};

const CodecProfileLevel kMpeg4DecProfileLevels[] = {
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level5},
    {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level5},
};

const CodecProfileLevel kMpeg2DecProfileLevels[] = {
    {OMX_VIDEO_MPEG2ProfileSimple, OMX_VIDEO_MPEG2LevelHL},
    {OMX_VIDEO_MPEG2ProfileMain, OMX_VIDEO_MPEG2LevelHL},
};

const CodecProfileLevel kAvcEncProfileLevels[] = {
    {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel31},
    {OMX_VIDEO_AVCProfileConstrainedBaseline, OMX_VIDEO_AVCLevel31},
};

const CodecProfileLevel kMpeg4EncProfileLevels[] = {
    {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level5},
};

} // namespace

// ============================================================================
// CodaV4L2Decoder (H.264 / MPEG-4 / MPEG-2 hardware decoder via coda-decoder)
// ============================================================================
class CodaV4L2Decoder : public SoftVideoDecoderOMXComponent {
public:
    CodaV4L2Decoder(const char *name,
                    const char *componentRole,
                    OMX_VIDEO_CODINGTYPE codingType,
                    uint32_t v4l2PixFmt,
                    const char *mimeType,
                    const CodecProfileLevel *profileLevels,
                    size_t numProfileLevels,
                    const OMX_CALLBACKTYPE *callbacks,
                    OMX_PTR appData,
                    OMX_COMPONENTTYPE **component)
        : SoftVideoDecoderOMXComponent(
                  name, componentRole, codingType,
                  profileLevels, numProfileLevels,
                  320 /* width */, 240 /* height */,
                  callbacks, appData, component),
          mV4L2PixFmt(v4l2PixFmt),
          mFd(-1),
          mV4L2Initialized(false),
          mStreamOnOut(false),
          mStreamOnCap(false),
          mEosQueued(false),
          mSignalledError(false),
          mNativeBuffersEnabled(false),
          mCodedWidth(320),
          mCodedHeight(240),
          mCaptureStride(320),
          mCaptureSliceHeight(240),
          mQueuedInputFrames(0) {
        initPorts(kNumInputBuffers, 512 * 1024 /* inputBufferSize */,
                  kNumOmxOutputBuffers, mimeType, 2 /* minCompressionRatio */);
    }

    virtual ~CodaV4L2Decoder() {
        stopAndCloseV4L2();
    }

protected:
    enum {
        kEnableAndroidNativeBuffersIndex = kDescribeHdrStaticInfoIndex + 1,
        kUseAndroidNativeBuffer2Index = kDescribeHdrStaticInfoIndex + 2,
        kGetAndroidNativeBufferUsageIndex = kDescribeHdrStaticInfoIndex + 3,
    };

    OMX_ERRORTYPE getExtensionIndex(const char *name, OMX_INDEXTYPE *index) override {
        if (!strcmp(name, "OMX.google.android.index.enableAndroidNativeBuffers")) {
            *index = static_cast<OMX_INDEXTYPE>(kEnableAndroidNativeBuffersIndex);
            return OMX_ErrorNone;
        }
        if (!strcmp(name, "OMX.google.android.index.useAndroidNativeBuffer2")) {
            *index = static_cast<OMX_INDEXTYPE>(kUseAndroidNativeBuffer2Index);
            return OMX_ErrorNone;
        }
        if (!strcmp(name, "OMX.google.android.index.getAndroidNativeBufferUsage")) {
            *index = static_cast<OMX_INDEXTYPE>(kGetAndroidNativeBufferUsageIndex);
            return OMX_ErrorNone;
        }
        return SoftVideoDecoderOMXComponent::getExtensionIndex(name, index);
    }

    OMX_ERRORTYPE internalGetParameter(OMX_INDEXTYPE index, OMX_PTR params) override {
        if (static_cast<int>(index) == kGetAndroidNativeBufferUsageIndex) {
            auto *usage = static_cast<GetAndroidNativeBufferUsageParams *>(params);
            if (!isValidOMXParam(usage) || usage->nPortIndex != kOutputPortIndex) {
                return OMX_ErrorBadParameter;
            }
            usage->nUsage = GRALLOC_USAGE_SW_WRITE_OFTEN;
            return OMX_ErrorNone;
        }
        OMX_ERRORTYPE err = SoftVideoDecoderOMXComponent::internalGetParameter(index, params);
        if (err == OMX_ErrorNone && mNativeBuffersEnabled) {
            if (index == OMX_IndexParamPortDefinition) {
                auto *def = static_cast<OMX_PARAM_PORTDEFINITIONTYPE *>(params);
                if (def->nPortIndex == kOutputPortIndex) {
                    def->format.video.eColorFormat =
                            static_cast<OMX_COLOR_FORMATTYPE>(HAL_PIXEL_FORMAT_RGBA_8888);
                    def->nBufferSize = alignUp(def->format.video.nFrameWidth, 32) * 4 *
                                       def->format.video.nFrameHeight;
                }
            } else if (index == OMX_IndexParamVideoPortFormat) {
                auto *fmt = static_cast<OMX_VIDEO_PARAM_PORTFORMATTYPE *>(params);
                if (fmt->nPortIndex == kOutputPortIndex) {
                    fmt->eColorFormat =
                            static_cast<OMX_COLOR_FORMATTYPE>(HAL_PIXEL_FORMAT_RGBA_8888);
                }
            }
        }
        return err;
    }

    OMX_ERRORTYPE internalSetParameter(OMX_INDEXTYPE index, const OMX_PTR params) override {
        if (static_cast<int>(index) == kEnableAndroidNativeBuffersIndex) {
            const auto *enableParams =
                    static_cast<const EnableAndroidNativeBuffersParams *>(params);
            if (!isValidOMXParam(enableParams) || enableParams->nPortIndex != kOutputPortIndex) {
                return OMX_ErrorBadParameter;
            }
            mNativeBuffersEnabled = (enableParams->enable == OMX_TRUE);
            return OMX_ErrorNone;
        }
        if (mNativeBuffersEnabled && index == OMX_IndexParamPortDefinition) {
            const auto *inDef = static_cast<const OMX_PARAM_PORTDEFINITIONTYPE *>(params);
            if (inDef->nPortIndex == kOutputPortIndex &&
                static_cast<int>(inDef->format.video.eColorFormat) == HAL_PIXEL_FORMAT_RGBA_8888) {
                OMX_PARAM_PORTDEFINITIONTYPE copy = *inDef;
                copy.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
                return SoftVideoDecoderOMXComponent::internalSetParameter(index, &copy);
            }
        } else if (mNativeBuffersEnabled && index == OMX_IndexParamVideoPortFormat) {
            const auto *inFmt = static_cast<const OMX_VIDEO_PARAM_PORTFORMATTYPE *>(params);
            if (inFmt->nPortIndex == kOutputPortIndex &&
                static_cast<int>(inFmt->eColorFormat) == HAL_PIXEL_FORMAT_RGBA_8888) {
                OMX_VIDEO_PARAM_PORTFORMATTYPE copy = *inFmt;
                copy.eColorFormat = OMX_COLOR_FormatYUV420Planar;
                return SoftVideoDecoderOMXComponent::internalSetParameter(index, &copy);
            }
        }
        return SoftVideoDecoderOMXComponent::internalSetParameter(index, params);
    }

    void onQueueFilled(OMX_U32 /* portIndex */) override {
        if (mSignalledError || mOutputPortSettingsChange != NONE) {
            return;
        }

        List<BufferInfo *> &inQueue = getPortQueue(kInputPortIndex);
        List<BufferInfo *> &outQueue = getPortQueue(kOutputPortIndex);

        // 1. Reclaim any consumed V4L2 OUTPUT buffers if streaming
        reclaimV4L2OutputBuffers();

        // 2. Process OMX input buffers
        while (!inQueue.empty()) {
            BufferInfo *inInfo = *inQueue.begin();
            OMX_BUFFERHEADERTYPE *inHeader = inInfo->mHeader;
            const uint8_t *inData = inHeader->pBuffer + inHeader->nOffset;
            size_t inLen = inHeader->nFilledLen;

            // Handle Codec Config (SPS/PPS / VOL)
            if (inHeader->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
                if (mV4L2PixFmt == V4L2_PIX_FMT_H264) {
                    appendAsAnnexB(inData, inLen, &mCsdData);
                    uint32_t spsW = 0, spsH = 0;
                    if (parseH264SpsDimensions(mCsdData.data(), mCsdData.size(), &spsW, &spsH)) {
                        if (spsW != mWidth || spsH != mHeight) {
                            ALOGI("SPS dimensions %ux%u (was %ux%u)", spsW, spsH, mWidth, mHeight);
                            bool portWillReset = false;
                            handlePortSettingsChange(&portWillReset, spsW, spsH);
                            inQueue.erase(inQueue.begin());
                            inInfo->mOwnedByUs = false;
                            notifyEmptyBufferDone(inHeader);
                            if (portWillReset) {
                                return;
                            }
                            continue;
                        }
                    }
                } else {
                    mCsdData.insert(mCsdData.end(), inData, inData + inLen);
                }

                inQueue.erase(inQueue.begin());
                inInfo->mOwnedByUs = false;
                notifyEmptyBufferDone(inHeader);
                continue;
            }

            // Prepare frame payload (prepending cached CSD if present)
            std::vector<uint8_t> framePayload;
            if (!mCsdData.empty()) {
                framePayload.insert(framePayload.end(), mCsdData.begin(), mCsdData.end());
            }
            if (inLen > 0) {
                if (mV4L2PixFmt == V4L2_PIX_FMT_H264) {
                    appendAsAnnexB(inData, inLen, &framePayload);
                } else {
                    framePayload.insert(framePayload.end(), inData, inData + inLen);
                }
            }

            // Before initializing V4L2, check if the first H.264 frame contains an in-band SPS
            if (!mV4L2Initialized && mV4L2PixFmt == V4L2_PIX_FMT_H264 && !framePayload.empty()) {
                uint32_t spsW = 0, spsH = 0;
                if (parseH264SpsDimensions(framePayload.data(), framePayload.size(), &spsW, &spsH)) {
                    if (spsW != mWidth || spsH != mHeight) {
                        ALOGI("In-band SPS dimensions %ux%u (was %ux%u)",
                              spsW, spsH, mWidth, mHeight);
                        bool portWillReset = false;
                        handlePortSettingsChange(&portWillReset, spsW, spsH);
                        if (portWillReset) {
                            return;
                        }
                    }
                }
            }

            // Initialize V4L2 device once we are ready to queue the first frame
            if (!mV4L2Initialized && (!framePayload.empty() || (inHeader->nFlags & OMX_BUFFERFLAG_EOS))) {
                if (!framePayload.empty()) {
                    if (!initV4L2()) {
                        ALOGE("Failed to initialize CODA7542 V4L2 decoder");
                        notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                        mSignalledError = true;
                        return;
                    }
                    mCsdData.clear();
                } else {
                    // Empty EOS before any frame was ever sent
                    inQueue.erase(inQueue.begin());
                    inInfo->mOwnedByUs = false;
                    notifyEmptyBufferDone(inHeader);
                    if (!outQueue.empty()) {
                        BufferInfo *outInfo = *outQueue.begin();
                        outQueue.erase(outQueue.begin());
                        OMX_BUFFERHEADERTYPE *outHeader = outInfo->mHeader;
                        outHeader->nFilledLen = 0;
                        outHeader->nTimeStamp = inHeader->nTimeStamp;
                        outHeader->nFlags = OMX_BUFFERFLAG_EOS;
                        outInfo->mOwnedByUs = false;
                        notifyFillBufferDone(outHeader);
                    }
                    return;
                }
            }

            if (!framePayload.empty()) {
                int outIdx = findFreeOutputBuffer();
                if (outIdx < 0) {
                    // All V4L2 OUTPUT buffers busy; drain capture first
                    break;
                }

                // coda_start_streaming requires >= 512 bytes in bitstream_fifo
                if (mQueuedInputFrames == 0 && framePayload.size() < 512) {
                    framePayload.resize(512, 0x00);
                }

                size_t copyLen = std::min(framePayload.size(), mOutBuffers[outIdx].length);
                memcpy(mOutBuffers[outIdx].start, framePayload.data(), copyLen);

                struct v4l2_buffer buf;
                memset(&buf, 0, sizeof(buf));
                buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = outIdx;
                buf.bytesused = copyLen;
                buf.timestamp.tv_sec = static_cast<time_t>(inHeader->nTimeStamp / 1000000LL);
                buf.timestamp.tv_usec = static_cast<suseconds_t>(inHeader->nTimeStamp % 1000000LL);

                if (xioctl(mFd, VIDIOC_QBUF, &buf) < 0) {
                    ALOGE("VIDIOC_QBUF(OUTPUT) failed: %s", strerror(errno));
                    notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                    mSignalledError = true;
                    return;
                }
                mOutBuffers[outIdx].queued = true;
                ++mQueuedInputFrames;
                mCsdData.clear();
            }

            bool isEos = (inHeader->nFlags & OMX_BUFFERFLAG_EOS) != 0;

            // Return input buffer to ACodec immediately so it keeps feeding frames
            // (coda_job_ready requires num_metas >= 2 unless stream_end is set).
            inQueue.erase(inQueue.begin());
            inInfo->mOwnedByUs = false;
            notifyEmptyBufferDone(inHeader);

            if (isEos) {
                sendDecoderStopCmd();
                mEosQueued = true;
            }

            // Start OUTPUT streaming once >= 2 frames are queued or EOS arrived
            if (!mStreamOnOut && (mQueuedInputFrames >= 2 || mEosQueued)) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                if (xioctl(mFd, VIDIOC_STREAMON, &type) < 0) {
                    ALOGE("VIDIOC_STREAMON(OUTPUT) failed: %s", strerror(errno));
                    notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                    mSignalledError = true;
                    return;
                }
                mStreamOnOut = true;
            }

            reclaimV4L2OutputBuffers();
        }

        // If client stopped feeding new frames right after 1 frame, start STREAMON anyway
        if (mV4L2Initialized && !mStreamOnOut && mQueuedInputFrames >= 1 && inQueue.empty()) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            if (xioctl(mFd, VIDIOC_STREAMON, &type) == 0) {
                mStreamOnOut = true;
                reclaimV4L2OutputBuffers();
            }
        }

        // 3. Dequeue decoded frames from V4L2 CAPTURE queue into OMX output buffers
        if (!mV4L2Initialized || !mStreamOnOut || !mStreamOnCap) {
            return;
        }

        bool dequeuedFrame = false;
        while (!outQueue.empty()) {
            struct pollfd pfd;
            pfd.fd = mFd;
            pfd.events = POLLIN | POLLPRI;
            pfd.revents = 0;

            // Wait for a decoded CAPTURE frame (don't include POLLOUT, which is
            // always ready on coda and would make poll() return in 0ms).
            // Once we've already dequeued a frame in this call, only check
            // non-blockingly for more so we don't stall the OMX looper thread.
            int timeoutMs = mEosQueued ? 150 : (dequeuedFrame ? 0 : 25);
            int pr = poll(&pfd, 1, timeoutMs);
            reclaimV4L2OutputBuffers();
            if (pr <= 0) {
                break;
            }

            struct v4l2_buffer capBuf;
            memset(&capBuf, 0, sizeof(capBuf));
            capBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            capBuf.memory = V4L2_MEMORY_MMAP;

            if (xioctl(mFd, VIDIOC_DQBUF, &capBuf) < 0) {
                if (errno == EAGAIN) {
                    break;
                }
                ALOGE("VIDIOC_DQBUF(CAPTURE) failed: %s", strerror(errno));
                break;
            }
            mCapBuffers[capBuf.index].queued = false;
            dequeuedFrame = true;

            // Check visible crop via VIDIOC_G_SELECTION
            uint32_t visW = mWidth;
            uint32_t visH = mHeight;
            struct v4l2_selection sel;
            memset(&sel, 0, sizeof(sel));
            sel.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            sel.target = V4L2_SEL_TGT_COMPOSE;
            if (xioctl(mFd, VIDIOC_G_SELECTION, &sel) == 0 &&
                sel.r.width > 0 && sel.r.height > 0) {
                visW = sel.r.width;
                visH = sel.r.height;
            }

            if (visW != mWidth || visH != mHeight) {
                bool portWillReset = false;
                handlePortSettingsChange(&portWillReset, visW, visH);
                if (portWillReset) {
                    // Re-queue capture buffer before port reset
                    if (xioctl(mFd, VIDIOC_QBUF, &capBuf) == 0) {
                        mCapBuffers[capBuf.index].queued = true;
                    }
                    return;
                }
            }

            BufferInfo *outInfo = *outQueue.begin();
            outQueue.erase(outQueue.begin());
            OMX_BUFFERHEADERTYPE *outHeader = outInfo->mHeader;

            if (capBuf.bytesused > 0) {
                const uint8_t *base =
                        static_cast<const uint8_t *>(mCapBuffers[capBuf.index].start);
                size_t yPlaneSize = mCaptureStride * mCaptureSliceHeight;
                const uint8_t *srcY = base;
                const uint8_t *srcU = base + yPlaneSize;
                const uint8_t *srcV = srcU + (yPlaneSize >> 2);

                if (mNativeBuffersEnabled) {
                    buffer_handle_t handle =
                            reinterpret_cast<buffer_handle_t>(outHeader->pBuffer);
                    size_t dstStride = alignUp(mWidth, 32) * 4;
                    size_t srcUVStride = mCaptureStride >> 1;
                    bool hwBlitted = false;

                    if (mEtna2d.isAvailable() &&
                        mCapBuffers[capBuf.index].gemHandle != 0 &&
                        handle && handle->numFds >= 1 && handle->data[0] >= 0) {
                        hwBlitted = mEtna2d.blitYuv420ToRgba(
                                mCapBuffers[capBuf.index].gemHandle,
                                0,
                                static_cast<uint32_t>(yPlaneSize),
                                static_cast<uint32_t>(yPlaneSize + (yPlaneSize >> 2)),
                                static_cast<uint32_t>(mCaptureStride),
                                static_cast<uint32_t>(srcUVStride),
                                handle,
                                handle->data[0],
                                static_cast<uint32_t>(dstStride),
                                mWidth, mHeight);
                    }

                    if (hwBlitted && !mEtna2dVerified) {
                        // Self-check the first GPU blit: if the destination is still all
                        // zero, the GC1000 2D engine did not convert YV12 on this part.
                        // Disable it for this decoder and fall back to the CPU NEON path.
                        bool allZero = true;
                        void *chkPtr = nullptr;
                        if (GraphicBufferMapper::get().lock(
                                    handle, GRALLOC_USAGE_SW_READ_OFTEN,
                                    Rect(mWidth, mHeight), &chkPtr) == OK && chkPtr) {
                            const uint32_t *p = static_cast<const uint32_t *>(chkPtr);
                            const size_t nWords = (dstStride * mHeight) / 4;
                            for (size_t i = 0; i < nWords && allZero; i += 61) {
                                allZero = (p[i] == 0);
                            }
                            GraphicBufferMapper::get().unlock(handle);
                        }
                        if (allZero) {
                            ALOGW("Etna2d YV12 blit produced no output; disabling GPU path, "
                                  "using CPU NEON conversion");
                            mEtna2d.closeDevice();
                            hwBlitted = false;
                        } else {
                            mEtna2dVerified = true;
                        }
                    }

                    if (hwBlitted) {
                        OMX_U32 rgbaSize = static_cast<OMX_U32>(dstStride * mHeight);
                        outHeader->nFilledLen = std::min(outHeader->nAllocLen, rgbaSize);
                    } else {
                        void *dstPtr = nullptr;
                        status_t lockRes = GraphicBufferMapper::get().lock(
                                handle,
                                GRALLOC_USAGE_SW_WRITE_OFTEN,
                                Rect(mWidth, mHeight),
                                &dstPtr);
                        if (lockRes == OK && dstPtr) {
                            uint8_t *dstBase = static_cast<uint8_t *>(dstPtr);
                            yuv420ToRgba8888(srcY, srcU, srcV,
                                             mCaptureStride, srcUVStride,
                                             dstBase, dstStride,
                                             mWidth, mHeight);
                            GraphicBufferMapper::get().unlock(handle);
                            OMX_U32 rgbaSize = static_cast<OMX_U32>(dstStride * mHeight);
                            outHeader->nFilledLen = std::min(outHeader->nAllocLen, rgbaSize);
                        } else {
                            ALOGE("GraphicBufferMapper::lock failed: %d", lockRes);
                            outHeader->nFilledLen = 0;
                        }
                    }
                } else {
                    copyYV12FrameToOutputBuffer(
                            outHeader->pBuffer + outHeader->nOffset,
                            srcY, srcU, srcV,
                            mCaptureStride, mCaptureStride >> 1, mCaptureStride >> 1);

                    outHeader->nFilledLen = (outputBufferWidth() * outputBufferHeight() * 3) / 2;
                }
            } else {
                outHeader->nFilledLen = 0;
            }

            outHeader->nOffset = 0;
            outHeader->nTimeStamp =
                    static_cast<OMX_TICKS>(capBuf.timestamp.tv_sec) * 1000000LL +
                    static_cast<OMX_TICKS>(capBuf.timestamp.tv_usec);
            outHeader->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;

            bool isLast = (capBuf.flags & V4L2_BUF_FLAG_LAST) != 0 ||
                          (mEosQueued && capBuf.bytesused == 0);
            if (isLast) {
                outHeader->nFlags |= OMX_BUFFERFLAG_EOS;
                mEosQueued = false;
            }

            // Re-queue CAPTURE buffer back to driver
            if (!isLast) {
                if (xioctl(mFd, VIDIOC_QBUF, &capBuf) == 0) {
                    mCapBuffers[capBuf.index].queued = true;
                }
            }

            outInfo->mOwnedByUs = false;
            notifyFillBufferDone(outHeader);

            if (isLast) {
                break;
            }
        }

        // If EOS was queued and no more frames remain in V4L2 CAPTURE, emit final EOS buffer
        if (mEosQueued && !outQueue.empty()) {
            struct v4l2_event ev;
            memset(&ev, 0, sizeof(ev));
            bool gotEosEvent = false;
            while (xioctl(mFd, VIDIOC_DQEVENT, &ev) == 0) {
                if (ev.type == V4L2_EVENT_EOS) {
                    gotEosEvent = true;
                }
            }
            if (gotEosEvent) {
                BufferInfo *outInfo = *outQueue.begin();
                outQueue.erase(outQueue.begin());
                OMX_BUFFERHEADERTYPE *outHeader = outInfo->mHeader;
                outHeader->nOffset = 0;
                outHeader->nFilledLen = 0;
                outHeader->nTimeStamp = 0;
                outHeader->nFlags = OMX_BUFFERFLAG_EOS;
                outInfo->mOwnedByUs = false;
                mEosQueued = false;
                notifyFillBufferDone(outHeader);
            }
        }
    }

    void onPortFlushCompleted(OMX_U32 portIndex) override {
        if (portIndex == kInputPortIndex && mV4L2Initialized) {
            // Reset V4L2 stream on input flush (seek)
            stopAndCloseV4L2();
        }
    }

    void onReset() override {
        SoftVideoDecoderOMXComponent::onReset();
        stopAndCloseV4L2();
        mCsdData.clear();
        mSignalledError = false;
    }

private:
    static constexpr uint32_t kNumInputBuffers = 4;
    static constexpr uint32_t kNumOmxOutputBuffers = 3;
    static constexpr uint32_t kNumV4L2CaptureBuffers = 6;

    uint32_t mV4L2PixFmt;
    int mFd;
    bool mV4L2Initialized;
    bool mStreamOnOut;
    bool mStreamOnCap;
    bool mEosQueued;
    bool mSignalledError;
    bool mNativeBuffersEnabled;
    uint32_t mCodedWidth;
    uint32_t mCodedHeight;
    size_t mCaptureStride;
    size_t mCaptureSliceHeight;
    uint32_t mQueuedInputFrames;
    std::vector<uint8_t> mCsdData;
    std::vector<MmapBuffer> mOutBuffers;
    std::vector<MmapBuffer> mCapBuffers;
    Etna2dYuvBlitter mEtna2d;
    bool mEtna2dVerified = false;

    bool initV4L2() {
        stopAndCloseV4L2();

        mFd = openCodaDevice(false /* isEncoder */, mV4L2PixFmt);
        if (mFd < 0) {
            return false;
        }

        struct v4l2_event_subscription sub;
        memset(&sub, 0, sizeof(sub));
        sub.type = V4L2_EVENT_EOS;
        xioctl(mFd, VIDIOC_SUBSCRIBE_EVENT, &sub);
        memset(&sub, 0, sizeof(sub));
        sub.type = V4L2_EVENT_SOURCE_CHANGE;
        xioctl(mFd, VIDIOC_SUBSCRIBE_EVENT, &sub);

        mCodedWidth = alignUp(mWidth, 16);
        mCodedHeight = alignUp(mHeight, 16);

        // Configure OUTPUT queue (compressed bitstream)
        struct v4l2_format fmtOut;
        memset(&fmtOut, 0, sizeof(fmtOut));
        fmtOut.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmtOut.fmt.pix.width = mCodedWidth;
        fmtOut.fmt.pix.height = mCodedHeight;
        fmtOut.fmt.pix.pixelformat = mV4L2PixFmt;
        fmtOut.fmt.pix.sizeimage = 512 * 1024;
        fmtOut.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(mFd, VIDIOC_S_FMT, &fmtOut) < 0) {
            ALOGE("VIDIOC_S_FMT(OUTPUT) failed: %s", strerror(errno));
            stopAndCloseV4L2();
            return false;
        }

        // Configure CAPTURE queue (decoded YUV420 / YU12 frames)
        struct v4l2_format fmtCap;
        memset(&fmtCap, 0, sizeof(fmtCap));
        fmtCap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmtCap.fmt.pix.width = mCodedWidth;
        fmtCap.fmt.pix.height = mCodedHeight;
        fmtCap.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
        fmtCap.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(mFd, VIDIOC_S_FMT, &fmtCap) < 0) {
            ALOGE("VIDIOC_S_FMT(CAPTURE) failed: %s", strerror(errno));
            stopAndCloseV4L2();
            return false;
        }

        mCaptureStride = fmtCap.fmt.pix.bytesperline
                                 ? fmtCap.fmt.pix.bytesperline
                                 : mCodedWidth;
        mCaptureSliceHeight = fmtCap.fmt.pix.height
                                      ? fmtCap.fmt.pix.height
                                      : mCodedHeight;

        if (!allocateMmapBuffers(V4L2_BUF_TYPE_VIDEO_OUTPUT, kNumInputBuffers, &mOutBuffers)) {
            stopAndCloseV4L2();
            return false;
        }

        if (!allocateMmapBuffers(V4L2_BUF_TYPE_VIDEO_CAPTURE, kNumV4L2CaptureBuffers, &mCapBuffers)) {
            stopAndCloseV4L2();
            return false;
        }

        // Export CAPTURE buffers via VIDIOC_EXPBUF for zero-copy GC1000 2D YUV->RGBA blit
        if (mEtna2d.init()) {
            for (size_t i = 0; i < mCapBuffers.size(); ++i) {
                struct v4l2_exportbuffer expbuf;
                memset(&expbuf, 0, sizeof(expbuf));
                expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                expbuf.index = static_cast<uint32_t>(i);
                expbuf.flags = O_RDWR | O_CLOEXEC;
                if (xioctl(mFd, VIDIOC_EXPBUF, &expbuf) == 0 && expbuf.fd >= 0) {
                    mCapBuffers[i].gemHandle = mEtna2d.importPrimeFd(expbuf.fd);
                    close(expbuf.fd);
                }
            }
        }

        // Queue all CAPTURE buffers and start CAPTURE streaming immediately
        for (size_t i = 0; i < mCapBuffers.size(); ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (xioctl(mFd, VIDIOC_QBUF, &buf) < 0) {
                ALOGE("VIDIOC_QBUF(CAPTURE %zu) failed: %s", i, strerror(errno));
                stopAndCloseV4L2();
                return false;
            }
            mCapBuffers[i].queued = true;
        }

        enum v4l2_buf_type capType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(mFd, VIDIOC_STREAMON, &capType) < 0) {
            ALOGE("VIDIOC_STREAMON(CAPTURE) failed: %s", strerror(errno));
            stopAndCloseV4L2();
            return false;
        }
        mStreamOnCap = true;
        mV4L2Initialized = true;
        mQueuedInputFrames = 0;

        ALOGI("Initialized CODA7542 decoder: %ux%u (coded %ux%u, stride=%zu)",
              mWidth, mHeight, mCodedWidth, mCodedHeight, mCaptureStride);
        return true;
    }

    bool allocateMmapBuffers(enum v4l2_buf_type type, uint32_t count,
                             std::vector<MmapBuffer> *buffers) {
        struct v4l2_requestbuffers req;
        uint32_t minCount = (type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ? 1u : 2u;
        uint32_t tryCount = count;
        int ret = -1;
        int lastErr = 0;
        while (tryCount >= minCount) {
            memset(&req, 0, sizeof(req));
            req.count = tryCount;
            req.type = type;
            req.memory = V4L2_MEMORY_MMAP;
            errno = 0;
            ret = xioctl(mFd, VIDIOC_REQBUFS, &req);
            lastErr = errno;
            if (ret == 0 && req.count >= minCount) {
                break;
            }
            if (ret < 0 && lastErr != ENOMEM) {
                break;
            }
            ALOGW("VIDIOC_REQBUFS(type=%d, count=%u) ret=%d got=%u err=%s, retrying",
                  type, tryCount, ret, req.count, strerror(lastErr));
            --tryCount;
        }
        if (ret < 0 || req.count < minCount) {
            ALOGE("VIDIOC_REQBUFS(type=%d) failed: ret=%d got=%u err=%s",
                  type, ret, req.count, strerror(lastErr));
            return false;
        }
        ALOGI("VIDIOC_REQBUFS(type=%d) allocated %u buffers", type, req.count);

        buffers->resize(req.count);
        for (uint32_t i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = type;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (xioctl(mFd, VIDIOC_QUERYBUF, &buf) < 0) {
                ALOGE("VIDIOC_QUERYBUF(type=%d, idx=%u) failed: %s", type, i, strerror(errno));
                return false;
            }
            void *ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, mFd, buf.m.offset);
            if (ptr == MAP_FAILED) {
                ALOGE("mmap(type=%d, idx=%u) failed: %s", type, i, strerror(errno));
                return false;
            }
            (*buffers)[i].start = ptr;
            (*buffers)[i].length = buf.length;
            (*buffers)[i].queued = false;
        }
        return true;
    }

    int findFreeOutputBuffer() const {
        for (size_t i = 0; i < mOutBuffers.size(); ++i) {
            if (!mOutBuffers[i].queued) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    void reclaimV4L2OutputBuffers() {
        if (!mV4L2Initialized || !mStreamOnOut) {
            return;
        }
        for (;;) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_MMAP;
            if (xioctl(mFd, VIDIOC_DQBUF, &buf) < 0) {
                break;
            }
            if (buf.index < mOutBuffers.size()) {
                mOutBuffers[buf.index].queued = false;
            }
        }
    }

    void sendDecoderStopCmd() {
        if (mFd < 0) {
            return;
        }
        struct v4l2_decoder_cmd cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = V4L2_DEC_CMD_STOP;
        if (xioctl(mFd, VIDIOC_DECODER_CMD, &cmd) < 0) {
            // Fallback: queue 0-byte OUTPUT buffer to trigger coda_bit_stream_end_flag
            int idx = findFreeOutputBuffer();
            if (idx >= 0) {
                struct v4l2_buffer buf;
                memset(&buf, 0, sizeof(buf));
                buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = idx;
                buf.bytesused = 0;
                if (xioctl(mFd, VIDIOC_QBUF, &buf) == 0) {
                    mOutBuffers[idx].queued = true;
                }
            }
        }
    }

    void stopAndCloseV4L2() {
        if (mFd >= 0) {
            if (mStreamOnOut) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                xioctl(mFd, VIDIOC_STREAMOFF, &type);
                mStreamOnOut = false;
            }
            if (mStreamOnCap) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                xioctl(mFd, VIDIOC_STREAMOFF, &type);
                mStreamOnCap = false;
            }
        }
        for (auto &b : mOutBuffers) {
            if (b.start && b.start != MAP_FAILED) {
                munmap(b.start, b.length);
            }
        }
        mOutBuffers.clear();
        for (auto &b : mCapBuffers) {
            if (b.gemHandle != 0) {
                mEtna2d.closeGemHandle(b.gemHandle);
                b.gemHandle = 0;
            }
            if (b.start && b.start != MAP_FAILED) {
                munmap(b.start, b.length);
            }
        }
        mCapBuffers.clear();
        mEtna2d.clearDstCache();
        if (mFd >= 0) {
            close(mFd);
            mFd = -1;
        }
        mV4L2Initialized = false;
        mEosQueued = false;
        mQueuedInputFrames = 0;
    }

    DISALLOW_EVIL_CONSTRUCTORS(CodaV4L2Decoder);
};

// ============================================================================
// CodaV4L2Encoder (H.264 / MPEG-4 hardware encoder via coda-encoder)
// ============================================================================
class CodaV4L2Encoder : public SoftVideoEncoderOMXComponent {
public:
    CodaV4L2Encoder(const char *name,
                    const char *componentRole,
                    OMX_VIDEO_CODINGTYPE codingType,
                    uint32_t v4l2PixFmt,
                    const char *mimeType,
                    const CodecProfileLevel *profileLevels,
                    size_t numProfileLevels,
                    const OMX_CALLBACKTYPE *callbacks,
                    OMX_PTR appData,
                    OMX_COMPONENTTYPE **component)
        : SoftVideoEncoderOMXComponent(
                  name, componentRole, codingType,
                  profileLevels, numProfileLevels,
                  176 /* width */, 144 /* height */,
                  callbacks, appData, component),
          mV4L2PixFmt(v4l2PixFmt),
          mFd(-1),
          mV4L2Initialized(false),
          mStreamOnOut(false),
          mStreamOnCap(false),
          mSentCodecConfig(false),
          mForceKeyFrame(false),
          mSignalledError(false),
          mGopSize(60),
          mAlignedWidth(176),
          mAlignedHeight(144),
          mOutStride(176) {
        initPorts(kNumBuffers, kNumBuffers, 512 * 1024 /* outputBufferSize */,
                  mimeType, 2 /* minCompressionRatio */);
    }

    virtual ~CodaV4L2Encoder() {
        stopAndCloseV4L2();
    }

protected:
    OMX_ERRORTYPE internalGetParameter(OMX_INDEXTYPE index, OMX_PTR params) override {
        switch (static_cast<int>(index)) {
            case OMX_IndexParamVideoBitrate: {
                auto *bitRate = static_cast<OMX_VIDEO_PARAM_BITRATETYPE *>(params);
                if (!isValidOMXParam(bitRate) || bitRate->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                bitRate->eControlRate = OMX_Video_ControlRateVariable;
                bitRate->nTargetBitrate = mBitrate;
                return OMX_ErrorNone;
            }
            case OMX_IndexParamVideoAvc: {
                if (mV4L2PixFmt != V4L2_PIX_FMT_H264) {
                    return OMX_ErrorUnsupportedIndex;
                }
                auto *avcParams = static_cast<OMX_VIDEO_PARAM_AVCTYPE *>(params);
                if (!isValidOMXParam(avcParams) || avcParams->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                avcParams->eProfile = OMX_VIDEO_AVCProfileBaseline;
                avcParams->eLevel = OMX_VIDEO_AVCLevel31;
                avcParams->nRefFrames = 1;
                avcParams->nBFrames = 0;
                avcParams->nPFrames = mGopSize - 1;
                avcParams->bUseHadamard = OMX_TRUE;
                avcParams->bEnableUEP = OMX_FALSE;
                avcParams->bEnableFMO = OMX_FALSE;
                avcParams->bEnableASO = OMX_FALSE;
                avcParams->bEnableRS = OMX_FALSE;
                avcParams->bFrameMBsOnly = OMX_TRUE;
                avcParams->bMBAFF = OMX_FALSE;
                avcParams->bEntropyCodingCABAC = OMX_FALSE;
                avcParams->eLoopFilterMode = OMX_VIDEO_AVCLoopFilterEnable;
                return OMX_ErrorNone;
            }
            case OMX_IndexParamVideoMpeg4: {
                if (mV4L2PixFmt != V4L2_PIX_FMT_MPEG4) {
                    return OMX_ErrorUnsupportedIndex;
                }
                auto *mpeg4Params = static_cast<OMX_VIDEO_PARAM_MPEG4TYPE *>(params);
                if (!isValidOMXParam(mpeg4Params) || mpeg4Params->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                mpeg4Params->eProfile = OMX_VIDEO_MPEG4ProfileSimple;
                mpeg4Params->eLevel = OMX_VIDEO_MPEG4Level5;
                mpeg4Params->nPFrames = mGopSize - 1;
                mpeg4Params->nBFrames = 0;
                return OMX_ErrorNone;
            }
            case OMX_IndexParamVideoProfileLevelCurrent: {
                auto *profileLevel = static_cast<OMX_VIDEO_PARAM_PROFILELEVELTYPE *>(params);
                if (!isValidOMXParam(profileLevel) || profileLevel->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                if (mV4L2PixFmt == V4L2_PIX_FMT_H264) {
                    profileLevel->eProfile = OMX_VIDEO_AVCProfileBaseline;
                    profileLevel->eLevel = OMX_VIDEO_AVCLevel31;
                } else {
                    profileLevel->eProfile = OMX_VIDEO_MPEG4ProfileSimple;
                    profileLevel->eLevel = OMX_VIDEO_MPEG4Level5;
                }
                return OMX_ErrorNone;
            }
            default:
                return SoftVideoEncoderOMXComponent::internalGetParameter(index, params);
        }
    }

    OMX_ERRORTYPE internalSetParameter(OMX_INDEXTYPE index, const OMX_PTR params) override {
        switch (static_cast<int>(index)) {
            case OMX_IndexParamVideoBitrate: {
                const auto *bitRate = static_cast<const OMX_VIDEO_PARAM_BITRATETYPE *>(params);
                if (!isValidOMXParam(bitRate) || bitRate->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                mBitrate = bitRate->nTargetBitrate;
                return OMX_ErrorNone;
            }
            case OMX_IndexParamVideoAvc: {
                if (mV4L2PixFmt != V4L2_PIX_FMT_H264) {
                    return OMX_ErrorUnsupportedIndex;
                }
                const auto *avcParams = static_cast<const OMX_VIDEO_PARAM_AVCTYPE *>(params);
                if (!isValidOMXParam(avcParams) || avcParams->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                if (avcParams->nPFrames > 0) {
                    mGopSize = std::max<uint32_t>(avcParams->nPFrames + 1, 30u);
                }
                return OMX_ErrorNone;
            }
            case OMX_IndexParamVideoMpeg4: {
                if (mV4L2PixFmt != V4L2_PIX_FMT_MPEG4) {
                    return OMX_ErrorUnsupportedIndex;
                }
                const auto *mpeg4Params = static_cast<const OMX_VIDEO_PARAM_MPEG4TYPE *>(params);
                if (!isValidOMXParam(mpeg4Params) || mpeg4Params->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                if (mpeg4Params->nPFrames > 0) {
                    mGopSize = std::max<uint32_t>(mpeg4Params->nPFrames + 1, 30u);
                }
                return OMX_ErrorNone;
            }
            case OMX_IndexParamVideoProfileLevelCurrent: {
                const auto *profileLevel =
                        static_cast<const OMX_VIDEO_PARAM_PROFILELEVELTYPE *>(params);
                if (!isValidOMXParam(profileLevel) || profileLevel->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                return OMX_ErrorNone;
            }
            default:
                return SoftVideoEncoderOMXComponent::internalSetParameter(index, params);
        }
    }

    OMX_ERRORTYPE setConfig(OMX_INDEXTYPE index, const OMX_PTR params) override {
        switch (static_cast<int>(index)) {
            case OMX_IndexConfigVideoIntraVOPRefresh: {
                const auto *intra =
                        static_cast<const OMX_CONFIG_INTRAREFRESHVOPTYPE *>(params);
                if (!isValidOMXParam(intra) || intra->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                if (intra->IntraRefreshVOP) {
                    mForceKeyFrame = true;
                }
                return OMX_ErrorNone;
            }
            case OMX_IndexConfigVideoBitrate: {
                const auto *bitRate =
                        static_cast<const OMX_VIDEO_CONFIG_BITRATETYPE *>(params);
                if (!isValidOMXParam(bitRate) || bitRate->nPortIndex != kOutputPortIndex) {
                    return OMX_ErrorBadParameter;
                }
                mBitrate = bitRate->nEncodeBitrate;
                if (mFd >= 0) {
                    setCtrl(V4L2_CID_MPEG_VIDEO_BITRATE,
                            static_cast<int32_t>(std::max<uint32_t>(mBitrate, 512000u)));
                }
                return OMX_ErrorNone;
            }
            default:
                return SimpleSoftOMXComponent::setConfig(index, params);
        }
    }

    void onQueueFilled(OMX_U32 /* portIndex */) override {
        if (mSignalledError) {
            return;
        }

        List<BufferInfo *> &inQueue = getPortQueue(kInputPortIndex);
        List<BufferInfo *> &outQueue = getPortQueue(kOutputPortIndex);

        // Flush any pending split frame payload (after emitting CSD in previous buffer)
        if (!mPendingFrame.empty() && !outQueue.empty()) {
            BufferInfo *outInfo = *outQueue.begin();
            outQueue.erase(outQueue.begin());
            OMX_BUFFERHEADERTYPE *outHeader = outInfo->mHeader;
            size_t copyLen = std::min<size_t>(mPendingFrame.size(), outHeader->nAllocLen);
            memcpy(outHeader->pBuffer + outHeader->nOffset, mPendingFrame.data(), copyLen);
            outHeader->nFilledLen = copyLen;
            outHeader->nTimeStamp = mPendingTimestamp;
            outHeader->nFlags = mPendingFlags;
            mPendingFrame.clear();
            outInfo->mOwnedByUs = false;
            notifyFillBufferDone(outHeader);
        }

        while (!inQueue.empty() && !outQueue.empty()) {
            BufferInfo *inInfo = *inQueue.begin();
            OMX_BUFFERHEADERTYPE *inHeader = inInfo->mHeader;

            if (inHeader->nFilledLen == 0) {
                bool isEos = (inHeader->nFlags & OMX_BUFFERFLAG_EOS) != 0;
                inQueue.erase(inQueue.begin());
                inInfo->mOwnedByUs = false;
                notifyEmptyBufferDone(inHeader);

                if (isEos) {
                    BufferInfo *outInfo = *outQueue.begin();
                    outQueue.erase(outQueue.begin());
                    OMX_BUFFERHEADERTYPE *outHeader = outInfo->mHeader;
                    outHeader->nFilledLen = 0;
                    outHeader->nTimeStamp = inHeader->nTimeStamp;
                    outHeader->nFlags = OMX_BUFFERFLAG_EOS;
                    outInfo->mOwnedByUs = false;
                    notifyFillBufferDone(outHeader);
                }
                return;
            }

            if (!mV4L2Initialized) {
                if (!initV4L2()) {
                    ALOGE("Failed to initialize CODA7542 V4L2 encoder");
                    notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                    mSignalledError = true;
                    return;
                }
            }

            reclaimV4L2OutputBuffers();
            int outIdx = findFreeOutputBuffer();
            if (outIdx < 0) {
                break;
            }

            if (mForceKeyFrame || (inHeader->nFlags & OMX_BUFFERFLAG_SYNCFRAME)) {
                setCtrl(V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1);
                mForceKeyFrame = false;
            }

            // Convert input frame into NV12 in mOutBuffers[outIdx]
            const uint8_t *srcPtr = inHeader->pBuffer + inHeader->nOffset;
            if (!convertInputToNV12(srcPtr, inHeader->nFilledLen,
                                    static_cast<uint8_t *>(mOutBuffers[outIdx].start))) {
                ALOGE("Failed to convert encoder input buffer to NV12");
                notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                mSignalledError = true;
                return;
            }

            struct v4l2_buffer vbuf;
            memset(&vbuf, 0, sizeof(vbuf));
            vbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            vbuf.memory = V4L2_MEMORY_MMAP;
            vbuf.index = outIdx;
            vbuf.bytesused = (mOutStride * mAlignedHeight * 3) / 2;
            vbuf.timestamp.tv_sec = static_cast<time_t>(inHeader->nTimeStamp / 1000000LL);
            vbuf.timestamp.tv_usec = static_cast<suseconds_t>(inHeader->nTimeStamp % 1000000LL);

            if (xioctl(mFd, VIDIOC_QBUF, &vbuf) < 0) {
                ALOGE("Encoder VIDIOC_QBUF(OUTPUT) failed: %s", strerror(errno));
                notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                mSignalledError = true;
                return;
            }
            mOutBuffers[outIdx].queued = true;

            if (!mStreamOnOut) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                if (xioctl(mFd, VIDIOC_STREAMON, &type) < 0) {
                    ALOGE("Encoder VIDIOC_STREAMON(OUTPUT) failed: %s", strerror(errno));
                    notify(OMX_EventError, OMX_ErrorUndefined, 0, nullptr);
                    mSignalledError = true;
                    return;
                }
                mStreamOnOut = true;
            }

            // Wait for CODA7542 to finish encoding the frame (do not include POLLOUT,
            // which is always ready on coda and makes poll() return in 0ms).
            struct pollfd pfd;
            pfd.fd = mFd;
            pfd.events = POLLIN;
            pfd.revents = 0;
            poll(&pfd, 1, 200);

            reclaimV4L2OutputBuffers();

            struct v4l2_buffer capBuf;
            memset(&capBuf, 0, sizeof(capBuf));
            capBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            capBuf.memory = V4L2_MEMORY_MMAP;

            if (xioctl(mFd, VIDIOC_DQBUF, &capBuf) == 0) {
                mCapBuffers[capBuf.index].queued = false;
                if ((capBuf.flags & V4L2_BUF_FLAG_ERROR) != 0) {
                    if (xioctl(mFd, VIDIOC_QBUF, &capBuf) == 0) {
                        mCapBuffers[capBuf.index].queued = true;
                    }
                    inQueue.erase(inQueue.begin());
                    inInfo->mOwnedByUs = false;
                    notifyEmptyBufferDone(inHeader);
                    continue;
                }
                const uint8_t *encData =
                        static_cast<const uint8_t *>(mCapBuffers[capBuf.index].start);
                size_t encLen = capBuf.bytesused;
                bool isKeyFrame = (capBuf.flags & V4L2_BUF_FLAG_KEYFRAME) != 0;
                OMX_TICKS ts = inHeader->nTimeStamp;
                OMX_U32 flags = OMX_BUFFERFLAG_ENDOFFRAME;
                if (isKeyFrame) {
                    flags |= OMX_BUFFERFLAG_SYNCFRAME;
                }
                if (inHeader->nFlags & OMX_BUFFERFLAG_EOS) {
                    flags |= OMX_BUFFERFLAG_EOS;
                }

                // On first frame, split out leading SPS/PPS (or MPEG4 VOL) as CODECCONFIG
                if (!mSentCodecConfig && encLen > 0) {
                    std::vector<uint8_t> csd;
                    size_t sliceOffset = extractCodecConfig(encData, encLen, &csd);
                    if (!csd.empty()) {
                        mSentCodecConfig = true;
                        BufferInfo *csdInfo = *outQueue.begin();
                        outQueue.erase(outQueue.begin());
                        OMX_BUFFERHEADERTYPE *csdHeader = csdInfo->mHeader;
                        size_t csdCopy = std::min<size_t>(csd.size(), csdHeader->nAllocLen);
                        memcpy(csdHeader->pBuffer + csdHeader->nOffset, csd.data(), csdCopy);
                        csdHeader->nFilledLen = csdCopy;
                        csdHeader->nTimeStamp = ts;
                        csdHeader->nFlags = OMX_BUFFERFLAG_CODECCONFIG | OMX_BUFFERFLAG_ENDOFFRAME;
                        csdInfo->mOwnedByUs = false;
                        notifyFillBufferDone(csdHeader);

                        encData += sliceOffset;
                        encLen -= sliceOffset;
                    }
                }

                if (!outQueue.empty()) {
                    BufferInfo *outInfo = *outQueue.begin();
                    outQueue.erase(outQueue.begin());
                    OMX_BUFFERHEADERTYPE *outHeader = outInfo->mHeader;
                    size_t copyLen = std::min<size_t>(encLen, outHeader->nAllocLen);
                    memcpy(outHeader->pBuffer + outHeader->nOffset, encData, copyLen);
                    outHeader->nFilledLen = copyLen;
                    outHeader->nTimeStamp = ts;
                    outHeader->nFlags = flags;
                    outInfo->mOwnedByUs = false;
                    notifyFillBufferDone(outHeader);
                } else if (encLen > 0) {
                    mPendingFrame.assign(encData, encData + encLen);
                    mPendingTimestamp = ts;
                    mPendingFlags = flags;
                }

                if (xioctl(mFd, VIDIOC_QBUF, &capBuf) == 0) {
                    mCapBuffers[capBuf.index].queued = true;
                }
            }

            inQueue.erase(inQueue.begin());
            inInfo->mOwnedByUs = false;
            notifyEmptyBufferDone(inHeader);
        }
    }

    void onReset() override {
        SimpleSoftOMXComponent::onReset();
        stopAndCloseV4L2();
        mSignalledError = false;
    }

private:
    static constexpr uint32_t kNumBuffers = 4;

    uint32_t mV4L2PixFmt;
    int mFd;
    bool mV4L2Initialized;
    bool mStreamOnOut;
    bool mStreamOnCap;
    bool mSentCodecConfig;
    bool mForceKeyFrame;
    bool mSignalledError;
    uint32_t mGopSize;
    uint32_t mAlignedWidth;
    uint32_t mAlignedHeight;
    size_t mOutStride;
    std::vector<MmapBuffer> mOutBuffers;
    std::vector<MmapBuffer> mCapBuffers;
    std::vector<uint8_t> mScratchI420;
    std::vector<uint8_t> mPendingFrame;
    OMX_TICKS mPendingTimestamp = 0;
    OMX_U32 mPendingFlags = 0;

    void setCtrl(uint32_t id, int32_t value) {
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.id = id;
        ctrl.value = value;
        xioctl(mFd, VIDIOC_S_CTRL, &ctrl);
    }

    bool initV4L2() {
        stopAndCloseV4L2();

        mFd = openCodaDevice(true /* isEncoder */, mV4L2PixFmt);
        if (mFd < 0) {
            return false;
        }

        mAlignedWidth = alignUp(mWidth, 16);
        mAlignedHeight = alignUp(mHeight, 16);

        // Configure encoder controls (docs/VPU.md quirk #5: bitrate floor & GOP >= 60)
        uint32_t targetBitrate = std::max<uint32_t>(mBitrate, 512000u);
        setCtrl(V4L2_CID_MPEG_VIDEO_BITRATE, static_cast<int32_t>(targetBitrate));
        setCtrl(V4L2_CID_MPEG_VIDEO_GOP_SIZE, static_cast<int32_t>(std::max<uint32_t>(mGopSize, 30u)));
        if (mV4L2PixFmt == V4L2_PIX_FMT_H264) {
            setCtrl(V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
            setCtrl(V4L2_CID_MPEG_VIDEO_H264_LEVEL, V4L2_MPEG_VIDEO_H264_LEVEL_3_1);
        }

        // Configure OUTPUT queue as V4L2_PIX_FMT_NV12 (docs/VPU.md quirk #4)
        struct v4l2_format fmtOut;
        memset(&fmtOut, 0, sizeof(fmtOut));
        fmtOut.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmtOut.fmt.pix.width = mAlignedWidth;
        fmtOut.fmt.pix.height = mAlignedHeight;
        fmtOut.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmtOut.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(mFd, VIDIOC_S_FMT, &fmtOut) < 0) {
            ALOGE("Encoder VIDIOC_S_FMT(OUTPUT) failed: %s", strerror(errno));
            stopAndCloseV4L2();
            return false;
        }
        mOutStride = fmtOut.fmt.pix.bytesperline ? fmtOut.fmt.pix.bytesperline : mAlignedWidth;

        // If visible dimensions differ from 16-aligned dimensions, set crop rectangle
        if (static_cast<uint32_t>(mWidth) != mAlignedWidth ||
            static_cast<uint32_t>(mHeight) != mAlignedHeight) {
            struct v4l2_selection sel;
            memset(&sel, 0, sizeof(sel));
            sel.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            sel.target = V4L2_SEL_TGT_CROP;
            sel.r.left = 0;
            sel.r.top = 0;
            sel.r.width = mWidth;
            sel.r.height = mHeight;
            xioctl(mFd, VIDIOC_S_SELECTION, &sel);
        }

        // Configure CAPTURE queue (encoded H.264 / MPEG-4 bitstream)
        struct v4l2_format fmtCap;
        memset(&fmtCap, 0, sizeof(fmtCap));
        fmtCap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmtCap.fmt.pix.width = mAlignedWidth;
        fmtCap.fmt.pix.height = mAlignedHeight;
        fmtCap.fmt.pix.pixelformat = mV4L2PixFmt;
        fmtCap.fmt.pix.sizeimage = 512 * 1024;
        fmtCap.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(mFd, VIDIOC_S_FMT, &fmtCap) < 0) {
            ALOGE("Encoder VIDIOC_S_FMT(CAPTURE) failed: %s", strerror(errno));
            stopAndCloseV4L2();
            return false;
        }

        uint32_t fps = (mFramerate >> 16) ? (mFramerate >> 16) : 30;
        struct v4l2_streamparm parm;
        memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        parm.parm.output.timeperframe.numerator = 1;
        parm.parm.output.timeperframe.denominator = fps;
        xioctl(mFd, VIDIOC_S_PARM, &parm);

        if (!allocateMmapBuffers(V4L2_BUF_TYPE_VIDEO_OUTPUT, kNumBuffers, &mOutBuffers) ||
            !allocateMmapBuffers(V4L2_BUF_TYPE_VIDEO_CAPTURE, kNumBuffers, &mCapBuffers)) {
            stopAndCloseV4L2();
            return false;
        }

        for (size_t i = 0; i < mCapBuffers.size(); ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (xioctl(mFd, VIDIOC_QBUF, &buf) < 0) {
                stopAndCloseV4L2();
                return false;
            }
            mCapBuffers[i].queued = true;
        }

        enum v4l2_buf_type capType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(mFd, VIDIOC_STREAMON, &capType) < 0) {
            ALOGE("Encoder VIDIOC_STREAMON(CAPTURE) failed: %s", strerror(errno));
            stopAndCloseV4L2();
            return false;
        }
        mStreamOnCap = true;
        mV4L2Initialized = true;
        mSentCodecConfig = false;

        ALOGI("Initialized CODA7542 encoder: %dx%d (aligned %ux%u, stride=%zu, bitrate=%u)",
              mWidth, mHeight, mAlignedWidth, mAlignedHeight, mOutStride, targetBitrate);
        return true;
    }

    bool allocateMmapBuffers(enum v4l2_buf_type type, uint32_t count,
                             std::vector<MmapBuffer> *buffers) {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = count;
        req.type = type;
        req.memory = V4L2_MEMORY_MMAP;
        if (xioctl(mFd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
            return false;
        }
        buffers->resize(req.count);
        for (uint32_t i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = type;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (xioctl(mFd, VIDIOC_QUERYBUF, &buf) < 0) {
                return false;
            }
            void *ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, mFd, buf.m.offset);
            if (ptr == MAP_FAILED) {
                return false;
            }
            (*buffers)[i].start = ptr;
            (*buffers)[i].length = buf.length;
            (*buffers)[i].queued = false;
        }
        return true;
    }

    int findFreeOutputBuffer() const {
        for (size_t i = 0; i < mOutBuffers.size(); ++i) {
            if (!mOutBuffers[i].queued) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    void reclaimV4L2OutputBuffers() {
        if (!mV4L2Initialized || !mStreamOnOut) {
            return;
        }
        for (;;) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_MMAP;
            if (xioctl(mFd, VIDIOC_DQBUF, &buf) < 0) {
                break;
            }
            if (buf.index < mOutBuffers.size()) {
                mOutBuffers[buf.index].queued = false;
            }
        }
    }

    bool convertInputToNV12(const uint8_t *src, size_t srcLen, uint8_t *dstNV12) {
        size_t w = static_cast<size_t>(mWidth);
        size_t h = static_cast<size_t>(mHeight);
        uint8_t *dstY = dstNV12;
        uint8_t *dstUV = dstNV12 + mOutStride * mAlignedHeight;

        if (mInputDataIsMeta) {
            size_t i420Size = w * h * 3 / 2;
            if (mScratchI420.size() < i420Size) {
                mScratchI420.resize(i420Size);
            }
            const uint8_t *planar = extractGraphicBuffer(
                    mScratchI420.data(), mScratchI420.size(), src, srcLen, w, h);
            if (!planar) {
                return false;
            }
            copyI420ToNV12(planar, w, h, dstY, dstUV);
            return true;
        }

        if (srcLen < (w * h * 3) / 2) {
            return false;
        }

        if (mColorFormat == OMX_COLOR_FormatYUV420SemiPlanar) {
            // Input is already NV12 (w x h)
            const uint8_t *srcY = src;
            const uint8_t *srcUV = src + w * h;
            for (size_t y = 0; y < h; ++y) {
                memcpy(dstY + y * mOutStride, srcY + y * w, w);
            }
            for (size_t y = 0; y < h / 2; ++y) {
                memcpy(dstUV + y * mOutStride, srcUV + y * w, w);
            }
            return true;
        }

        // Input is I420 (OMX_COLOR_FormatYUV420Planar)
        copyI420ToNV12(src, w, h, dstY, dstUV);
        return true;
    }

    void copyI420ToNV12(const uint8_t *srcI420, size_t w, size_t h,
                        uint8_t *dstY, uint8_t *dstUV) const {
        const uint8_t *srcY = srcI420;
        const uint8_t *srcU = srcI420 + w * h;
        const uint8_t *srcV = srcU + (w * h) / 4;

        for (size_t y = 0; y < h; ++y) {
            memcpy(dstY + y * mOutStride, srcY + y * w, w);
        }
        for (size_t y = 0; y < h / 2; ++y) {
            uint8_t *rowUV = dstUV + y * mOutStride;
            const uint8_t *rowU = srcU + y * (w / 2);
            const uint8_t *rowV = srcV + y * (w / 2);
            for (size_t x = 0; x < w / 2; ++x) {
                rowUV[2 * x] = rowU[x];
                rowUV[2 * x + 1] = rowV[x];
            }
        }
    }

    /*
     * Extract leading SPS/PPS (H.264) or VOS/VIS/VOL (MPEG-4) from the first
     * encoded CAPTURE buffer and return the byte offset where the first picture
     * slice starts.
     */
    size_t extractCodecConfig(const uint8_t *data, size_t size,
                              std::vector<uint8_t> *csdOut) const {
        if (mV4L2PixFmt == V4L2_PIX_FMT_H264) {
            // Collect NALs of type 7 (SPS) and 8 (PPS), skip type 12 (filler),
            // stop at type 5 (IDR) or type 1 (non-IDR).
            struct NalSpan {
                size_t scStart;
                size_t nalStart;
                size_t end;
                uint8_t type;
            };
            std::vector<NalSpan> nals;
            for (size_t i = 0; i + 4 <= size; ++i) {
                size_t scLen = 0;
                if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
                    scLen = 3;
                } else if (i + 4 < size && data[i] == 0 && data[i + 1] == 0 &&
                           data[i + 2] == 0 && data[i + 3] == 1) {
                    scLen = 4;
                }
                if (scLen > 0 && i + scLen < size) {
                    if (!nals.empty()) {
                        nals.back().end = i;
                    }
                    NalSpan ns;
                    ns.scStart = i;
                    ns.nalStart = i + scLen;
                    ns.end = size;
                    ns.type = data[i + scLen] & 0x1F;
                    nals.push_back(ns);
                    i += scLen - 1;
                }
            }
            size_t sliceStart = 0;
            for (const auto &ns : nals) {
                if (ns.type == 7 || ns.type == 8) {
                    csdOut->insert(csdOut->end(), data + ns.scStart, data + ns.end);
                    sliceStart = ns.end;
                } else if (ns.type == 12) {
                    // Skip coda_h264_padding filler NAL
                    sliceStart = ns.end;
                } else if (ns.type == 5 || ns.type == 1) {
                    sliceStart = ns.scStart;
                    break;
                }
            }
            return sliceStart;
        } else if (mV4L2PixFmt == V4L2_PIX_FMT_MPEG4) {
            // Everything before first VOP start code (00 00 01 B6) is VOS/VIS/VOL
            for (size_t i = 0; i + 4 <= size; ++i) {
                if (data[i] == 0 && data[i + 1] == 0 &&
                    data[i + 2] == 1 && data[i + 3] == 0xB6) {
                    if (i > 0) {
                        csdOut->insert(csdOut->end(), data, data + i);
                    }
                    return i;
                }
            }
        }
        return 0;
    }

    void stopAndCloseV4L2() {
        if (mFd >= 0) {
            if (mStreamOnOut) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                xioctl(mFd, VIDIOC_STREAMOFF, &type);
                mStreamOnOut = false;
            }
            if (mStreamOnCap) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                xioctl(mFd, VIDIOC_STREAMOFF, &type);
                mStreamOnCap = false;
            }
        }
        for (auto &b : mOutBuffers) {
            if (b.start && b.start != MAP_FAILED) {
                munmap(b.start, b.length);
            }
        }
        mOutBuffers.clear();
        for (auto &b : mCapBuffers) {
            if (b.start && b.start != MAP_FAILED) {
                munmap(b.start, b.length);
            }
        }
        mCapBuffers.clear();
        if (mFd >= 0) {
            close(mFd);
            mFd = -1;
        }
        mV4L2Initialized = false;
        mSentCodecConfig = false;
        mPendingFrame.clear();
    }

    DISALLOW_EVIL_CONSTRUCTORS(CodaV4L2Encoder);
};

// ============================================================================
// CodaOMXPlugin (OMXPluginBase entry point for libstagefrighthw.so)
// ============================================================================
struct CodaComponentEntry {
    const char *mName;
    const char *mRole;
    bool mIsEncoder;
    OMX_VIDEO_CODINGTYPE mCodingType;
    uint32_t mV4L2PixFmt;
    const char *mMimeType;
};

static const CodaComponentEntry kComponents[] = {
    {"OMX.marvell.video.decoder.avc", "video_decoder.avc", false,
     OMX_VIDEO_CodingAVC, V4L2_PIX_FMT_H264, MEDIA_MIMETYPE_VIDEO_AVC},
    {"OMX.marvell.video.decoder.mpeg4", "video_decoder.mpeg4", false,
     OMX_VIDEO_CodingMPEG4, V4L2_PIX_FMT_MPEG4, MEDIA_MIMETYPE_VIDEO_MPEG4},
    {"OMX.marvell.video.decoder.mpeg2", "video_decoder.mpeg2", false,
     OMX_VIDEO_CodingMPEG2, V4L2_PIX_FMT_MPEG2, MEDIA_MIMETYPE_VIDEO_MPEG2},
    {"OMX.marvell.video.encoder.avc", "video_encoder.avc", true,
     OMX_VIDEO_CodingAVC, V4L2_PIX_FMT_H264, MEDIA_MIMETYPE_VIDEO_AVC},
    {"OMX.marvell.video.encoder.mpeg4", "video_encoder.mpeg4", true,
     OMX_VIDEO_CodingMPEG4, V4L2_PIX_FMT_MPEG4, MEDIA_MIMETYPE_VIDEO_MPEG4},
};

static constexpr size_t kNumComponents = sizeof(kComponents) / sizeof(kComponents[0]);

class CodaOMXPlugin : public OMXPluginBase {
public:
    CodaOMXPlugin() = default;
    virtual ~CodaOMXPlugin() = default;

    OMX_ERRORTYPE makeComponentInstance(const char *name,
                                        const OMX_CALLBACKTYPE *callbacks,
                                        OMX_PTR appData,
                                        OMX_COMPONENTTYPE **component) override {
        for (size_t i = 0; i < kNumComponents; ++i) {
            const auto &entry = kComponents[i];
            if (strcmp(name, entry.mName) != 0) {
                continue;
            }

            sp<SoftOMXComponent> codec;
            if (!entry.mIsEncoder) {
                const CodecProfileLevel *profiles = kAvcDecProfileLevels;
                size_t numProfiles = sizeof(kAvcDecProfileLevels) / sizeof(kAvcDecProfileLevels[0]);
                if (entry.mV4L2PixFmt == V4L2_PIX_FMT_MPEG4) {
                    profiles = kMpeg4DecProfileLevels;
                    numProfiles = sizeof(kMpeg4DecProfileLevels) / sizeof(kMpeg4DecProfileLevels[0]);
                } else if (entry.mV4L2PixFmt == V4L2_PIX_FMT_MPEG2) {
                    profiles = kMpeg2DecProfileLevels;
                    numProfiles = sizeof(kMpeg2DecProfileLevels) / sizeof(kMpeg2DecProfileLevels[0]);
                }
                codec = new CodaV4L2Decoder(entry.mName, entry.mRole, entry.mCodingType,
                                            entry.mV4L2PixFmt, entry.mMimeType,
                                            profiles, numProfiles,
                                            callbacks, appData, component);
            } else {
                const CodecProfileLevel *profiles = kAvcEncProfileLevels;
                size_t numProfiles = sizeof(kAvcEncProfileLevels) / sizeof(kAvcEncProfileLevels[0]);
                if (entry.mV4L2PixFmt == V4L2_PIX_FMT_MPEG4) {
                    profiles = kMpeg4EncProfileLevels;
                    numProfiles = sizeof(kMpeg4EncProfileLevels) / sizeof(kMpeg4EncProfileLevels[0]);
                }
                codec = new CodaV4L2Encoder(entry.mName, entry.mRole, entry.mCodingType,
                                            entry.mV4L2PixFmt, entry.mMimeType,
                                            profiles, numProfiles,
                                            callbacks, appData, component);
            }

            OMX_ERRORTYPE err = codec->initCheck();
            if (err != OMX_ErrorNone) {
                return err;
            }
            codec->incStrong(this);
            return OMX_ErrorNone;
        }
        return OMX_ErrorInvalidComponentName;
    }

    OMX_ERRORTYPE destroyComponentInstance(OMX_COMPONENTTYPE *component) override {
        auto *me = static_cast<SimpleSoftOMXComponent *>(component->pComponentPrivate);
        me->prepareForDestruction();
        CHECK_EQ(me->getStrongCount(), 1);
        me->decStrong(this);
        return OMX_ErrorNone;
    }

    OMX_ERRORTYPE enumerateComponents(OMX_STRING name, size_t size, OMX_U32 index) override {
        if (index >= kNumComponents) {
            return OMX_ErrorNoMore;
        }
        snprintf(name, size, "%s", kComponents[index].mName);
        return OMX_ErrorNone;
    }

    OMX_ERRORTYPE getRolesOfComponent(const char *name, Vector<String8> *roles) override {
        for (size_t i = 0; i < kNumComponents; ++i) {
            if (strcmp(name, kComponents[i].mName) == 0) {
                roles->clear();
                roles->push(String8(kComponents[i].mRole));
                return OMX_ErrorNone;
            }
        }
        return OMX_ErrorInvalidComponentName;
    }
};

OMXPluginBase *createOMXPlugin() {
    return new CodaOMXPlugin();
}

} // namespace android

extern "C" void destroyOMXPlugin(android::OMXPluginBase *plugin) {
    delete plugin;
}
