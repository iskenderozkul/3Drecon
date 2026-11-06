#pragma once

#include <cstdint>
#include <vector>

#include "math/math.h"

namespace recon {

// One live camera frame delivered from ARCore (CPU YUV_420_888) plus the
// matching 6DoF pose and pinhole intrinsics. Nothing here is synthesized.
struct FrameInput {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> y;   // de-strided luma, width * height
    std::vector<uint8_t> u;   // de-strided chroma, (width/2) * (height/2)
    std::vector<uint8_t> v;   // de-strided chroma, (width/2) * (height/2)
    int64_t timestampNs = 0;
    float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;
    float tx = 0.f, ty = 0.f, tz = 0.f;
    Intrinsics K;             // full-resolution image intrinsics
    bool tracking = false;

    // Optional depth delivered by ARCore's Depth API (millimetres, aligned to
    // the camera image). Empty when the device/session does not provide it; the
    // engine then falls back to its own multi-view stereo.
    std::vector<uint16_t> depthMm;
    int depthWidth = 0;
    int depthHeight = 0;
    // ARCore pairs the depth image with the camera frame it was computed from;
    // a timestamp that stops advancing identifies a republished stale map.
    int64_t depthTimestampNs = 0;
    // Session runs with Config.DepthMode.AUTOMATIC: ARCore depth is expected on
    // every frame, so a missing image is a transient failure to skip, not a
    // reason to substitute a different (weaker) depth source.
    bool depthApiActive = false;
    // ARCore raw depth is paired with a confidence image (8-bit, same grid).
    // Flying pixels on silhouettes and values smoothed across a depth step are
    // exactly the low-confidence ones, so they are dropped before fusion.
    std::vector<uint8_t> confidence;
    int confidenceWidth = 0;
    int confidenceHeight = 0;
    // ARCore's smoothed depth map (acquireDepthImage16Bits), same grid and
    // timestamp as the raw one. Raw depth carries the sharp surface but also
    // the per-pixel noise and the ramps ARCore blends across a silhouette;
    // the smoothed map is the same surface with those filtered out. It is not
    // trusted on its own (it bridges small depth steps), only where it agrees
    // with a confidence-gated raw sample.
    std::vector<uint16_t> smoothDepthMm;
    int smoothWidth = 0;
    int smoothHeight = 0;
    // CPU-image normalised (x,y) -> depth/texture normalised (u,v):
    // u = m[0]*x + m[1]*y + m[2], v = m[3]*x + m[4]*y + m[5].
    // ARCore crops its depth map to the display aspect ratio, so the depth map
    // does not cover the full 4:3 camera image on many phones.
    float depthTransform[6] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f};
    // Intrinsics of the depth grid itself: ARCore depth is aligned to the GPU
    // texture, not to the CPU image, so these come from Camera.getTextureIntrinsics
    // scaled to the depth resolution. Unprojecting with the CPU intrinsics is
    // what stretched the geometry before.
    Intrinsics Kdepth;

    bool hasDepth() const {
        return depthWidth > 1 && depthHeight > 1 &&
               static_cast<int>(depthMm.size()) == depthWidth * depthHeight;
    }

    bool hasConfidence() const {
        return confidenceWidth == depthWidth && confidenceHeight == depthHeight &&
               static_cast<int>(confidence.size()) == depthWidth * depthHeight;
    }

    bool hasSmoothDepth() const {
        return smoothWidth == depthWidth && smoothHeight == depthHeight &&
               static_cast<int>(smoothDepthMm.size()) == depthWidth * depthHeight;
    }

    bool valid() const {
        return width > 1 && height > 1 &&
               static_cast<int>(y.size()) == width * height &&
               static_cast<int>(u.size()) == (width / 2) * (height / 2) &&
               static_cast<int>(v.size()) == (width / 2) * (height / 2);
    }
};

}  // namespace recon
