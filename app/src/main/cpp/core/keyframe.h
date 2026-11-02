#pragma once

#include <cstdint>
#include <vector>

#include "math/math.h"

namespace recon {

// A keyframe owns the rectified (half-resolution) luminance + color image,
// its 64-bit Census descriptors, and the world pose captured by ARCore.
struct Keyframe {
    int id = 0;
    int64_t timestampNs = 0;
    int w = 0;
    int h = 0;
    std::vector<uint8_t> luma;
    std::vector<uint8_t> rgb;
    std::vector<uint64_t> census;
    // Depth computed when this keyframe was the target (empty if stereo was not
    // available yet). Later keyframes reproject into it for the consistency
    // check that kills false matches.
    std::vector<float> depthMap;
    Mat3 R_gl;   // camera -> world, OpenGL camera convention
    Vec3 t;      // camera position in world (ARCore world frame)
    Intrinsics K;

    Vec3 center() const { return t; }
    Vec3 viewDir() const { return R_gl * Vec3{0.f, 0.f, -1.f}; }
    Mat3 R_cv() const { return arcoreToCvRotation(R_gl); }
    Vec3 toWorld(const Vec3& pCv) const { return R_cv() * pCv + t; }
};

}  // namespace recon
