#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "core/keyframe.h"

namespace recon {

struct DepthResult {
    int w = 0;
    int h = 0;
    std::vector<float> depth;       // metres, 0 == invalid
    std::vector<float> confidence;  // [0,1]

    void resize(int width, int height) {
        w = width;
        h = height;
        depth.assign(static_cast<size_t>(w) * h, 0.f);
        confidence.assign(static_cast<size_t>(w) * h, 0.f);
    }
    void clear() {
        depth.assign(depth.size(), 0.f);
        confidence.assign(confidence.size(), 0.f);
    }
};

// Dense multi-view stereo:
//   inverse-depth plane sampling -> weighted multi-view Census cost volume ->
//   8-path SGM with a gradient adaptive P2 -> parabolic sub-pixel refinement ->
//   paper's confidence filtering (Eqs. 12-14): directional SGM uncertainty
//   weighted by the 5x5 sub-level label disagreement and normalised per frame.
class StereoEngine {
public:
    static constexpr int kLevels = 96;
    static constexpr int kDirections = 8;
    static constexpr float kMinConfidence = 0.15f;

    StereoEngine() = default;

    bool compute(const Keyframe& target,
                 const std::vector<std::pair<const Keyframe*, float>>& refs,
                 DepthResult& out);

private:
    struct RefProjector {
        Mat3 M;     // K1 * R1^T * R0 * K0^-1
        Vec3 e;     // K1 * R1^T * (t0 - t1)
        const Keyframe* kf = nullptr;
        float weight = 1.f;
    };

    void allocate(int w, int h);
    void computeGradient(const Keyframe& target, int w, int h);

    std::vector<RefProjector> projectors_;
    std::vector<float> planeDepth_;
    std::vector<float> planeInvDepth_;
    std::vector<float> planeStep_;

    std::vector<uint8_t> cost_;
    std::vector<float> aggregated_;
    std::vector<float> lr_;
    std::vector<float> gradient_;
    std::vector<float> rawDepth_;
    std::vector<float> dirTerm_;     // Sum_r min_l (C_r(x,l) - lambda * C(x,l))
    std::vector<float> labelSub_;    // Sub-level winner-take-all label l_s(x)
    std::vector<float> uncertainty_; // U(x) of Eq. (12)

    int w_ = 0;
    int h_ = 0;
    float near_ = 0.30f;
    float far_ = 3.0f;
};

}  // namespace recon
