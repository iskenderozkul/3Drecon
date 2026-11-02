#include "core/keyframe_manager.h"

#include <algorithm>
#include <cmath>

namespace recon {
namespace {
// A keyframe needs real parallax: with less than ~10 cm of baseline the stereo
// depth error grows quadratically with range and turns into volume noise.
constexpr float kMinBaseline = 0.15f;  // metres
constexpr float kMinRotationDeg = 8.f;
// Fusing a keyframe costs 50-300 ms once the volume is large, and ARCore depth
// is noisy per frame: only a genuinely new viewpoint (not the same wall a few
// centimetres later) should pay that price.
constexpr double kMinIntervalSeconds = 0.25;
// Eq. (1)-(3): baseline preference around 0.6 m with sigma = 0.2, and a
// viewing score that rewards overlap (angles below 10 degrees).
constexpr float kBaselineMean = 0.6f;
constexpr float kBaselineSigma = 0.2f;
constexpr float kReferenceAngleDeg = 10.f;
// A reference looking more than 75 degrees away shares almost no pixels, so
// its cost volume is pure uninformative matching noise.
constexpr float kMaxReferenceAngleDeg = 75.f;
}  // namespace

float KeyframeManager::baselineScore(const Keyframe& a, const Keyframe& b) const {
    const float baseline = (a.center() - b.center()).length();
    const float delta = (baseline - kBaselineMean) / kBaselineSigma;
    return std::exp(-delta * delta);
}

float KeyframeManager::angleScore(const Keyframe& a, const Keyframe& b) const {
    const float c = clampf(a.viewDir().dot(b.viewDir()), -1.f, 1.f);
    const float angleDeg = std::acos(c) * 180.f / kPi;
    return std::max(kReferenceAngleDeg / std::max(angleDeg, 0.25f), 1.f);
}

bool KeyframeManager::shouldAdd(const Keyframe& candidate, double dtSeconds) const {
    if (pool_.empty()) return true;
    if (dtSeconds < kMinIntervalSeconds) return false;
    const Keyframe& last = pool_.back();
    const float baseline = (last.center() - candidate.center()).length();
    const float cosAngle = clampf(last.viewDir().dot(candidate.viewDir()), -1.f, 1.f);
    const float angleDeg = std::acos(cosAngle) * 180.f / kPi;
    return baseline >= kMinBaseline || angleDeg >= kMinRotationDeg;
}

void KeyframeManager::add(Keyframe&& kf) {
    if (pool_.size() >= maxPool_) pool_.pop_front();
    pool_.push_back(std::move(kf));
}

void KeyframeManager::selectReferences(
    const Keyframe& target, int maxRefs,
    std::vector<std::pair<const Keyframe*, float>>& refs) const {
    refs.clear();
    std::vector<std::pair<const Keyframe*, float>> candidates;
    candidates.reserve(pool_.size());
    for (const Keyframe& kf : pool_) {
        if (kf.id == target.id) continue;
        const float c = clampf(target.viewDir().dot(kf.viewDir()), -1.f, 1.f);
        const float angleDeg = std::acos(c) * 180.f / kPi;
        if (angleDeg > kMaxReferenceAngleDeg) continue;
        const float score = baselineScore(target, kf) * angleScore(target, kf);
        if (score < 1e-3f) continue;
        candidates.emplace_back(&kf, score);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    if (static_cast<int>(candidates.size()) > maxRefs) candidates.resize(maxRefs);

    float sum = 0.f;
    for (const auto& entry : candidates) sum += entry.second;
    if (sum > 0.f) {
        for (auto& entry : candidates) entry.second /= sum;  // w_t' in Eq. (7)
    }
    refs = std::move(candidates);
}

}  // namespace recon
