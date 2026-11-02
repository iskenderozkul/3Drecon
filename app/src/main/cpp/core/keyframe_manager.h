#pragma once

#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

#include "core/keyframe.h"

namespace recon {

// Sliding-window keyframe pool. The pool keeps a bounded set of keyframes so
// the reconstruction never loses its global spatial memory while still only
// evaluating a few reference views per frame.
class KeyframeManager {
public:
    explicit KeyframeManager(size_t maxPool = 14) : maxPool_(maxPool) {}

    bool shouldAdd(const Keyframe& candidate, double dtSeconds) const;
    void add(Keyframe&& kf);

    // Fills refs with (keyframe, cost weight) sorted by descending reference
    // score S = wb * wv (paper Eqs. 1-3); the weights are normalised to sum 1
    // as required by the multi-view cost in Eq. (7).
    void selectReferences(const Keyframe& target, int maxRefs,
                          std::vector<std::pair<const Keyframe*, float>>& refs) const;

    size_t size() const { return pool_.size(); }
    void clear() { pool_.clear(); }
    bool empty() const { return pool_.empty(); }

private:
    float baselineScore(const Keyframe& a, const Keyframe& b) const;
    float angleScore(const Keyframe& a, const Keyframe& b) const;

    std::deque<Keyframe> pool_;
    size_t maxPool_;
};

}  // namespace recon
