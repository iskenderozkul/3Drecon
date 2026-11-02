#pragma once

#include <cstdint>

#include "mvs/stereo.h"

namespace recon {

// Image-guided, edge-preserving depth refinement worker. It removes
// single-pixel SGM outliers (median) and then smooths each depth sample with a
// joint bilateral kernel driven by the luminance guide, so planar surfaces are
// flattened while depth discontinuities (silhouettes) stay sharp.
class DepthRefiner {
public:
    void refine(const uint8_t* luma, int w, int h, DepthResult& depth);

private:
    std::vector<float> median_;
    std::vector<float> scratch_;
};

}  // namespace recon
