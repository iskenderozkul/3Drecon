#include "mvs/refine.h"

#include <algorithm>
#include <cmath>

namespace recon {

void DepthRefiner::refine(const uint8_t* luma, int w, int h, DepthResult& depth) {
    if (w < 5 || h < 5) return;
    const size_t n = static_cast<size_t>(w) * h;
    median_.assign(n, 0.f);
    scratch_.assign(n, 0.f);

    // 3x3 median of valid neighbours suppresses speckle noise.
    float window[9];
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const size_t p = static_cast<size_t>(y) * w + x;
            const float center = depth.depth[p];
            if (center <= 0.f) continue;
            int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const float v = depth.depth[static_cast<size_t>(y + dy) * w + (x + dx)];
                    if (v > 0.f && std::fabs(v - center) < 0.35f * center) {
                        window[count++] = v;
                    }
                }
            }
            if (count >= 3) {
                std::sort(window, window + count);
                median_[p] = window[count / 2];
            } else {
                median_[p] = center;
            }
        }
    }
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t p = static_cast<size_t>(y) * w + x;
            if (depth.depth[p] > 0.f && median_[p] > 0.f) depth.depth[p] = median_[p];
        }
    }

    // Joint bilateral pass (5x5), range weights both in depth and in luma.
    const float sigmaSpatial2 = 2.f * 1.6f * 1.6f;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t p = static_cast<size_t>(y) * w + x;
            const float center = depth.depth[p];
            if (center <= 0.f) {
                scratch_[p] = 0.f;
                continue;
            }
            const float guide0 = static_cast<float>(luma[p]) / 255.f;
            const float sigmaDepth = 0.025f * center + 0.004f;
            float sum = 0.f;
            float weightSum = 0.f;
            for (int dy = -2; dy <= 2; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -2; dx <= 2; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    const size_t q = static_cast<size_t>(yy) * w + xx;
                    const float nd = depth.depth[q];
                    if (nd <= 0.f) continue;
                    const float dd = nd - center;
                    const float depthW = 1.f / (1.f + (dd / sigmaDepth) * (dd / sigmaDepth));
                    const float guide1 = static_cast<float>(luma[q]) / 255.f;
                    const float dg = guide1 - guide0;
                    const float lumaW = 1.f / (1.f + (dg / 0.045f) * (dg / 0.045f));
                    const float spatialW = std::exp(-static_cast<float>(dx * dx + dy * dy) / sigmaSpatial2);
                    const float weight = depthW * lumaW * spatialW;
                    sum += nd * weight;
                    weightSum += weight;
                }
            }
            scratch_[p] = weightSum > 1e-6f ? sum / weightSum : center;
        }
    }

    for (size_t p = 0; p < n; ++p) {
        if (depth.depth[p] > 0.f) depth.depth[p] = scratch_[p];
    }
}

}  // namespace recon
