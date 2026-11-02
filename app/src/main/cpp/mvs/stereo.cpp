#include "mvs/stereo.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mvs/census.h"

namespace recon {
namespace {

inline Mat3 intrinsicsMatrix(const Intrinsics& k) {
    Mat3 m;
    m.m[0] = k.fx;
    m.m[1] = 0.f;
    m.m[2] = k.cx;
    m.m[3] = 0.f;
    m.m[4] = k.fy;
    m.m[5] = k.cy;
    m.m[6] = 0.f;
    m.m[7] = 0.f;
    m.m[8] = 1.f;
    return m;
}

inline Mat3 intrinsicsInverse(const Intrinsics& k) {
    Mat3 m;
    m.m[0] = 1.f / k.fx;
    m.m[1] = 0.f;
    m.m[2] = -k.cx / k.fx;
    m.m[3] = 0.f;
    m.m[4] = 1.f / k.fy;
    m.m[5] = -k.cy / k.fy;
    m.m[6] = 0.f;
    m.m[7] = 0.f;
    m.m[8] = 1.f;
    return m;
}

constexpr float kP1 = 9.f;
constexpr float kP2Base = 40.f;
constexpr float kP2Gradient = 0.45f;
// Raw 64-bit census Hamming distance above which the winning plane is treated
// as an ambiguous / occluded match. Correct matches typically score 5-15.
constexpr float kMaxMatchCost = 22.f;

}  // namespace

void StereoEngine::allocate(int w, int h) {
    if (w == w_ && h == h_) return;
    w_ = w;
    h_ = h;
    const size_t n = static_cast<size_t>(w) * h;
    cost_.assign(n * kLevels, 64);
    aggregated_.assign(n * kLevels, 0.f);
    lr_.assign(n * kLevels, 0.f);
    gradient_.assign(n, 0.f);
    rawDepth_.assign(n, 0.f);
    dirTerm_.assign(n, 0.f);
    labelSub_.assign(n, 0.f);
    uncertainty_.assign(n, 0.f);

    planeDepth_.resize(kLevels);
    planeInvDepth_.resize(kLevels);
    planeStep_.resize(kLevels);
    const float invNear = 1.f / near_;
    const float invFar = 1.f / far_;
    const float step = (invFar - invNear) / static_cast<float>(kLevels - 1);
    for (int l = 0; l < kLevels; ++l) {
        const float inv = invNear + step * static_cast<float>(l);
        planeInvDepth_[l] = inv;
        planeDepth_[l] = 1.f / inv;
        planeStep_[l] = step;
    }
}

void StereoEngine::computeGradient(const Keyframe& target, int w, int h) {
    const uint8_t* img = target.luma.data();
    for (int y = 0; y < h; ++y) {
        const int yc = y < 1 ? 0 : (y >= h - 1 ? h - 1 : y);
        const int yu = yc > 0 ? yc - 1 : 0;
        const int yd = yc < h - 1 ? yc + 1 : h - 1;
        for (int x = 0; x < w; ++x) {
            const int xc = x < 1 ? 0 : (x >= w - 1 ? w - 1 : x);
            const int xl = xc > 0 ? xc - 1 : 0;
            const int xr = xc < w - 1 ? xc + 1 : w - 1;
            const int gx = static_cast<int>(img[yc * w + xr]) - static_cast<int>(img[yc * w + xl]);
            const int gy = static_cast<int>(img[yd * w + xc]) - static_cast<int>(img[yu * w + xc]);
            gradient_[static_cast<size_t>(y * w + x)] =
                std::sqrt(static_cast<float>(gx * gx + gy * gy));
        }
    }
}

bool StereoEngine::compute(const Keyframe& target,
                           const std::vector<std::pair<const Keyframe*, float>>& refs,
                           DepthResult& out) {
    const int w = target.w;
    const int h = target.h;
    if (w < 16 || h < 16 || refs.empty()) return false;
    if (static_cast<int>(target.census.size()) != w * h) return false;

    allocate(w, h);
    out.resize(w, h);

    projectors_.clear();
    projectors_.reserve(refs.size());
    const Mat3 R0 = target.R_cv();
    const Mat3 K0inv = intrinsicsInverse(target.K);
    for (const auto& entry : refs) {
        const Keyframe* kf = entry.first;
        if (!kf || kf->w != w || kf->h != h) continue;
        if (static_cast<int>(kf->census.size()) != w * h) continue;
        const Mat3 R1t = kf->R_cv().transposed();
        const Mat3 K1 = intrinsicsMatrix(kf->K);
        RefProjector p;
        p.M = K1 * (R1t * R0) * K0inv;
        p.e = K1 * (R1t * (target.t - kf->t));
        p.kf = kf;
        p.weight = std::max(0.05f, entry.second);
        projectors_.push_back(p);
    }
    if (projectors_.empty()) return false;

    computeGradient(target, w, h);

    const int numRefs = static_cast<int>(projectors_.size());
    float ax[4], ay[4], az[4], ex[4], ey[4], ez[4], wgt[4];
    for (int i = 0; i < numRefs; ++i) {
        const RefProjector& p = projectors_[i];
        ex[i] = p.e.x;
        ey[i] = p.e.y;
        ez[i] = p.e.z;
        wgt[i] = p.weight;
    }

    const uint64_t* targetCensus = target.census.data();
    const size_t n = static_cast<size_t>(w) * h;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t pixel = static_cast<size_t>(y) * w + x;
            const uint64_t desc = targetCensus[pixel];
            uint8_t* costs = &cost_[pixel * kLevels];

            // Fully uniform or fully textured descriptors carry no information.
            if (desc == 0 || desc == ~uint64_t(0)) {
                for (int l = 0; l < kLevels; ++l) costs[l] = 64;
                continue;
            }

            for (int r = 0; r < numRefs; ++r) {
                const Mat3& M = projectors_[r].M;
                ax[r] = M.m[0] * x + M.m[1] * y + M.m[2];
                ay[r] = M.m[3] * x + M.m[4] * y + M.m[5];
                az[r] = M.m[6] * x + M.m[7] * y + M.m[8];
            }

            for (int l = 0; l < kLevels; ++l) {
                const float d = planeDepth_[l];
                float weighted = 0.f;
                float weightSum = 0.f;
                for (int r = 0; r < numRefs; ++r) {
                    const float qz = az[r] * d + ez[r];
                    if (qz <= 0.08f) {
                        weighted += 64.f * wgt[r];
                        weightSum += wgt[r];
                        continue;
                    }
                    const float qx = (ax[r] * d + ex[r]) / qz;
                    const float qy = (ay[r] * d + ey[r]) / qz;
                    const int u = static_cast<int>(qx + 0.5f);
                    const int v = static_cast<int>(qy + 0.5f);
                    int ham = 64;
                    if (u >= 0 && u < w && v >= 0 && v < h) {
                        const uint64_t refDesc =
                            projectors_[r].kf->census[static_cast<size_t>(v) * w + u];
                        ham = hammingDistance64(desc, refDesc);
                    }
                    weighted += static_cast<float>(ham) * wgt[r];
                    weightSum += wgt[r];
                }
                const float c = weightSum > 0.f ? weighted / weightSum : 64.f;
                costs[l] = static_cast<uint8_t>(std::min(64.f, c + 0.5f));
            }
        }
    }

    // 8-direction semi-global matching.
    std::fill(aggregated_.begin(), aggregated_.end(), 0.f);
    static const int dirs[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                   {1, 1},  {-1, -1}, {1, -1}, {-1, 1}};
    const int L = kLevels;

    for (int dir = 0; dir < 8; ++dir) {
        const int dx = dirs[dir][0];
        const int dy = dirs[dir][1];
        const int yStart = dy >= 0 ? 0 : h - 1;
        const int yEnd = dy >= 0 ? h : -1;
        const int xStart = dx >= 0 ? 0 : w - 1;
        const int xEnd = dx >= 0 ? w : -1;
        const int yStep = dy >= 0 ? 1 : -1;
        const int xStep = dx >= 0 ? 1 : -1;

        for (int y = yStart; y != yEnd; y += yStep) {
            const int prevY = y - dy;
            for (int x = xStart; x != xEnd; x += xStep) {
                const int prevX = x - dx;
                const size_t pixel = static_cast<size_t>(y) * w + x;
                float* curr = &lr_[pixel * L];
                const uint8_t* costs = &cost_[pixel * L];
                const bool hasPrev =
                    prevX >= 0 && prevX < w && prevY >= 0 && prevY < h;
                const float p2 =
                    kP2Base + kP2Gradient * std::min(240.f, gradient_[pixel]);
                if (!hasPrev) {
                    for (int l = 0; l < L; ++l) {
                        const float v = static_cast<float>(costs[l]);
                        curr[l] = v;
                        aggregated_[pixel * L + l] += v;
                    }
                    continue;
                }
                const float* prev = &lr_[(static_cast<size_t>(prevY) * w + prevX) * L];
                float minPrev = prev[0];
                for (int l = 1; l < L; ++l) minPrev = std::min(minPrev, prev[l]);
                for (int l = 0; l < L; ++l) {
                    float v = prev[l];
                    if (l > 0) v = std::min(v, prev[l - 1] + kP1);
                    if (l < L - 1) v = std::min(v, prev[l + 1] + kP1);
                    v = std::min(v, minPrev + p2);
                    const float val = static_cast<float>(costs[l]) + v - minPrev;
                    curr[l] = val;
                    aggregated_[pixel * L + l] += val;
                }
            }
        }
    }

    // Winner-take-all + parabolic sub-pixel refinement (Eqs. 10-11).
    for (size_t pixel = 0; pixel < n; ++pixel) {
        const float* agg = &aggregated_[pixel * L];
        int bestL = 0;
        float best = agg[0];
        for (int l = 1; l < L; ++l) {
            if (agg[l] < best) {
                best = agg[l];
                bestL = l;
            }
        }

        if (bestL <= 0 || bestL >= L - 1 ||
            static_cast<float>(cost_[pixel * L + bestL]) > kMaxMatchCost) {
            rawDepth_[pixel] = 0.f;
            labelSub_[pixel] = 0.f;
            continue;
        }
        const float c0 = agg[bestL - 1];
        const float c1 = best;
        const float c2 = agg[bestL + 1];
        const float denom = c0 - 2.f * c1 + c2;
        float delta = 0.f;
        if (std::fabs(denom) > 1e-4f) {
            delta = 0.5f * (c0 - c2) / denom;
            delta = clampf(delta, -0.5f, 0.5f);
        }
        labelSub_[pixel] = static_cast<float>(bestL) + delta;

        const float inv = planeInvDepth_[bestL] + planeStep_[bestL] * delta;
        const float depth = inv > 1e-6f ? 1.f / inv : 0.f;
        if (depth < near_ || depth > far_) {
            rawDepth_[pixel] = 0.f;
            continue;
        }
        rawDepth_[pixel] = depth;
    }

    // Continuous inverse-depth refinement. The plane sweep quantises depth to
    // ~7 cm at 1.5 m even at 96 planes, which alone would fuse into shells. A
    // local search over +/- one plane with bilinear luminance sampling recovers
    // sub-pixel disparity; the depth of its cost valley becomes the new
    // confidence uncertainty U (flat valley = ambiguous match).
    const float planeStep = std::fabs(planeStep_[0]);
    for (size_t pixel = 0; pixel < n; ++pixel) {
        if (rawDepth_[pixel] <= 0.f) {
            uncertainty_[pixel] = 1.f;
            continue;
        }
        const int x = static_cast<int>(pixel % static_cast<size_t>(w));
        const int y = static_cast<int>(pixel / static_cast<size_t>(w));

        float targetPatch[9];
        for (int dy = -1; dy <= 1; ++dy) {
            const int yy = std::min(h - 1, std::max(0, y + dy));
            for (int dx = -1; dx <= 1; ++dx) {
                const int xx = std::min(w - 1, std::max(0, x + dx));
                targetPatch[(dy + 1) * 3 + (dx + 1)] =
                    static_cast<float>(target.luma[static_cast<size_t>(yy) * w + xx]);
            }
        }

        const float rho0 = 1.f / rawDepth_[pixel];
        const float span = planeStep;  // 2.5 samples per plane step
        float costs[5];
        bool ok[5] = {false, false, false, false, false};
        int validCount = 0;
        for (int k = -2; k <= 2; ++k) {
            const float rho = rho0 + span * 0.5f * static_cast<float>(k);
            if (rho <= 1e-4f) continue;
            const float d = 1.f / rho;
            if (d < near_ || d > far_) continue;
            float cost = 0.f;
            int count = 0;
            bool patchOk = true;
            for (const RefProjector& p : projectors_) {
                if (!p.kf) continue;
                const float qz = (p.M.m[6] * x + p.M.m[7] * y + p.M.m[8]) * d + p.e.z;
                if (qz <= 0.08f) {
                    patchOk = false;
                    break;
                }
                const float qx =
                    ((p.M.m[0] * x + p.M.m[1] * y + p.M.m[2]) * d + p.e.x) / qz;
                const float qy =
                    ((p.M.m[3] * x + p.M.m[4] * y + p.M.m[5]) * d + p.e.y) / qz;
                for (int dy = -1; dy <= 1 && patchOk; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const float sx = qx + dx;
                        const float sy = qy + dy;
                        if (sx < 0.f || sy < 0.f || sx >= w - 1.f || sy >= h - 1.f) {
                            patchOk = false;
                            break;
                        }
                        const int ix0 = static_cast<int>(sx);
                        const int iy0 = static_cast<int>(sy);
                        const float fx = sx - ix0;
                        const float fy = sy - iy0;
                        const uint8_t* row0 = &p.kf->luma[static_cast<size_t>(iy0) * w + ix0];
                        const uint8_t* row1 = row0 + w;
                        const float v = (1.f - fx) * (1.f - fy) * row0[0] +
                                        fx * (1.f - fy) * row0[1] +
                                        (1.f - fx) * fy * row1[0] + fx * fy * row1[1];
                        cost += std::fabs(targetPatch[(dy + 1) * 3 + (dx + 1)] - v);
                        ++count;
                    }
                }
                if (!patchOk) break;
            }
            if (!patchOk || count == 0) continue;
            costs[k + 2] = cost / static_cast<float>(count);
            ok[k + 2] = true;
            ++validCount;
        }
        if (validCount < 3) {
            uncertainty_[pixel] = 1.f;
            continue;
        }

        int bestK = -1;
        float bestCost = std::numeric_limits<float>::max();
        for (int k = 0; k < 5; ++k) {
            if (ok[k] && costs[k] < bestCost) {
                bestCost = costs[k];
                bestK = k;
            }
        }
        float sideCost = bestCost;
        if (bestK > 0 && ok[bestK - 1]) sideCost = std::max(sideCost, costs[bestK - 1]);
        if (bestK < 4 && ok[bestK + 1]) sideCost = std::max(sideCost, costs[bestK + 1]);
        // Valley depth relative to the residual: flat cost curves mean the
        // match is ambiguous, so uncertainty approaches 1.
        const float valley = (sideCost - bestCost) / (bestCost + 8.f);
        uncertainty_[pixel] = clampf(1.f - valley, 0.f, 1.f);

        float rho = rho0 + span * 0.5f * static_cast<float>(bestK - 2);
        if (bestK > 0 && bestK < 4 && ok[bestK - 1] && ok[bestK + 1]) {
            const float c0 = costs[bestK - 1];
            const float c2 = costs[bestK + 1];
            const float denom = c0 - 2.f * bestCost + c2;
            if (denom > 1e-3f) {
                float shift = 0.5f * (c0 - c2) / denom;
                shift = clampf(shift, -0.5f, 0.5f);
                rho += span * 0.5f * shift;
            }
        }
        const float refined = rho > 1e-6f ? 1.f / rho : 0.f;
        if (refined >= near_ && refined <= far_) {
            rawDepth_[pixel] = refined;
        } else {
            rawDepth_[pixel] = 0.f;
            uncertainty_[pixel] = 1.f;
        }
    }

    // Forward-backward consistency: reproject every depth into the references'
    // own depth maps and drop matches that no reference agrees with. Census
    // matching alone cannot tell a texture repeat from the true surface, and
    // those false matches are what tear the TSDF into shells.
    for (size_t pixel = 0; pixel < n; ++pixel) {
        if (rawDepth_[pixel] <= 0.f) continue;
        const int x = static_cast<int>(pixel % static_cast<size_t>(w));
        const int y = static_cast<int>(pixel / static_cast<size_t>(w));
        const float d = rawDepth_[pixel];
        int checked = 0;
        int agreed = 0;
        for (const RefProjector& p : projectors_) {
            if (!p.kf || p.kf->depthMap.size() != n) continue;
            const float qz = (p.M.m[6] * x + p.M.m[7] * y + p.M.m[8]) * d + p.e.z;
            if (qz <= 0.08f) continue;
            const float qx = ((p.M.m[0] * x + p.M.m[1] * y + p.M.m[2]) * d + p.e.x) / qz;
            const float qy = ((p.M.m[3] * x + p.M.m[4] * y + p.M.m[5]) * d + p.e.y) / qz;
            const int u = static_cast<int>(qx + 0.5f);
            const int v = static_cast<int>(qy + 0.5f);
            if (u < 0 || u >= w || v < 0 || v >= h) continue;
            const float rd = p.kf->depthMap[static_cast<size_t>(v) * w + u];
            if (rd <= 0.f) continue;
            ++checked;
            const float tol = 0.04f + 0.03f * d;
            if (std::fabs(rd - d) <= tol) ++agreed;
        }
        if (checked > 0 && agreed == 0) {
            rawDepth_[pixel] = 0.f;
            uncertainty_[pixel] = 1.f;
        }
    }

    // Confidence filtering, Eqs. (13)-(14): the 5x5 sub-level label
    // disagreement weights the SGM uncertainty U(x) and the product is
    // normalised by the frame maximum. dirTerm_ is recycled as the exposure
    // buffer because the per-direction sums are no longer needed.
    float maxExposure = 0.f;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t pixel = static_cast<size_t>(y) * w + x;
            if (rawDepth_[pixel] <= 0.f) {
                dirTerm_[pixel] = 0.f;
                continue;
            }
            int count = 0;
            float disagreement = 0.f;
            for (int dy = -2; dy <= 2; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -2; dx <= 2; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    const size_t q = static_cast<size_t>(yy) * w + xx;
                    if (rawDepth_[q] <= 0.f) continue;
                    ++count;
                    if (std::fabs(labelSub_[pixel] - labelSub_[q]) > 2.f) {
                        disagreement += 1.f;
                    }
                }
            }
            const float omega = count > 0 ? disagreement / static_cast<float>(count) : 1.f;
            dirTerm_[pixel] = omega * uncertainty_[pixel];
            maxExposure = std::max(maxExposure, dirTerm_[pixel]);
        }
    }

    for (size_t pixel = 0; pixel < n; ++pixel) {
        if (rawDepth_[pixel] <= 0.f) {
            out.depth[pixel] = 0.f;
            out.confidence[pixel] = 0.f;
            continue;
        }
        const float confidence =
            maxExposure > 1e-6f ? 1.f - dirTerm_[pixel] / maxExposure : 1.f;
        const float m = clampf(confidence, 0.f, 1.f);
        if (m < kMinConfidence) {
            out.depth[pixel] = 0.f;
            out.confidence[pixel] = 0.f;
        } else {
            out.depth[pixel] = rawDepth_[pixel];
            out.confidence[pixel] = m;
        }
    }

    return true;
}

}  // namespace recon
