#include "tsdf/integrator.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <thread>
#include <unordered_set>

namespace recon {
namespace {
constexpr int kPixelStep = 1;
constexpr float kMinWeightForCarve = 3.f;
}  // namespace

void TSDFIntegrator::integrate(const Keyframe& kf, const DepthResult& depth, VoxelHash& hash,
                               std::vector<VoxelBlock*>& dirty) {
    dirty.clear();
    if (depth.w != kf.w || depth.h != kf.h || depth.depth.empty()) return;

    const float vs = hash.voxelSize();
    const float blockSize = vs * VoxelBlock::kSize;
    const float tau = hash.truncation();
    const Mat3 R_cv = kf.R_cv();          // camera -> world
    const Mat3 R_wc = R_cv.transposed();  // world -> camera
    const Vec3 origin = kf.t;
    const Intrinsics& K = kf.K;

    std::unordered_set<uint64_t> seen;
    seen.reserve(16384);
    std::vector<VoxelBlock*> candidates;
    candidates.reserve(16384);

    auto addWorldPoint = [&](const Vec3& pWorld) {
        const int bx = static_cast<int>(std::floor(pWorld.x / blockSize));
        const int by = static_cast<int>(std::floor(pWorld.y / blockSize));
        const int bz = static_cast<int>(std::floor(pWorld.z / blockSize));
        const uint64_t key = VoxelHash::coordKey(bx, by, bz);
        if (!seen.insert(key).second) return;
        if (VoxelBlock* block = hash.getOrCreate(bx, by, bz)) candidates.push_back(block);
    };

    for (int y = 0; y < depth.h; y += kPixelStep) {
        for (int x = 0; x < depth.w; x += kPixelStep) {
            const size_t idx = static_cast<size_t>(y) * depth.w + x;
            const float d = depth.depth[idx];
            if (d <= 0.05f) continue;
            const float conf = depth.confidence[idx];
            if (conf <= 0.f) continue;

            const Vec3 pCv = K.unproject(static_cast<float>(x), static_cast<float>(y), d);
            // Allocate the surface band: centre sample plus +/- truncation.
            const float nearScale = (d - tau) / d;
            if (nearScale > 0.05f) {
                const Vec3 pn = R_cv * (pCv * nearScale) + origin;
                addWorldPoint(pn);
            }
            const Vec3 pc = R_cv * pCv + origin;
            addWorldPoint(pc);
            const Vec3 pf = R_cv * (pCv * ((d + tau) / d)) + origin;
            addWorldPoint(pf);
        }
    }

    if (candidates.empty()) return;

    if (threads_ <= 1 || candidates.size() < 64) {
        for (VoxelBlock* block : candidates) {
            integrateBlock(kf, depth, hash, block, R_wc, tau);
        }
    } else {
        std::atomic<size_t> next{0};
        std::vector<std::thread> workers;
        const int threadCount = std::min<int>(threads_, static_cast<int>(candidates.size()));
        workers.reserve(threadCount);
        for (int t = 0; t < threadCount; ++t) {
            workers.emplace_back([&]() {
                while (true) {
                    const size_t i = next.fetch_add(1, std::memory_order_relaxed);
                    if (i >= candidates.size()) break;
                    integrateBlock(kf, depth, hash, candidates[i], R_wc, tau);
                }
            });
        }
        for (auto& worker : workers) worker.join();
    }

    for (VoxelBlock* block : candidates) {
        if (!block->modified) continue;
        block->modified = false;
        // Blocks created during this pass keep ADD; already-known blocks are
        // promoted to UPDATE. The mesher consumes both and sets NORMAL.
        if (block->state != VoxelState::ADD) block->state = VoxelState::UPDATE;
        dirty.push_back(block);
    }
}

void TSDFIntegrator::integrateBlock(const Keyframe& kf, const DepthResult& depth, VoxelHash& hash,
                                    VoxelBlock* block, const Mat3& Rcw, float tau) {
    const float vs = hash.voxelSize();
    const float Q = VoxelHash::kFixedScale;
    const Intrinsics& K = kf.K;
    const float fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
    const int w = depth.w, h = depth.h;
    const float ox = static_cast<float>(block->bx * VoxelBlock::kSize) * vs;
    const float oy = static_cast<float>(block->by * VoxelBlock::kSize) * vs;
    const float oz = static_cast<float>(block->bz * VoxelBlock::kSize) * vs;

    struct Sample {
        int index = 0;
        int u = 0;
        int v = 0;
        float sdf = 0.f;
    };

    // Paper Section 5.2: a cube is either fused completely or not at all. All
    // eight corner updates are staged first and reverted when any corner lands
    // outside the depth map, on an invalid pixel, or behind the observed
    // surface; this is what keeps block borders free of broken triangles.
    constexpr int kCube = 2;
    for (int vz0 = 0; vz0 < VoxelBlock::kSize; vz0 += kCube) {
        for (int vy0 = 0; vy0 < VoxelBlock::kSize; vy0 += kCube) {
            for (int vx0 = 0; vx0 < VoxelBlock::kSize; vx0 += kCube) {
                Sample samples[8];
                bool valid = true;
                for (int c = 0; c < 8 && valid; ++c) {
                    const int vx = vx0 + (c & 1);
                    const int vy = vy0 + ((c >> 1) & 1);
                    const int vz = vz0 + ((c >> 2) & 1);
                    const Vec3 pWorld{ox + (vx + 0.5f) * vs, oy + (vy + 0.5f) * vs,
                                      oz + (vz + 0.5f) * vs};
                    const Vec3 pCam = Rcw * (pWorld - kf.t);
                    if (pCam.z <= 0.08f) {
                        valid = false;
                        break;
                    }
                    const float invZ = 1.f / pCam.z;
                    const int u = static_cast<int>(fx * pCam.x * invZ + cx + 0.5f);
                    const int v = static_cast<int>(fy * pCam.y * invZ + cy + 0.5f);
                    if (u < 0 || u >= w || v < 0 || v >= h) {
                        valid = false;
                        break;
                    }
                    const size_t idx = static_cast<size_t>(v) * w + u;
                    const float dMeas = depth.depth[idx];
                    const float conf = depth.confidence[idx];
                    if (dMeas <= 0.08f || conf <= 0.f) {
                        valid = false;
                        break;
                    }
                    const float sdf = dMeas - pCam.z;
                    if (sdf < -tau) {
                        valid = false;
                        break;
                    }
                    samples[c].index = VoxelBlock::index(vx, vy, vz);
                    samples[c].u = u;
                    samples[c].v = v;
                    samples[c].sdf = sdf;
                }
                if (!valid) continue;

                for (int c = 0; c < 8; ++c) {
                    const Sample& s = samples[c];
                    const int wOld = block->weight[s.index];
                    const int tsdfOld = block->tsdf[s.index];

                    // Paper Eq. (21): every observation contributes weight 1 and
                    // free space (sdf > tau) saturates at +Q. Ghost removal is
                    // left to visibilityCarve, which is more conservative than
                    // carving from a single frame's ray band.
                    int measured = 0;
                    if (s.sdf > tau) {
                        measured = static_cast<int>(Q);
                    } else {
                        measured = static_cast<int>((s.sdf / tau) * Q);
                        measured = std::max(-static_cast<int>(Q),
                                            std::min(static_cast<int>(Q), measured));
                    }
                    const int dWeight = 1;

                    const int wNew = std::min(255, wOld + dWeight);
                    const int tsdfNew =
                        wOld > 0 ? (tsdfOld * wOld + measured * dWeight) / wNew : measured;
                    block->tsdf[s.index] = static_cast<int16_t>(tsdfNew);
                    block->weight[s.index] = static_cast<uint8_t>(wNew);

                    const size_t rgbIdx = (static_cast<size_t>(s.v) * w + s.u) * 3;
                    int r = block->color[s.index * 3 + 0];
                    int g = block->color[s.index * 3 + 1];
                    int b = block->color[s.index * 3 + 2];
                    r = (r * wOld + kf.rgb[rgbIdx + 0] * dWeight) / wNew;
                    g = (g * wOld + kf.rgb[rgbIdx + 1] * dWeight) / wNew;
                    b = (b * wOld + kf.rgb[rgbIdx + 2] * dWeight) / wNew;
                    block->color[s.index * 3 + 0] = static_cast<uint8_t>(r);
                    block->color[s.index * 3 + 1] = static_cast<uint8_t>(g);
                    block->color[s.index * 3 + 2] = static_cast<uint8_t>(b);

                    block->modified = true;
                }
            }
        }
    }
}

void TSDFIntegrator::visibilityCarve(const Keyframe& kf, const DepthResult& depth,
                                     VoxelHash& hash, std::vector<VoxelBlock*>& dirty) {
    if (depth.w != kf.w || depth.h != kf.h || depth.depth.empty()) return;

    const float vs = hash.voxelSize();
    const float tau = hash.truncation();
    const float Q = VoxelHash::kFixedScale;
    const Mat3 Rcw = kf.R_cv().transposed();  // world -> camera
    const float fx = kf.K.fx, fy = kf.K.fy, cx = kf.K.cx, cy = kf.K.cy;
    const int w = depth.w, h = depth.h;
    const float blockSize = static_cast<float>(VoxelBlock::kSize) * vs;
    const float halfBlock = 0.5f * blockSize;

    // Silhouette guard, precomputed once: the smallest measurement in each 3x3
    // neighbourhood. Doing it per voxel made carving nine depth lookups deep.
    std::vector<float> minDepth(static_cast<size_t>(w) * h, 0.f);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float best = 0.f;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    const float d = depth.depth[static_cast<size_t>(yy) * w + xx];
                    if (d > 0.08f && (best == 0.f || d < best)) best = d;
                }
            }
            minDepth[static_cast<size_t>(y) * w + x] = best;
        }
    }

    std::unordered_set<VoxelBlock*> queued(dirty.begin(), dirty.end());
    std::vector<VoxelBlock*> targets;
    targets.reserve(hash.blocks().size());
    for (auto& owned : hash.blocks()) {
        if (!queued.count(owned.get())) targets.push_back(owned.get());
    }

    // Each block is visited by exactly one worker, and blocks share no state,
    // so the per-voxel work parallelises without locking.
    auto carveBlock = [&](VoxelBlock* block) -> int {
        // Coarse reject: the block centre must itself be in observed free
        // space, otherwise the block is on (or behind) the visible surface and
        // its voxels cannot be visibility conflicts.
        const Vec3 center{static_cast<float>(block->bx) * blockSize + halfBlock,
                          static_cast<float>(block->by) * blockSize + halfBlock,
                          static_cast<float>(block->bz) * blockSize + halfBlock};
        const Vec3 centerCam = Rcw * (center - kf.t);
        if (centerCam.z <= 0.15f) return 0;
        const int cu = static_cast<int>(fx * centerCam.x / centerCam.z + cx + 0.5f);
        const int cv = static_cast<int>(fy * centerCam.y / centerCam.z + cy + 0.5f);
        if (cu < 0 || cu >= w || cv < 0 || cv >= h) return 0;
        const float centerDepth = depth.depth[static_cast<size_t>(cv) * w + cu];
        if (centerDepth <= 0.08f) return 0;
        if (centerCam.z >= centerDepth - tau) return 0;

        const float ox = static_cast<float>(block->bx * VoxelBlock::kSize) * vs;
        const float oy = static_cast<float>(block->by * VoxelBlock::kSize) * vs;
        const float oz = static_cast<float>(block->bz * VoxelBlock::kSize) * vs;

        int changed = 0;
        for (int vz = 0; vz < VoxelBlock::kSize; ++vz) {
            for (int vy = 0; vy < VoxelBlock::kSize; ++vy) {
                for (int vx = 0; vx < VoxelBlock::kSize; ++vx) {
                    const int index = VoxelBlock::index(vx, vy, vz);
                    const int wOld = block->weight[index];
                    if (wOld < kMinWeightForCarve) continue;
                    const int tsdfOld = block->tsdf[index];
                    // Only voxels inside the truncation band can be ghosts;
                    // saturated free space needs no carving. The band is wide
                    // enough to include confident surface voxels, while the
                    // 1.5*tau margin below keeps measurements from eroding the
                    // surface they actually describe.
                    if (tsdfOld >= static_cast<int>(0.85f * Q) ||
                        tsdfOld <= static_cast<int>(-0.85f * Q)) {
                        continue;
                    }
                    const Vec3 pWorld{ox + (vx + 0.5f) * vs, oy + (vy + 0.5f) * vs,
                                      oz + (vz + 0.5f) * vs};
                    const Vec3 pCam = Rcw * (pWorld - kf.t);
                    if (pCam.z <= 0.08f) continue;
                    const float invZ = 1.f / pCam.z;
                    const int u = static_cast<int>(fx * pCam.x * invZ + cx + 0.5f);
                    const int v = static_cast<int>(fy * pCam.y * invZ + cy + 0.5f);
                    if (u < 0 || u >= w || v < 0 || v >= h) continue;
                    const size_t idx = static_cast<size_t>(v) * w + u;
                    const float dMeas = depth.depth[idx];
                    if (dMeas <= 0.08f) continue;
                    if (pCam.z >= dMeas - 1.5f * tau) continue;
                    // All neighbouring measurements must also be clearly behind
                    // the voxel, otherwise the sample belongs to a thin
                    // foreground structure that the ray just missed.
                    const float minNeighbourDepth = minDepth[idx];
                    if (minNeighbourDepth <= 0.08f) continue;
                    if (minNeighbourDepth < pCam.z + tau) continue;

                    // Mark the voxel as measured free space (+Q, the same value
                    // a ray passing through it would write). Flipping it to -Q
                    // put it *inside* a phantom surface, so marching cubes kept
                    // a sheet there and the real surface behind it stayed too.
                    // Zeroing the weight instead removed the voxel from the
                    // field entirely; cubes were skipped and every conflicting
                    // frame punched a hole, which shredded the mesh into
                    // islands. Free space keeps the field continuous, so the
                    // ghost surface collapses back onto the real one.
                    block->tsdf[index] = static_cast<int16_t>(Q);
                    block->weight[index] = static_cast<uint8_t>(std::max(1, wOld));
                    ++changed;
                }
            }
        }
        return changed;
    };

    const int threadCount = std::min<int>(threads_, static_cast<int>(targets.size()));
    if (threadCount <= 1 || targets.size() < 64) {
        int carvedVoxels = 0;
        for (VoxelBlock* block : targets) {
            const int n = carveBlock(block);
            if (n == 0) continue;
            carvedVoxels += n;
            if (block->state != VoxelState::ADD) block->state = VoxelState::UPDATE;
            dirty.push_back(block);
        }
        carvedVoxels_ = static_cast<uint32_t>(carvedVoxels);
        return;
    }

    std::vector<std::vector<VoxelBlock*>> local(static_cast<size_t>(threadCount));
    std::vector<int> localCount(static_cast<size_t>(threadCount), 0);
    std::atomic<size_t> next{0};
    std::vector<std::thread> workers;
    workers.reserve(static_cast<size_t>(threadCount));
    for (int t = 0; t < threadCount; ++t) {
        workers.emplace_back([&, t]() {
            std::vector<VoxelBlock*>& carved = local[static_cast<size_t>(t)];
            int& count = localCount[static_cast<size_t>(t)];
            while (true) {
                const size_t i = next.fetch_add(1, std::memory_order_relaxed);
                if (i >= targets.size()) break;
                const int n = carveBlock(targets[i]);
                if (n == 0) continue;
                count += n;
                carved.push_back(targets[i]);
            }
        });
    }
    for (auto& worker : workers) worker.join();

    uint32_t carvedVoxels = 0;
    for (int t = 0; t < threadCount; ++t) {
        carvedVoxels += static_cast<uint32_t>(localCount[static_cast<size_t>(t)]);
        for (VoxelBlock* block : local[static_cast<size_t>(t)]) {
            if (block->state != VoxelState::ADD) block->state = VoxelState::UPDATE;
            dirty.push_back(block);
        }
    }
    carvedVoxels_ = carvedVoxels;
}

}  // namespace recon
