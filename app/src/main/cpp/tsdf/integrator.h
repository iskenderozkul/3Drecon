#pragma once

#include <vector>

#include "core/keyframe.h"
#include "mvs/stereo.h"
#include "tsdf/voxel_hash.h"

namespace recon {

// Projective TSDF fusion with visibility-based dynamic object eviction.
// Voxel updates are staged per 2x2x2 cube and rolled back when any corner
// projects outside the depth map, onto an invalid pixel or behind the surface
// (paper Section 5.2), which removes most broken triangles at map borders.
class TSDFIntegrator {
public:
    explicit TSDFIntegrator(int workerThreads = 3) : threads_(workerThreads) {}

    // Integrates one depth map into the scalable hash. Fills dirty with blocks
    // whose voxels changed (state ADD or UPDATE).
    void integrate(const Keyframe& kf, const DepthResult& depth, VoxelHash& hash,
                   std::vector<VoxelBlock*>& dirty);

    // Paper 5.1.2 visibility check: every allocated voxel is projected into the
    // new keyframe and surface voxels that ended up more than a truncation
    // distance in front of the measured surface (dynamic objects that went
    // away) are carved below zero. Appends newly modified blocks to dirty.
    void visibilityCarve(const Keyframe& kf, const DepthResult& depth, VoxelHash& hash,
                         std::vector<VoxelBlock*>& dirty);

    // Voxels the last visibilityCarve rewrote to free space. Carving is what
    // renews the surface, but it also erases geometry, so it is reported: a
    // large count with a torn mesh means the carve is too eager.
    uint32_t carvedVoxels() const { return carvedVoxels_; }

private:
    void integrateBlock(const Keyframe& kf, const DepthResult& depth, VoxelHash& hash,
                        VoxelBlock* block, const Mat3& Rcw, float tau);

    int threads_ = 3;
    uint32_t carvedVoxels_ = 0;
};

}  // namespace recon
