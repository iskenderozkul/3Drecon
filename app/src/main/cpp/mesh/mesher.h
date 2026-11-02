#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "tsdf/voxel_hash.h"

namespace recon {

struct MeshVertex {
    float px, py, pz;
    float nx, ny, nz;
    uint8_t r, g, b, a;
};
static_assert(sizeof(MeshVertex) == 28, "MeshVertex must stay tightly packed");

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
    uint32_t version = 0;
    uint32_t triangleCount = 0;
};

struct BlockMesh {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
};

// Incremental Marching Cubes. Only blocks flagged ADD/UPDATE are re-triangulated;
// the remaining surface triangles are already cached per block.
class MeshBuilder {
public:
    void extractDirty(const VoxelHash& hash, const std::vector<VoxelBlock*>& dirty);
    // Re-triangulates every block from the current voxel field. The incremental
    // cache only covers blocks that changed while scanning; a full pass at the
    // end makes the final model reflect every observation in one consistent
    // state (including blocks later refilled after being carved).
    void rebuildAll(const VoxelHash& hash);
    // Drops cached geometry for blocks no longer present in the hash (after
    // eviction) and rebuilds the snapshot once.
    void pruneBlocks(const VoxelHash& hash);
    void reset();

    std::shared_ptr<const MeshData> snapshot() const;
    uint32_t version() const;
    int vertexCount() const;
    int triangleCount() const;
    uint32_t indexCount() const;

private:
    void extractBlock(const VoxelHash& hash, const VoxelBlock& block, BlockMesh& out) const;
    void rebuildSnapshot();

    std::unordered_map<uint64_t, BlockMesh> blockMeshes_;
    mutable std::mutex mutex_;
    std::shared_ptr<MeshData> snapshot_ = std::make_shared<MeshData>();
    uint32_t version_ = 1;
};

}  // namespace recon
