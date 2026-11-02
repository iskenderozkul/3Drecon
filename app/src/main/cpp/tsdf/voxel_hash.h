#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "math/math.h"

namespace recon {

enum class VoxelState : uint8_t { NORMAL = 0, ADD = 1, UPDATE = 2, DELETE = 3 };

struct VoxelBlock {
    static constexpr int kSize = 8;
    static constexpr int kTotal = kSize * kSize * kSize;

    int bx = 0, by = 0, bz = 0;
    int16_t tsdf[kTotal];
    uint8_t weight[kTotal];
    uint8_t color[kTotal * 3];
    VoxelState state = VoxelState::ADD;
    bool modified = false;

    VoxelBlock(int x, int y, int z) : bx(x), by(y), bz(z) {
        for (int i = 0; i < kTotal; ++i) {
            tsdf[i] = 30000;  // empty space, SDF == +1
            weight[i] = 0;
            color[i * 3 + 0] = 200;
            color[i * 3 + 1] = 200;
            color[i * 3 + 2] = 200;
        }
    }

    static int index(int vx, int vy, int vz) { return (vz * kSize + vy) * kSize + vx; }
};

// Scalable, conflict-free voxel hashing. Block coordinates are packed into a
// 21-bit-per-axis key which covers a +/-10 km range without a predefined grid;
// collisions from wrapped coordinates are resolved with deterministic probing
// that verifies the stored block coordinates.
class VoxelHash {
public:
    static constexpr float kFixedScale = 30000.f;

    VoxelHash(float voxelSize = 0.03f, size_t maxBlocks = 12000);

    float voxelSize() const { return voxelSize_; }
    float truncation() const { return truncation_; }
    size_t size() const { return blocks_.size(); }
    size_t maxBlocks() const { return maxBlocks_; }

    VoxelBlock* get(int bx, int by, int bz);
    const VoxelBlock* get(int bx, int by, int bz) const;
    VoxelBlock* getOrCreate(int bx, int by, int bz);

    Vec3 blockOrigin(const VoxelBlock& b) const {
        return Vec3(static_cast<float>(b.bx * VoxelBlock::kSize) * voxelSize_,
                    static_cast<float>(b.by * VoxelBlock::kSize) * voxelSize_,
                    static_cast<float>(b.bz * VoxelBlock::kSize) * voxelSize_);
    }

    // Trilinear-free nearest sample used by marching cubes.
    bool sampleVoxel(int gx, int gy, int gz, float& tsdf, bool& known,
                     uint8_t* rgb, uint8_t* weight = nullptr) const;

    std::vector<std::unique_ptr<VoxelBlock>>& blocks() { return blocks_; }
    const std::vector<std::unique_ptr<VoxelBlock>>& blocks() const { return blocks_; }

    void evictFarthest(const Vec3& camera, float maxDistance, size_t targetSize);

    void clear() { blocks_.clear(); index_.clear(); }

    static uint64_t coordKey(int bx, int by, int bz);

private:
    VoxelBlock* find(int bx, int by, int bz) const;

    float voxelSize_;
    float truncation_;
    size_t maxBlocks_;
    std::vector<std::unique_ptr<VoxelBlock>> blocks_;
    std::unordered_map<uint64_t, VoxelBlock*> index_;
};

}  // namespace recon
