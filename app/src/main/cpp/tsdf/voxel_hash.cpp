#include "tsdf/voxel_hash.h"

#include <algorithm>
#include <cmath>

namespace recon {
namespace {
constexpr uint64_t kGolden = 0x9e3779b97f4a7c15ull;
inline uint64_t mix64(uint64_t x) {
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9ull;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebull;
    x ^= x >> 31;
    return x;
}
}  // namespace

VoxelHash::VoxelHash(float voxelSize, size_t maxBlocks)
    : voxelSize_(voxelSize),
      // 2.5 voxels instead of 3: the truncation band is the region where a
      // silhouette can be rounded or stretched, so a narrower band hugs the
      // measured rim. Anything shallower than 2 voxels starts to tear where the
      // depth map has a dropout, hence 2.5 as the compromise.
      truncation_(voxelSize * 2.5f),
      maxBlocks_(maxBlocks) {
    blocks_.reserve(maxBlocks / 4);
    index_.reserve(maxBlocks);
}

uint64_t VoxelHash::coordKey(int bx, int by, int bz) {
    const uint64_t x = static_cast<uint64_t>(static_cast<uint32_t>(bx)) & 0x1FFFFFull;
    const uint64_t y = static_cast<uint64_t>(static_cast<uint32_t>(by)) & 0x1FFFFFull;
    const uint64_t z = static_cast<uint64_t>(static_cast<uint32_t>(bz)) & 0x1FFFFFull;
    return (x << 42) | (y << 21) | z;
}

VoxelBlock* VoxelHash::find(int bx, int by, int bz) const {
    uint64_t key = coordKey(bx, by, bz);
    for (int probe = 0; probe < 64; ++probe) {
        auto it = index_.find(key);
        if (it == index_.end()) return nullptr;
        VoxelBlock* b = it->second;
        if (b->bx == bx && b->by == by && b->bz == bz) return b;
        key = mix64(key + kGolden + static_cast<uint64_t>(probe));
    }
    return nullptr;
}

VoxelBlock* VoxelHash::get(int bx, int by, int bz) { return find(bx, by, bz); }
const VoxelBlock* VoxelHash::get(int bx, int by, int bz) const { return find(bx, by, bz); }

VoxelBlock* VoxelHash::getOrCreate(int bx, int by, int bz) {
    if (VoxelBlock* existing = find(bx, by, bz)) return existing;

    uint64_t key = coordKey(bx, by, bz);
    for (int probe = 0; probe < 64; ++probe) {
        auto it = index_.find(key);
        if (it == index_.end()) {
            auto block = std::make_unique<VoxelBlock>(bx, by, bz);
            VoxelBlock* raw = block.get();
            blocks_.push_back(std::move(block));
            index_.emplace(key, raw);
            return raw;
        }
        key = mix64(key + kGolden + static_cast<uint64_t>(probe));
    }
    return nullptr;
}

bool VoxelHash::sampleVoxel(int gx, int gy, int gz, float& tsdf, bool& known,
                            uint8_t* rgb, uint8_t* weightOut) const {
    const int bx = floorDiv(gx, VoxelBlock::kSize);
    const int by = floorDiv(gy, VoxelBlock::kSize);
    const int bz = floorDiv(gz, VoxelBlock::kSize);
    const VoxelBlock* block = find(bx, by, bz);
    if (!block) {
        known = false;
        tsdf = 1.f;
        if (weightOut) *weightOut = 0;
        return false;
    }
    const int lx = gx - bx * VoxelBlock::kSize;
    const int ly = gy - by * VoxelBlock::kSize;
    const int lz = gz - bz * VoxelBlock::kSize;
    const int idx = VoxelBlock::index(lx, ly, lz);
    known = block->weight[idx] > 0;
    tsdf = static_cast<float>(block->tsdf[idx]) / VoxelHash::kFixedScale;
    if (rgb) {
        rgb[0] = block->color[idx * 3 + 0];
        rgb[1] = block->color[idx * 3 + 1];
        rgb[2] = block->color[idx * 3 + 2];
    }
    if (weightOut) *weightOut = block->weight[idx];
    return known;
}

void VoxelHash::evictFarthest(const Vec3& camera, float maxDistance, size_t targetSize) {
    if (blocks_.size() <= targetSize) return;
    const float maxDistSq = maxDistance * maxDistance;
    std::vector<size_t> candidates;
    candidates.reserve(blocks_.size());
    for (size_t i = 0; i < blocks_.size(); ++i) {
        const VoxelBlock& b = *blocks_[i];
        const Vec3 origin = blockOrigin(b);
        const float dx = origin.x - camera.x;
        const float dy = origin.y - camera.y;
        const float dz = origin.z - camera.z;
        if (dx * dx + dy * dy + dz * dz > maxDistSq) candidates.push_back(i);
    }
    if (candidates.empty()) return;
    std::sort(candidates.begin(), candidates.end(), [&](size_t a, size_t b) {
        const Vec3 oa = blockOrigin(*blocks_[a]);
        const Vec3 ob = blockOrigin(*blocks_[b]);
        return (oa - camera).lengthSq() > (ob - camera).lengthSq();
    });

    const size_t toRemove = blocks_.size() - targetSize;
    const size_t removeCount = std::min(toRemove, candidates.size());
    std::vector<bool> remove(blocks_.size(), false);
    for (size_t i = 0; i < removeCount; ++i) remove[candidates[i]] = true;

    // Rebuild the index without the evicted blocks.
    index_.clear();
    std::vector<std::unique_ptr<VoxelBlock>> kept;
    kept.reserve(blocks_.size() - removeCount);
    for (size_t i = 0; i < blocks_.size(); ++i) {
        if (remove[i]) continue;
        VoxelBlock* raw = blocks_[i].get();
        index_.emplace(coordKey(raw->bx, raw->by, raw->bz), raw);
        kept.push_back(std::move(blocks_[i]));
    }
    blocks_.swap(kept);
}

}  // namespace recon
