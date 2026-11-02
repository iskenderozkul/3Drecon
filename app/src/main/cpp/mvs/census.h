#pragma once

#include <cstdint>
#include <vector>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define RECON_HAVE_NEON 1
#endif

namespace recon {

// 8x8 Census window (center excluded) -> 63 comparison bits packed into uint64.
void computeCensus(const uint8_t* luma, int w, int h, std::vector<uint64_t>& out);

// Bitwise Hamming distance. On ARM the 64-bit XOR is popcounted 8 bytes at a
// time with NEON (VCNT), which is the hot inner loop of the cost volume.
inline int hammingDistance64(uint64_t a, uint64_t b) {
#if defined(RECON_HAVE_NEON)
    const uint64_t diff = a ^ b;
    const uint8x8_t bytes = vreinterpret_u8_u64(vdup_n_u64(diff));
    const uint8x8_t counts = vcnt_u8(bytes);
    const uint16x4_t pairs = vpaddl_u8(counts);
    const uint32x2_t quads = vpaddl_u16(pairs);
    return static_cast<int>(vget_lane_u32(quads, 0) + vget_lane_u32(quads, 1));
#else
    return __builtin_popcountll(a ^ b);
#endif
}

}  // namespace recon
