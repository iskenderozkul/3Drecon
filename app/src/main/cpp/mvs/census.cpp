#include "mvs/census.h"

namespace recon {

void computeCensus(const uint8_t* luma, int w, int h, std::vector<uint64_t>& out) {
    const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
    out.assign(n, 0);
    if (w < 10 || h < 10) return;

    for (int y = 0; y < h; ++y) {
        const int yBase = y * w;
        for (int x = 0; x < w; ++x) {
            const int center = luma[yBase + x];
            uint64_t bits = 0;
            int bit = 0;
            for (int dy = -4; dy < 4; ++dy) {
                int sy = y + dy;
                sy = sy < 0 ? 0 : (sy >= h ? h - 1 : sy);
                const int row = sy * w;
                for (int dx = -4; dx < 4; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int sx = x + dx;
                    sx = sx < 0 ? 0 : (sx >= w ? w - 1 : sx);
                    if (luma[row + sx] < center) bits |= (uint64_t(1) << bit);
                    ++bit;
                }
            }
            out[static_cast<size_t>(yBase + x)] = bits;
        }
    }
}

}  // namespace recon
