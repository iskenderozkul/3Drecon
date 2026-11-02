#include "mesh/mesher.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <unordered_map>
#include <vector>

#include "mesh/mc_tables.h"

namespace recon {
namespace {

constexpr int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                               {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
constexpr int kEdge[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

struct CachedSample {
    float tsdf = 1.f;
    uint8_t weight = 0;
    uint8_t rgb[3] = {200, 200, 200};
};

struct FieldSample {
    float tsdf = 1.f;
    uint8_t weight = 0;
    uint8_t rgb[3] = {200, 200, 200};
    bool valid = false;
};

// Separable [1,2,1] per axis, indexed by Manhattan distance.
constexpr int kKernel[4] = {8, 4, 2, 1};

inline uint64_t blockQuadKey(int bx, int by, int bz) {
    const uint64_t x = static_cast<uint64_t>(static_cast<uint32_t>(bx)) & 0x1FFFFFull;
    const uint64_t y = static_cast<uint64_t>(static_cast<uint32_t>(by)) & 0x1FFFFFull;
    const uint64_t z = static_cast<uint64_t>(static_cast<uint32_t>(bz)) & 0x1FFFFFull;
    return (x << 42) | (y << 21) | z;
}

// Marching cubes emits one small triangle per grid cell even where the surface
// is a plane, so a wide empty wall costs as many triangles as a detailed
// object. When a whole block is flat, its boundary loop already carries the
// complete shape, and the interior is replaced by a fan over that loop. The
// boundary vertices keep their marching-cubes positions and normals, which is
// what keeps neighbouring blocks watertight; only redundant interior detail is
// dropped. Curved blocks (a small object) fail the planarity test and keep the
// full 3 cm resolution.
void simplifyPlanarPatch(BlockMesh& out, float voxelSize) {
    if (out.vertices.size() < 10 || out.indices.size() < 48) return;

    std::unordered_map<uint64_t, int> edgeUses;
    edgeUses.reserve(out.indices.size());
    for (size_t i = 0; i + 2 < out.indices.size(); i += 3) {
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = out.indices[i + k];
            const uint32_t b = out.indices[i + (k + 1) % 3];
            const uint64_t key = (static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
            ++edgeUses[key];
        }
    }

    std::unordered_map<uint32_t, std::array<uint32_t, 2>> links;
    std::unordered_map<uint32_t, int> linkCount;
    for (const auto& use : edgeUses) {
        if (use.second != 1) continue;
        const uint32_t a = static_cast<uint32_t>(use.first >> 32);
        const uint32_t b = static_cast<uint32_t>(use.first & 0xFFFFFFFFull);
        int& ca = linkCount[a];
        if (ca < 2) links[a][ca] = b;
        ++ca;
        int& cb = linkCount[b];
        if (cb < 2) links[b][cb] = a;
        ++cb;
    }
    if (linkCount.size() < 8) return;
    // A hole or a pinched corner means the patch is not one simple disk, so a
    // fan over the loop would cut across the real surface. Keep the detail.
    for (const auto& entry : linkCount) {
        if (entry.second != 2) return;
    }

    std::vector<uint32_t> loop;
    loop.reserve(linkCount.size());
    const uint32_t start = linkCount.begin()->first;
    uint32_t prev = start;
    uint32_t current = links[start][0];
    loop.push_back(start);
    while (current != start && loop.size() <= linkCount.size()) {
        loop.push_back(current);
        const auto& nbr = links[current];
        const uint32_t next = (nbr[0] == prev) ? nbr[1] : nbr[0];
        prev = current;
        current = next;
    }
    if (current != start || loop.size() != linkCount.size()) return;

    // A whole block is up to 8 voxels across, and a fan over that loop is a
    // single 20 cm facet whose vertex colours are interpolated across the span.
    // On screen that reads as a smeared, coarse panel, so only compact loops
    // (a few voxels wide) are collapsed; wide flat surfaces keep the marching
    // cubes tessellation, which also keeps their colour detail.
    float lo[3] = {1e9f, 1e9f, 1e9f};
    float hi[3] = {-1e9f, -1e9f, -1e9f};
    for (const uint32_t index : loop) {
        const MeshVertex& v = out.vertices[index];
        const float p[3] = {v.px, v.py, v.pz};
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
        }
    }
    const float ex = hi[0] - lo[0], ey = hi[1] - lo[1], ez = hi[2] - lo[2];
    if (std::sqrt(ex * ex + ey * ey + ez * ez) > 3.f * voxelSize) return;

    // Newell normal and centroid of the boundary loop.
    Vec3 area{0.f, 0.f, 0.f};
    Vec3 centroid{0.f, 0.f, 0.f};
    for (size_t i = 0; i < loop.size(); ++i) {
        const MeshVertex& p = out.vertices[loop[i]];
        const MeshVertex& q = out.vertices[loop[(i + 1) % loop.size()]];
        area.x += (p.py - q.py) * (p.pz + q.pz);
        area.y += (p.pz - q.pz) * (p.px + q.px);
        area.z += (p.px - q.px) * (p.py + q.py);
        centroid += Vec3{p.px, p.py, p.pz};
    }
    centroid = centroid * (1.f / static_cast<float>(loop.size()));
    const float areaLen = area.length();
    if (areaLen < 1e-6f) return;
    Vec3 planeN = area * (1.f / areaLen);

    Vec3 normalSum{0.f, 0.f, 0.f};
    for (const MeshVertex& v : out.vertices) normalSum += Vec3{v.nx, v.ny, v.nz};
    if (normalSum.dot(planeN) < 0.f) {
        planeN = -planeN;
        std::reverse(loop.begin(), loop.end());
    }

    // Every vertex, including the interior ones about to be removed, must lie
    // on the plane and agree with it. Anything above that is real geometry
    // (a curve, a bump) and must not be flattened away.
    const float tol = 0.75f * voxelSize;
    const float minCos = 0.966f;  // 15 degrees
    for (const MeshVertex& v : out.vertices) {
        const Vec3 p{v.px, v.py, v.pz};
        if (std::fabs((p - centroid).dot(planeN)) > tol) return;
        if (std::fabs(Vec3{v.nx, v.ny, v.nz}.dot(planeN)) < minCos) return;
    }

    // A fan is only correct for a convex loop; a concave one folds triangles
    // over each other, which is the overlap this pass is meant to remove.
    const Vec3 seed = std::fabs(planeN.x) < 0.9f ? Vec3{1.f, 0.f, 0.f} : Vec3{0.f, 1.f, 0.f};
    const Vec3 eu = (seed - planeN * planeN.dot(seed)).normalized();
    const Vec3 ev = cross(planeN, eu);
    auto project = [&](const MeshVertex& v) {
        const Vec3 p = Vec3{v.px, v.py, v.pz} - centroid;
        return Vec3{p.dot(eu), p.dot(ev), 0.f};
    };
    float sign = 0.f;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec3 a = project(out.vertices[loop[i]]);
        const Vec3 b = project(out.vertices[loop[(i + 1) % loop.size()]]);
        const Vec3 c = project(out.vertices[loop[(i + 2) % loop.size()]]);
        const Vec3 ab = b - a;
        const Vec3 bc = c - b;
        const float z = ab.x * bc.y - ab.y * bc.x;
        if (std::fabs(z) < 1e-9f) continue;
        if (sign == 0.f) {
            sign = z;
        } else if ((z > 0.f) != (sign > 0.f)) {
            return;
        }
    }
    if (sign == 0.f) return;

    BlockMesh simplified;
    simplified.vertices.reserve(loop.size());
    simplified.indices.reserve((loop.size() - 2) * 3);
    for (const uint32_t index : loop) simplified.vertices.push_back(out.vertices[index]);
    for (size_t i = 1; i + 1 < loop.size(); ++i) {
        simplified.indices.push_back(0);
        simplified.indices.push_back(static_cast<uint32_t>(i));
        simplified.indices.push_back(static_cast<uint32_t>(i + 1));
    }
    out = std::move(simplified);
}

struct CollapseCandidate {
    float len;
    uint32_t a, b;
    bool operator<(const CollapseCandidate& o) const { return len > o.len; }
};

inline uint64_t edgeKey(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(a) << 32) | b;
}

// Curvature-adaptive simplification of the merged surface. Flat regions - a
// wall, a floor, a table top - grow into facets up to 9 cm, while anything that
// turns keeps its 2 cm marching-cubes triangles: a merge is only accepted when
// both endpoint normals, every incident face normal and the surface error all
// agree, so a leaf, a mug rim or a corner refuses every collapse without needing
// a separate curvature estimate. Rim vertices (edges with a single face) never
// move, so silhouettes and hole borders stay exactly where the scan put them.
// The colour test keeps painted or textured detail at full resolution instead
// of letting one big facet blend it away.
void simplifyAdaptive(std::vector<MeshVertex>& verts, std::vector<uint32_t>& indices) {
    constexpr float kMaxEdge = 0.09f;
    constexpr float kMinCos = 0.995f;
    constexpr float kMinFaceCos = 0.99f;
    constexpr float kMaxError = 0.0025f;
    constexpr int kMaxColorDelta = 30;

    const size_t vertexCount = verts.size();
    const size_t faceCount = indices.size() / 3;
    if (faceCount < 256) return;

    std::unordered_map<uint64_t, uint32_t> uses;
    uses.reserve(faceCount * 2 + 1);
    for (size_t i = 0; i < faceCount; ++i) {
        for (int k = 0; k < 3; ++k) {
            ++uses[edgeKey(indices[i * 3 + k], indices[i * 3 + (k + 1) % 3])];
        }
    }

    std::vector<uint8_t> locked(vertexCount, 0);
    for (const auto& entry : uses) {
        if (entry.second != 1) continue;
        locked[static_cast<uint32_t>(entry.first >> 32)] = 1;
        locked[static_cast<uint32_t>(entry.first & 0xFFFFFFFFull)] = 1;
    }

    std::vector<std::vector<uint32_t>> incident(vertexCount);
    for (size_t i = 0; i < faceCount; ++i) {
        for (int k = 0; k < 3; ++k) {
            incident[indices[i * 3 + k]].push_back(static_cast<uint32_t>(i));
        }
    }

    std::vector<uint8_t> deadVertex(vertexCount, 0);
    std::vector<uint8_t> deadFace(faceCount, 0);
    std::priority_queue<CollapseCandidate> queue;

    auto normalDot = [&](uint32_t a, uint32_t b) {
        const MeshVertex& va = verts[a];
        const MeshVertex& vb = verts[b];
        return va.nx * vb.nx + va.ny * vb.ny + va.nz * vb.nz;
    };
    auto colorDelta = [&](uint32_t a, uint32_t b) {
        const MeshVertex& va = verts[a];
        const MeshVertex& vb = verts[b];
        return std::abs(static_cast<int>(va.r) - static_cast<int>(vb.r)) +
               std::abs(static_cast<int>(va.g) - static_cast<int>(vb.g)) +
               std::abs(static_cast<int>(va.b) - static_cast<int>(vb.b));
    };
    auto consider = [&](uint32_t a, uint32_t b) {
        if (a == b || deadVertex[a] || deadVertex[b] || locked[a] || locked[b]) return;
        if (normalDot(a, b) < kMinCos) return;
        if (colorDelta(a, b) > kMaxColorDelta) return;
        const MeshVertex& va = verts[a];
        const MeshVertex& vb = verts[b];
        const float dx = va.px - vb.px, dy = va.py - vb.py, dz = va.pz - vb.pz;
        const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len > kMaxEdge) return;
        queue.push(CollapseCandidate{len, a, b});
    };
    for (const auto& entry : uses) {
        if (entry.second != 2) continue;
        consider(static_cast<uint32_t>(entry.first >> 32),
                 static_cast<uint32_t>(entry.first & 0xFFFFFFFFull));
    }
    uses.clear();

    std::vector<uint32_t> nbrA, nbrB;
    while (!queue.empty()) {
        const CollapseCandidate c = queue.top();
        queue.pop();
        const uint32_t a = c.a, b = c.b;
        if (deadVertex[a] || deadVertex[b]) continue;
        if (normalDot(a, b) < kMinCos) continue;
        if (colorDelta(a, b) > kMaxColorDelta) continue;

        std::vector<uint32_t>& fa = incident[a];
        std::vector<uint32_t>& fb = incident[b];
        auto dropDead = [&](std::vector<uint32_t>& list) {
            list.erase(std::remove_if(list.begin(), list.end(),
                                      [&](uint32_t f) { return deadFace[f] != 0; }),
                       list.end());
        };
        dropDead(fa);
        dropDead(fb);

        uint32_t common[2] = {0, 0};
        int commonCount = 0;
        for (const uint32_t f : fa) {
            if (std::find(fb.begin(), fb.end(), f) == fb.end()) continue;
            if (commonCount == 2) {
                commonCount = 3;
                break;
            }
            common[commonCount++] = f;
        }
        if (commonCount != 2) continue;

        // Link condition: the two vertices may only have the shared pair of
        // neighbours, otherwise the collapse would create a non-manifold edge.
        nbrA.clear();
        nbrB.clear();
        for (const uint32_t f : fa) {
            if (f == common[0] || f == common[1]) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = indices[f * 3 + k];
                if (v != a && v != b) nbrA.push_back(v);
            }
        }
        for (const uint32_t f : fb) {
            if (f == common[0] || f == common[1]) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = indices[f * 3 + k];
                if (v != a && v != b) nbrB.push_back(v);
            }
        }
        int shared = 0;
        for (const uint32_t v : nbrB) {
            if (std::find(nbrA.begin(), nbrA.end(), v) != nbrA.end()) ++shared;
        }
        if (shared != 2) continue;

        const MeshVertex& va = verts[a];
        const MeshVertex& vb = verts[b];
        const float np[3] = {(va.px + vb.px) * 0.5f, (va.py + vb.py) * 0.5f,
                             (va.pz + vb.pz) * 0.5f};

        auto keepFace = [&](uint32_t f, uint32_t apex) {
            const uint32_t* tri = &indices[f * 3];
            uint32_t v0 = 0, v1 = 0;
            int found = 0;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = tri[k];
                if (v == apex) continue;
                if (found == 0) {
                    v0 = v;
                } else if (found == 1) {
                    v1 = v;
                } else {
                    return false;
                }
                ++found;
            }
            if (found != 2) return false;
            const MeshVertex& p0 = verts[v0];
            const MeshVertex& p1 = verts[v1];
            const MeshVertex& pa = verts[apex];
            float oldN[3], newN[3];
            const float ux = p1.px - p0.px, uy = p1.py - p0.py, uz = p1.pz - p0.pz;
            const float ax = pa.px - p0.px, ay = pa.py - p0.py, az = pa.pz - p0.pz;
            oldN[0] = uy * az - uz * ay;
            oldN[1] = uz * ax - ux * az;
            oldN[2] = ux * ay - uy * ax;
            const float bx = np[0] - p0.px, by = np[1] - p0.py, bz = np[2] - p0.pz;
            newN[0] = uy * bz - uz * by;
            newN[1] = uz * bx - ux * bz;
            newN[2] = ux * by - uy * bx;
            const float lo = std::sqrt(oldN[0] * oldN[0] + oldN[1] * oldN[1] + oldN[2] * oldN[2]);
            const float ln = std::sqrt(newN[0] * newN[0] + newN[1] * newN[1] + newN[2] * newN[2]);
            if (lo < 1e-9f || ln < 1e-9f) return false;
            const float cosA =
                (oldN[0] * newN[0] + oldN[1] * newN[1] + oldN[2] * newN[2]) / (lo * ln);
            if (cosA < kMinFaceCos) return false;
            const float d = (np[0] - p0.px) * oldN[0] + (np[1] - p0.py) * oldN[1] +
                            (np[2] - p0.pz) * oldN[2];
            return std::fabs(d / lo) <= kMaxError;
        };

        bool ok = true;
        for (const uint32_t f : fa) {
            if (f == common[0] || f == common[1]) continue;
            if (!keepFace(f, a)) {
                ok = false;
                break;
            }
        }
        if (ok) {
            for (const uint32_t f : fb) {
                if (f == common[0] || f == common[1]) continue;
                if (!keepFace(f, b)) {
                    ok = false;
                    break;
                }
            }
        }
        if (!ok) continue;

        float nx = va.nx + vb.nx, ny = va.ny + vb.ny, nz = va.nz + vb.nz;
        const float nl = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nl > 1e-6f) {
            nx /= nl;
            ny /= nl;
            nz /= nl;
        }
        const int rr = (static_cast<int>(va.r) + static_cast<int>(vb.r) + 1) / 2;
        const int gg = (static_cast<int>(va.g) + static_cast<int>(vb.g) + 1) / 2;
        const int bb = (static_cast<int>(va.b) + static_cast<int>(vb.b) + 1) / 2;
        MeshVertex& merged = verts[a];
        merged.px = np[0];
        merged.py = np[1];
        merged.pz = np[2];
        merged.nx = nx;
        merged.ny = ny;
        merged.nz = nz;
        merged.r = static_cast<uint8_t>(rr);
        merged.g = static_cast<uint8_t>(gg);
        merged.b = static_cast<uint8_t>(bb);

        deadVertex[b] = 1;
        deadFace[common[0]] = 1;
        deadFace[common[1]] = 1;
        for (const uint32_t f : fb) {
            if (deadFace[f]) continue;
            uint32_t* tri = &indices[f * 3];
            for (int k = 0; k < 3; ++k) {
                if (tri[k] == b) tri[k] = a;
            }
            fa.push_back(f);
        }
        incident[b].clear();

        nbrA.clear();
        for (const uint32_t f : fa) {
            if (deadFace[f]) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = indices[f * 3 + k];
                if (v != a) nbrA.push_back(v);
            }
        }
        for (const uint32_t v : nbrA) consider(a, v);
    }

    std::vector<uint32_t> remap(vertexCount, 0xFFFFFFFFu);
    std::vector<uint32_t> kept;
    kept.reserve(faceCount);
    for (size_t i = 0; i < faceCount; ++i) {
        if (deadFace[i]) continue;
        const uint32_t* tri = &indices[i * 3];
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) continue;
        kept.push_back(static_cast<uint32_t>(i));
        remap[tri[0]] = 0;
        remap[tri[1]] = 0;
        remap[tri[2]] = 0;
    }

    std::vector<MeshVertex> newVerts;
    newVerts.reserve(vertexCount);
    for (size_t i = 0; i < vertexCount; ++i) {
        if (deadVertex[i] || remap[i] == 0xFFFFFFFFu) continue;
        remap[i] = static_cast<uint32_t>(newVerts.size());
        newVerts.push_back(verts[i]);
    }
    std::vector<uint32_t> newIndices;
    newIndices.reserve(kept.size() * 3);
    for (const uint32_t f : kept) {
        for (int k = 0; k < 3; ++k) newIndices.push_back(remap[indices[f * 3 + k]]);
    }

    verts.swap(newVerts);
    indices.swap(newIndices);
}

}  // namespace

void MeshBuilder::extractBlock(const VoxelHash& hash, const VoxelBlock& block,
                               BlockMesh& out) const {
    out.vertices.clear();
    out.indices.clear();

    const float vs = hash.voxelSize();
    const int baseX = block.bx * VoxelBlock::kSize;
    const int baseY = block.by * VoxelBlock::kSize;
    const int baseZ = block.bz * VoxelBlock::kSize;

    // Cubes use grid positions 0..8 and their gradient reaches -1..9, so the
    // smoothed field spans -1..9 and needs raw samples at -2..10.
    constexpr int kRaw = 13;
    constexpr int kRawOff = 2;
    std::array<CachedSample, kRaw * kRaw * kRaw> raw;
    for (int z = -kRawOff; z <= kRaw - kRawOff - 1; ++z) {
        for (int y = -kRawOff; y <= kRaw - kRawOff - 1; ++y) {
            for (int x = -kRawOff; x <= kRaw - kRawOff - 1; ++x) {
                float tsdf = 1.f;
                bool known = false;
                uint8_t rgb[3] = {200, 200, 200};
                uint8_t weight = 0;
                hash.sampleVoxel(baseX + x, baseY + y, baseZ + z, tsdf, known, rgb, &weight);
                CachedSample& s =
                    raw[static_cast<size_t>((z + kRawOff) * kRaw + (y + kRawOff)) * kRaw +
                        (x + kRawOff)];
                s.tsdf = known ? tsdf : 1.f;
                s.weight = known ? weight : 0;
                s.rgb[0] = rgb[0];
                s.rgb[1] = rgb[1];
                s.rgb[2] = rgb[2];
            }
        }
    }
    auto rawAt = [&](int x, int y, int z) -> const CachedSample& {
        return raw[static_cast<size_t>((z + kRawOff) * kRaw + (y + kRawOff)) * kRaw +
                   (x + kRawOff)];
    };

    // The fused TSDF carries the depth sensor's noise, so the raw field is
    // lumpy at the 2-3 voxel scale and the headlight shading turns that into
    // speckle. A [1,2,1]^3 blend of the measured neighbourhood removes most of
    // it while moving a flat surface by well under a millimetre. A voxel the
    // integrator never wrote (one invalid depth pixel skips its whole cube)
    // inherits the mean of its measured neighbours, which closes pinholes
    // without inventing anything: the sign, and therefore the surface, still
    // comes from real observations. The neighbour count is set high (18 of 26,
    // not the bare majority) because a low threshold also fills the half-space
    // beside a silhouette, where the missing voxels are not a dropout but the
    // empty world behind the object -- that fill was what grew a skirt around
    // every object.
    constexpr int kField = 11;
    constexpr int kFieldOff = 1;
    std::array<FieldSample, kField * kField * kField> field;
    for (int z = -kFieldOff; z <= kField - kFieldOff - 1; ++z) {
        for (int y = -kFieldOff; y <= kField - kFieldOff - 1; ++y) {
            for (int x = -kFieldOff; x <= kField - kFieldOff - 1; ++x) {
                float sum = 0.f, wsum = 0.f;
                float rsum = 0.f, gsum = 0.f, bsum = 0.f;
                int measured = 0;
                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const CachedSample& s = rawAt(x + dx, y + dy, z + dz);
                            if (s.weight == 0) continue;
                            const float w =
                                static_cast<float>(kKernel[std::abs(dx) + std::abs(dy) + std::abs(dz)]);
                            sum += s.tsdf * w;
                            wsum += w;
                            rsum += static_cast<float>(s.rgb[0]) * w;
                            gsum += static_cast<float>(s.rgb[1]) * w;
                            bsum += static_cast<float>(s.rgb[2]) * w;
                            if (dx != 0 || dy != 0 || dz != 0) ++measured;
                        }
                    }
                }
                if (wsum <= 0.f) continue;

                const CachedSample& self = rawAt(x, y, z);
                FieldSample& f =
                    field[static_cast<size_t>((z + kFieldOff) * kField + (y + kFieldOff)) *
                              kField +
                          (x + kFieldOff)];
                const bool fill = self.weight == 0 && measured >= 18;
                if (self.weight == 0 && !fill) continue;

                f.tsdf = sum / wsum;
                f.valid = true;
                if (!fill) {
                    f.weight = self.weight;
                    f.rgb[0] = self.rgb[0];
                    f.rgb[1] = self.rgb[1];
                    f.rgb[2] = self.rgb[2];
                } else {
                    // Two observations is the threshold marching cubes already
                    // uses for emitting a triangle.
                    f.weight = 2;
                    f.rgb[0] = static_cast<uint8_t>(rsum / wsum);
                    f.rgb[1] = static_cast<uint8_t>(gsum / wsum);
                    f.rgb[2] = static_cast<uint8_t>(bsum / wsum);
                }
            }
        }
    }
    auto fieldAt = [&](int x, int y, int z) -> const FieldSample& {
        return field[static_cast<size_t>((z + kFieldOff) * kField + (y + kFieldOff)) * kField +
                     (x + kFieldOff)];
    };
    // Central-difference gradient of the smoothed field, in local grid units.
    // It depends only on the fused TSDF, so neighbouring blocks produce the
    // same normal for a shared voxel and a block seam is not shaded as a crease.
    auto gradAt = [&](int x, int y, int z) -> Vec3 {
        return Vec3{fieldAt(x + 1, y, z).tsdf - fieldAt(x - 1, y, z).tsdf,
                    fieldAt(x, y + 1, z).tsdf - fieldAt(x, y - 1, z).tsdf,
                    fieldAt(x, y, z + 1).tsdf - fieldAt(x, y, z - 1).tsdf};
    };

    std::unordered_map<uint32_t, uint32_t> edgeVertex;
    edgeVertex.reserve(1024);

    for (int z = 0; z < VoxelBlock::kSize; ++z) {
        for (int y = 0; y < VoxelBlock::kSize; ++y) {
            for (int x = 0; x < VoxelBlock::kSize; ++x) {
                float values[8];
                bool known = true;
                const FieldSample* corners[8];
                for (int c = 0; c < 8; ++c) {
                    const FieldSample& s = fieldAt(x + kCorner[c][0], y + kCorner[c][1],
                                                   z + kCorner[c][2]);
                    values[c] = s.tsdf;
                    corners[c] = &s;
                    if (!s.valid) known = false;
                }
                if (!known) continue;

                int cubeIndex = 0;
                for (int c = 0; c < 8; ++c) {
                    if (values[c] < 0.f) cubeIndex |= (1 << c);
                }
                if (mc::kEdgeTable[cubeIndex] == 0) continue;

                const int8_t* tri = mc::kTriTable[cubeIndex];
                for (int t = 0; t < 15 && tri[t] >= 0; t += 3) {
                    // Two-observation rule: a surface edge must have been seen by
                    // at least two integrated frames. Single-frame depth is noisy
                    // enough to produce floating shards, so those are dropped.
                    bool emit = true;
                    for (int k = 0; k < 3 && emit; ++k) {
                        const int edge = tri[t + k];
                        const int a = kEdge[edge][0];
                        const int b = kEdge[edge][1];
                        if (corners[a]->weight < 2 || corners[b]->weight < 2) emit = false;
                    }
                    if (!emit) continue;

                    for (int k = 0; k < 3; ++k) {
                        const int edge = tri[t + k];
                        const uint32_t key = static_cast<uint32_t>(
                            ((z * 8 + y) * 8 + x) * 12 + edge);
                        auto found = edgeVertex.find(key);
                        uint32_t index;
                        if (found != edgeVertex.end()) {
                            index = found->second;
                        } else {
                            const int a = kEdge[edge][0];
                            const int b = kEdge[edge][1];
                            const float va = values[a];
                            const float vb = values[b];
                            float frac = 0.5f;
                            if (std::fabs(va - vb) > 1e-6f) frac = va / (va - vb);
                            frac = clampf(frac, 0.f, 1.f);

                            const float gax = static_cast<float>(baseX + x + kCorner[a][0]);
                            const float gay = static_cast<float>(baseY + y + kCorner[a][1]);
                            const float gaz = static_cast<float>(baseZ + z + kCorner[a][2]);
                            const float gbx = static_cast<float>(baseX + x + kCorner[b][0]);
                            const float gby = static_cast<float>(baseY + y + kCorner[b][1]);
                            const float gbz = static_cast<float>(baseZ + z + kCorner[b][2]);

                            MeshVertex v;
                            v.px = (gax + (gbx - gax) * frac + 0.5f) * vs;
                            v.py = (gay + (gby - gay) * frac + 0.5f) * vs;
                            v.pz = (gaz + (gbz - gaz) * frac + 0.5f) * vs;
                            // The normal is interpolated between the two edge
                            // corners, so a shared edge yields the same normal
                            // in both cubes and the surface shades smoothly
                            // instead of showing the grid as facets.
                            const Vec3 na = gradAt(x + kCorner[a][0], y + kCorner[a][1],
                                                   z + kCorner[a][2]);
                            const Vec3 nb = gradAt(x + kCorner[b][0], y + kCorner[b][1],
                                                   z + kCorner[b][2]);
                            const Vec3 nrm = (na + (nb - na) * frac).normalized();
                            v.nx = nrm.x;
                            v.ny = nrm.y;
                            v.nz = nrm.z;
                            const FieldSample& ca = *corners[a];
                            const FieldSample& cb = *corners[b];
                            v.r = static_cast<uint8_t>(ca.rgb[0] + (cb.rgb[0] - ca.rgb[0]) * frac);
                            v.g = static_cast<uint8_t>(ca.rgb[1] + (cb.rgb[1] - ca.rgb[1]) * frac);
                            v.b = static_cast<uint8_t>(ca.rgb[2] + (cb.rgb[2] - ca.rgb[2]) * frac);
                            v.a = 255;
                            index = static_cast<uint32_t>(out.vertices.size());
                            out.vertices.push_back(v);
                            edgeVertex.emplace(key, index);
                        }
                        out.indices.push_back(index);
                    }
                }
            }
        }
    }

    simplifyPlanarPatch(out, vs);
}

void MeshBuilder::rebuildAll(const VoxelHash& hash) {
    blockMeshes_.clear();
    for (const auto& owned : hash.blocks()) {
        if (!owned) continue;
        BlockMesh mesh;
        extractBlock(hash, *owned, mesh);
        if (!mesh.indices.empty()) {
            blockMeshes_[blockQuadKey(owned->bx, owned->by, owned->bz)] = std::move(mesh);
        }
    }
    rebuildSnapshot();
}

void MeshBuilder::extractDirty(const VoxelHash& hash,
                               const std::vector<VoxelBlock*>& dirty) {
    if (dirty.empty()) return;

    for (VoxelBlock* block : dirty) {
        if (!block) continue;
        const uint64_t key = blockQuadKey(block->bx, block->by, block->bz);
        BlockMesh mesh;
        extractBlock(hash, *block, mesh);
        if (mesh.indices.empty()) {
            blockMeshes_.erase(key);
        } else {
            blockMeshes_[key] = std::move(mesh);
        }
        block->state = VoxelState::NORMAL;
    }

    rebuildSnapshot();
}

void MeshBuilder::rebuildSnapshot() {
    // The snapshot is the whole fused surface. There is deliberately no
    // "keep only the largest island" pass: a chair, a plant or a second wall is
    // a legitimate connected component that has not been bridged to the main
    // body yet, and deleting it was what made earlier scans lose objects. Noise
    // shards are already prevented before this point by the two-observation
    // rule in extractBlock and by visibility carving.
    if (blockMeshes_.empty()) {
        auto empty = std::make_shared<MeshData>();
        empty->version = ++version_;
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = std::move(empty);
        return;
    }

    size_t totalVertices = 0;
    size_t totalIndices = 0;
    for (const auto& entry : blockMeshes_) {
        totalVertices += entry.second.vertices.size();
        totalIndices += entry.second.indices.size();
    }
    if (totalIndices == 0) {
        auto empty = std::make_shared<MeshData>();
        empty->version = ++version_;
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = std::move(empty);
        return;
    }

    auto data = std::make_shared<MeshData>();
    data->vertices.reserve(totalVertices);
    data->indices.reserve(totalIndices);
    for (const auto& entry : blockMeshes_) {
        const BlockMesh& mesh = entry.second;
        if (mesh.indices.empty()) continue;
        const uint32_t base = static_cast<uint32_t>(data->vertices.size());
        data->vertices.insert(data->vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        for (uint32_t index : mesh.indices) data->indices.push_back(base + index);
    }

    const uint32_t vertexCount = static_cast<uint32_t>(data->vertices.size());
    const uint32_t triangleCount = static_cast<uint32_t>(data->indices.size() / 3);

    // Marching cubes derives a shared edge vertex from the same grid positions
    // in every block, so joining at 1 mm is exact for blocks that really touch
    // and keeps vertices that differ by sensor noise apart.
    std::vector<uint32_t> weld(vertexCount);
    std::unordered_map<uint64_t, uint32_t> welded;
    welded.reserve(vertexCount * 2 + 1);
    for (uint32_t i = 0; i < vertexCount; ++i) {
        const MeshVertex& v = data->vertices[i];
        const uint64_t x = static_cast<uint64_t>(static_cast<int64_t>(std::llround(v.px * 1000.f))) & 0x1FFFFFull;
        const uint64_t y = static_cast<uint64_t>(static_cast<int64_t>(std::llround(v.py * 1000.f))) & 0x1FFFFFull;
        const uint64_t z = static_cast<uint64_t>(static_cast<int64_t>(std::llround(v.pz * 1000.f))) & 0x1FFFFFull;
        const uint64_t key = x | (y << 21) | (z << 42);
        const auto inserted = welded.emplace(key, static_cast<uint32_t>(welded.size()));
        weld[i] = inserted.first->second;
    }

    // Publish the welded mesh, not one copy of every vertex per block: the
    // adaptive pass below needs to see the real topology at block seams, and the
    // renderer saves half the vertex uploads.
    std::shared_ptr<MeshData> filtered = std::make_shared<MeshData>();
    std::vector<uint32_t> remap(welded.size(), 0xFFFFFFFFu);
    filtered->vertices.reserve(welded.size());
    filtered->indices.reserve(data->indices.size());
    for (uint32_t t = 0; t < triangleCount; ++t) {
        for (int k = 0; k < 3; ++k) {
            const uint32_t vi = data->indices[t * 3 + k];
            const uint32_t w = weld[vi];
            if (remap[w] == 0xFFFFFFFFu) {
                remap[w] = static_cast<uint32_t>(filtered->vertices.size());
                filtered->vertices.push_back(data->vertices[vi]);
            }
            filtered->indices.push_back(remap[w]);
        }
    }

    simplifyAdaptive(filtered->vertices, filtered->indices);

    filtered->version = ++version_;
    filtered->triangleCount = static_cast<uint32_t>(filtered->indices.size() / 3);

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = std::move(filtered);
}

void MeshBuilder::pruneBlocks(const VoxelHash& hash) {
    bool changed = false;
    for (auto it = blockMeshes_.begin(); it != blockMeshes_.end();) {
        const uint64_t key = it->first;
        const int ux = static_cast<int>((key >> 42) & 0x1FFFFF);
        const int uy = static_cast<int>((key >> 21) & 0x1FFFFF);
        const int uz = static_cast<int>(key & 0x1FFFFF);
        const int bx = (ux & 0x100000) ? ux - 0x200000 : ux;
        const int by = (uy & 0x100000) ? uy - 0x200000 : uy;
        const int bz = (uz & 0x100000) ? uz - 0x200000 : uz;
        if (hash.get(bx, by, bz) == nullptr) {
            it = blockMeshes_.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    if (changed) rebuildSnapshot();
}

void MeshBuilder::reset() {
    blockMeshes_.clear();
    auto data = std::make_shared<MeshData>();
    data->version = ++version_;
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = std::move(data);
}

std::shared_ptr<const MeshData> MeshBuilder::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

uint32_t MeshBuilder::version() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_->version;
}

int MeshBuilder::vertexCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(snapshot_->vertices.size());
}

int MeshBuilder::triangleCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(snapshot_->triangleCount);
}

uint32_t MeshBuilder::indexCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(snapshot_->indices.size());
}

}  // namespace recon
