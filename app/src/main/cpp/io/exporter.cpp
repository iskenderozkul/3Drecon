#include "io/exporter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace recon {
namespace {

// The mesher caches each 8^3 block independently, so a voxel on a block seam
// exists once per neighbouring block: 495k triangles came out as 1.0M vertices
// for 279k distinct positions. Welding them makes the file usable in external
// tools and lets the normals be averaged across the seam.
struct WeldedMesh {
    std::vector<float> pos;
    std::vector<float> nrm;
    std::vector<uint8_t> col;
    std::vector<uint32_t> index;

    size_t vertexCount() const { return pos.size() / 3; }
    size_t faceCount() const { return index.size() / 3; }
};

uint64_t weldKey(const MeshVertex& v) {
    // 1 mm grid, 21 bits per axis (+/- 1048 m) matches the TSDF key range.
    constexpr long long kLimit = (1LL << 20) - 1;
    const long long x = static_cast<long long>(std::llround(v.px * 1000.0f));
    const long long y = static_cast<long long>(std::llround(v.py * 1000.0f));
    const long long z = static_cast<long long>(std::llround(v.pz * 1000.0f));
    if (x < -kLimit || x > kLimit || y < -kLimit || y > kLimit || z < -kLimit ||
        z > kLimit) {
        return 0;
    }
    const uint64_t ux = static_cast<uint64_t>(x + kLimit) & 0x1FFFFF;
    const uint64_t uy = static_cast<uint64_t>(y + kLimit) & 0x1FFFFF;
    const uint64_t uz = static_cast<uint64_t>(z + kLimit) & 0x1FFFFF;
    return (ux << 42) | (uy << 21) | uz;
}

uint32_t weldVertex(WeldedMesh& out, std::unordered_map<uint64_t, uint32_t>& lookup,
                    const MeshVertex& v) {
    const uint64_t key = weldKey(v);
    if (key != 0) {
        const auto found = lookup.find(key);
        if (found != lookup.end()) {
            const uint32_t idx = found->second;
            out.nrm[idx * 3 + 0] += v.nx;
            out.nrm[idx * 3 + 1] += v.ny;
            out.nrm[idx * 3 + 2] += v.nz;
            return idx;
        }
    }
    const uint32_t idx = static_cast<uint32_t>(out.vertexCount());
    out.pos.push_back(v.px);
    out.pos.push_back(v.py);
    out.pos.push_back(v.pz);
    out.nrm.push_back(v.nx);
    out.nrm.push_back(v.ny);
    out.nrm.push_back(v.nz);
    out.col.push_back(v.r);
    out.col.push_back(v.g);
    out.col.push_back(v.b);
    if (key != 0) lookup.emplace(key, idx);
    return idx;
}

uint32_t findRoot(std::vector<uint32_t>& parent, uint32_t i) {
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

// Drops faces that carry no area and components smaller than kMinFaces. The
// threshold is deliberately tiny: a few triangles are degenerate shards, while
// a leaf or a chair leg is real geometry the file must keep.
void filterMesh(WeldedMesh& mesh) {
    constexpr uint32_t kMinFaces = 8;

    std::vector<uint32_t> kept;
    kept.reserve(mesh.index.size());
    for (size_t i = 0; i + 2 < mesh.index.size(); i += 3) {
        const uint32_t a = mesh.index[i];
        const uint32_t b = mesh.index[i + 1];
        const uint32_t c = mesh.index[i + 2];
        if (a == b || b == c || a == c) continue;
        const float* pa = &mesh.pos[a * 3];
        const float* pb = &mesh.pos[b * 3];
        const float* pc = &mesh.pos[c * 3];
        const float ux = pb[0] - pa[0], uy = pb[1] - pa[1], uz = pb[2] - pa[2];
        const float vx = pc[0] - pa[0], vy = pc[1] - pa[1], vz = pc[2] - pa[2];
        const float cx = uy * vz - uz * vy;
        const float cy = uz * vx - ux * vz;
        const float cz = ux * vy - uy * vx;
        if (cx * cx + cy * cy + cz * cz < 4e-10f) continue;
        kept.push_back(a);
        kept.push_back(b);
        kept.push_back(c);
    }

    const size_t verts = mesh.vertexCount();
    std::vector<uint32_t> parent(verts);
    for (uint32_t i = 0; i < verts; ++i) parent[i] = i;
    for (size_t i = 0; i + 2 < kept.size(); i += 3) {
        const uint32_t ra = findRoot(parent, kept[i]);
        const uint32_t rb = findRoot(parent, kept[i + 1]);
        if (rb != ra) parent[rb] = ra;
        const uint32_t rc = findRoot(parent, kept[i + 2]);
        if (rc != ra) parent[rc] = ra;
    }
    // Second pass so every chain is resolved after the merges above.
    for (uint32_t i = 0; i < verts; ++i) parent[i] = findRoot(parent, i);

    std::vector<uint32_t> facesPerRoot(verts, 0);
    for (size_t i = 0; i + 2 < kept.size(); i += 3) {
        ++facesPerRoot[parent[kept[i]]];
    }

    std::vector<uint32_t> filtered;
    filtered.reserve(kept.size());
    size_t dropped = 0;
    for (size_t i = 0; i + 2 < kept.size(); i += 3) {
        if (facesPerRoot[parent[kept[i]]] >= kMinFaces) {
            filtered.push_back(kept[i]);
            filtered.push_back(kept[i + 1]);
            filtered.push_back(kept[i + 2]);
        } else {
            ++dropped;
        }
    }
    // Never trade a real surface for a tidy one: if the filter would remove a
    // large share of the mesh, keep the original faces.
    if (dropped * 4 < kept.size() / 3) {
        mesh.index.swap(filtered);
    } else {
        mesh.index.swap(kept);
    }
}

inline uint64_t edgeKey(uint32_t a, uint32_t b) {
    const uint32_t lo = std::min(a, b);
    const uint32_t hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | hi;
}

// Flat-region decimation. A wide wall and a small curved object carry the same
// 3 cm triangles out of marching cubes; this pass collapses edges only where
// the surface is already locally flat, so a wall ends up with ~6 cm triangles
// while a small object keeps its detail. The collapse is guarded so it cannot
// tear or fold the surface:
//  - link condition: the two one-rings may meet only in the two corners they
//    share, otherwise the merge would create a non-manifold fin,
//  - every surviving incident triangle must keep its orientation and stay
//    within a millimetre scale of its old plane,
//  - open-boundary vertices never move, so silhouettes and ripped edges are
//    preserved exactly (verified: boundary edge count is unchanged).
// Edge length is capped so a flat wall cannot turn into a handful of huge
// triangles that no longer follow its gentle waviness.
void decimateFlatRegions(WeldedMesh& mesh) {
    // Adaptive resolution: flat areas are supposed to read as large facets (a
    // wall is a wall), while curved and cornered geometry keeps the grid-scale
    // triangles. The edge cap only bounds how big one facet may get; the plane
    // error and normal tests decide whether a merge is admissible at all, so a
    // leaf, a mug rim or a chair leg stays fine automatically - on a 3 cm
    // radius curve the 2.5 mm sagitta is exceeded after one edge. The colour
    // test keeps a printed surface (a poster, a tiled floor) from turning into
    // one blended panel.
    constexpr float kMaxEdge = 0.085f;
    constexpr float kMaxError = 0.0025f;
    constexpr float kMinCos = 0.995f;

    const size_t verts = mesh.vertexCount();
    const size_t faces = mesh.faceCount();
    if (verts < 64 || faces < 128) return;

    std::vector<uint32_t>& index = mesh.index;
    std::unordered_map<uint64_t, uint32_t> uses;
    uses.reserve(faces * 2);
    for (size_t i = 0; i < faces; ++i) {
        for (int k = 0; k < 3; ++k) ++uses[edgeKey(index[i * 3 + k], index[i * 3 + (k + 1) % 3])];
    }

    std::vector<uint8_t> boundary(verts, 0);
    for (const auto& entry : uses) {
        if (entry.second != 1) continue;
        boundary[static_cast<uint32_t>(entry.first >> 32)] = 1;
        boundary[static_cast<uint32_t>(entry.first & 0xFFFFFFFFull)] = 1;
    }

    std::vector<std::vector<uint32_t>> incident(verts);
    for (size_t i = 0; i < faces; ++i) {
        for (int k = 0; k < 3; ++k) incident[index[i * 3 + k]].push_back(static_cast<uint32_t>(i));
    }

    std::vector<uint8_t> dead(verts, 0);
    std::vector<uint8_t> deadFace(faces, 0);

    struct Candidate {
        float len;
        uint32_t a, b;
        bool operator<(const Candidate& o) const { return len > o.len; }
    };
    std::priority_queue<Candidate> queue;

    auto normalDot = [&](uint32_t a, uint32_t b) {
        const float* na = &mesh.nrm[a * 3];
        const float* nb = &mesh.nrm[b * 3];
        return na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2];
    };
    auto consider = [&](uint32_t a, uint32_t b) {
        if (a == b || dead[a] || dead[b] || boundary[a] || boundary[b]) return;
        if (normalDot(a, b) < kMinCos) return;
        const int dr = std::abs(static_cast<int>(mesh.col[a * 3]) - static_cast<int>(mesh.col[b * 3]));
        const int dg = std::abs(static_cast<int>(mesh.col[a * 3 + 1]) - static_cast<int>(mesh.col[b * 3 + 1]));
        const int db = std::abs(static_cast<int>(mesh.col[a * 3 + 2]) - static_cast<int>(mesh.col[b * 3 + 2]));
        if (dr + dg + db > 45) return;
        const float* pa = &mesh.pos[a * 3];
        const float* pb = &mesh.pos[b * 3];
        const float dx = pa[0] - pb[0], dy = pa[1] - pb[1], dz = pa[2] - pb[2];
        const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len > kMaxEdge) return;
        queue.push(Candidate{len, a, b});
    };
    for (const auto& entry : uses) {
        if (entry.second != 2) continue;
        consider(static_cast<uint32_t>(entry.first >> 32),
                 static_cast<uint32_t>(entry.first & 0xFFFFFFFFull));
    }
    uses.clear();

    std::vector<uint32_t> nbrA, nbrB;
    size_t collapsed = 0;
    while (!queue.empty()) {
        const Candidate c = queue.top();
        queue.pop();
        const uint32_t a = c.a, b = c.b;
        if (dead[a] || dead[b]) continue;
        if (normalDot(a, b) < kMinCos) continue;

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

        // One-ring of each vertex without the shared pair.
        nbrA.clear();
        nbrB.clear();
        for (const uint32_t f : fa) {
            if (f == common[0] || f == common[1]) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = index[f * 3 + k];
                if (v != a && v != b) nbrA.push_back(v);
            }
        }
        for (const uint32_t f : fb) {
            if (f == common[0] || f == common[1]) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = index[f * 3 + k];
                if (v != a && v != b) nbrB.push_back(v);
            }
        }
        int shared = 0;
        for (const uint32_t v : nbrB) {
            if (std::find(nbrA.begin(), nbrA.end(), v) != nbrA.end()) ++shared;
        }
        if (shared != 2) continue;

        const float* pa = &mesh.pos[a * 3];
        const float* pb = &mesh.pos[b * 3];
        const float np[3] = {(pa[0] + pb[0]) * 0.5f, (pa[1] + pb[1]) * 0.5f,
                             (pa[2] + pb[2]) * 0.5f};

        auto keepFace = [&](uint32_t f, uint32_t apex) {
            const uint32_t* tri = &index[f * 3];
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
            const float* p0 = &mesh.pos[v0 * 3];
            const float* p1 = &mesh.pos[v1 * 3];
            float oldN[3], newN[3];
            const float ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
            const float ax = mesh.pos[apex * 3] - p0[0];
            const float ay = mesh.pos[apex * 3 + 1] - p0[1];
            const float az = mesh.pos[apex * 3 + 2] - p0[2];
            oldN[0] = uy * az - uz * ay;
            oldN[1] = uz * ax - ux * az;
            oldN[2] = ux * ay - uy * ax;
            const float bx = np[0] - p0[0], by = np[1] - p0[1], bz = np[2] - p0[2];
            newN[0] = uy * bz - uz * by;
            newN[1] = uz * bx - ux * bz;
            newN[2] = ux * by - uy * bx;
            const float lo = std::sqrt(oldN[0] * oldN[0] + oldN[1] * oldN[1] + oldN[2] * oldN[2]);
            const float ln = std::sqrt(newN[0] * newN[0] + newN[1] * newN[1] + newN[2] * newN[2]);
            if (lo < 1e-9f || ln < 1e-9f) return false;
            const float cosA =
                (oldN[0] * newN[0] + oldN[1] * newN[1] + oldN[2] * newN[2]) / (lo * ln);
            if (cosA < kMinCos) return false;
            const float d = (np[0] - p0[0]) * oldN[0] + (np[1] - p0[1]) * oldN[1] +
                            (np[2] - p0[2]) * oldN[2];
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

        const float* na = &mesh.nrm[a * 3];
        const float* nb = &mesh.nrm[b * 3];
        float nx = na[0] + nb[0], ny = na[1] + nb[1], nz = na[2] + nb[2];
        const float nl = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nl > 1e-6f) {
            nx /= nl;
            ny /= nl;
            nz /= nl;
        }
        for (int k = 0; k < 3; ++k) {
            mesh.pos[a * 3 + k] = np[k];
            mesh.nrm[a * 3 + k] = k == 0 ? nx : (k == 1 ? ny : nz);
            mesh.col[a * 3 + k] = static_cast<uint8_t>(
                (static_cast<int>(mesh.col[a * 3 + k]) + mesh.col[b * 3 + k] + 1) / 2);
        }
        dead[b] = 1;
        deadFace[common[0]] = 1;
        deadFace[common[1]] = 1;
        for (const uint32_t f : fb) {
            if (deadFace[f]) continue;
            uint32_t* tri = &index[f * 3];
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
                const uint32_t v = index[f * 3 + k];
                if (v != a) nbrA.push_back(v);
            }
        }
        for (const uint32_t v : nbrA) consider(a, v);
        ++collapsed;
    }

    if (collapsed == 0) return;

    std::vector<uint32_t> remap(verts, 0xFFFFFFFFu);
    std::vector<uint32_t> keptFaces;
    keptFaces.reserve(faces);
    for (size_t i = 0; i < faces; ++i) {
        if (deadFace[i]) continue;
        const uint32_t* tri = &index[i * 3];
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) continue;
        keptFaces.push_back(static_cast<uint32_t>(i));
        remap[tri[0]] = 0;
        remap[tri[1]] = 0;
        remap[tri[2]] = 0;
    }

    std::vector<float> newPos, newNrm;
    std::vector<uint8_t> newCol;
    newPos.reserve(mesh.pos.size());
    newNrm.reserve(mesh.nrm.size());
    newCol.reserve(mesh.col.size());
    for (size_t i = 0; i < verts; ++i) {
        if (remap[i] == 0xFFFFFFFFu) continue;
        remap[i] = static_cast<uint32_t>(newPos.size() / 3);
        newPos.push_back(mesh.pos[i * 3]);
        newPos.push_back(mesh.pos[i * 3 + 1]);
        newPos.push_back(mesh.pos[i * 3 + 2]);
        newNrm.push_back(mesh.nrm[i * 3]);
        newNrm.push_back(mesh.nrm[i * 3 + 1]);
        newNrm.push_back(mesh.nrm[i * 3 + 2]);
        newCol.push_back(mesh.col[i * 3]);
        newCol.push_back(mesh.col[i * 3 + 1]);
        newCol.push_back(mesh.col[i * 3 + 2]);
    }

    std::vector<uint32_t> newIndex;
    newIndex.reserve(keptFaces.size() * 3);
    for (const uint32_t f : keptFaces) {
        for (int k = 0; k < 3; ++k) newIndex.push_back(remap[index[f * 3 + k]]);
    }
    mesh.pos.swap(newPos);
    mesh.nrm.swap(newNrm);
    mesh.col.swap(newCol);
    index.swap(newIndex);
}

// Small holes are what turns a scanned object into a sieve: a single rejected
// triangle leaves a three-edge rim, and a mesh with two thousand of them reads
// as disconnected even though every neighbourhood is locally fine. This closes
// rims that are short enough to be noise (a dropout, a decimation slit, a
// silhouette that two views met on) with a fan to their centroid; it changes no
// existing vertex, so it cannot move the surface. Long rims are left open: those
// are real silhouettes where the scan simply has no data behind the object.
void fillSmallHoles(WeldedMesh& mesh) {
    constexpr size_t kMaxLoopVertices = 32u;
    // Measured on a real export: 76 % of closed rims are under 6 cm and are
    // tears, while everything past 10 cm is a real silhouette whose total rim
    // length jumps by 20 m per 5 cm of threshold. Filling those would draw
    // membranes across the empty space behind an object.
    constexpr float kMaxPerimeter = 0.12f;

    std::unordered_map<uint64_t, int> edgeUses;
    edgeUses.reserve(mesh.index.size());
    for (size_t i = 0; i + 2 < mesh.index.size(); i += 3) {
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = mesh.index[i + k];
            const uint32_t b = mesh.index[i + (k + 1) % 3];
            ++edgeUses[edgeKey(a, b)];
        }
    }

    // Directed boundary edges: a triangle wound (a,b,c) has the outside of the
    // surface on its right, so the rim's successor of b is a.
    std::unordered_map<uint32_t, uint32_t> succ;
    std::unordered_set<uint32_t> ambiguous;
    succ.reserve(4096);
    for (size_t i = 0; i + 2 < mesh.index.size(); i += 3) {
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = mesh.index[i + k];
            const uint32_t b = mesh.index[i + (k + 1) % 3];
            if (edgeUses[edgeKey(a, b)] != 1) continue;
            if (!succ.emplace(b, a).second) ambiguous.insert(b);
        }
    }
    if (succ.empty()) return;

    // Existing rim vertices keep their indices; the fan centres are appended
    // after them, so the only indices that need an offset are the new ones.
    const uint32_t base = static_cast<uint32_t>(mesh.vertexCount());

    std::unordered_set<uint32_t> visited;
    visited.reserve(succ.size());
    std::vector<uint32_t> loop;
    std::vector<float> fillPos, fillNrm;
    std::vector<uint8_t> fillCol;
    std::vector<uint32_t> fillIndex;

    for (const auto& start : succ) {
        if (visited.count(start.first) || ambiguous.count(start.first)) continue;
        loop.clear();
        uint32_t v = start.first;
        float perimeter = 0.f;
        bool ok = true;
        while (visited.insert(v).second) {
            loop.push_back(v);
            if (loop.size() > kMaxLoopVertices) break;
            if (ambiguous.count(v)) {
                ok = false;
                break;
            }
            const auto next = succ.find(v);
            if (next == succ.end()) break;
            const size_t a = static_cast<size_t>(v) * 3;
            const size_t b = static_cast<size_t>(next->second) * 3;
            const float dx = mesh.pos[a] - mesh.pos[b];
            const float dy = mesh.pos[a + 1] - mesh.pos[b + 1];
            const float dz = mesh.pos[a + 2] - mesh.pos[b + 2];
            perimeter += std::sqrt(dx * dx + dy * dy + dz * dz);
            v = next->second;
            if (v == start.first) break;
        }
        if (!ok || v != start.first || loop.size() < 3 || loop.size() > kMaxLoopVertices) continue;
        if (perimeter > kMaxPerimeter) continue;

        float cx = 0.f, cy = 0.f, cz = 0.f;
        float nx = 0.f, ny = 0.f, nz = 0.f;
        float cr = 0.f, cg = 0.f, cb = 0.f;
        for (const uint32_t index : loop) {
            const size_t p = static_cast<size_t>(index) * 3;
            cx += mesh.pos[p];
            cy += mesh.pos[p + 1];
            cz += mesh.pos[p + 2];
            nx += mesh.nrm[p];
            ny += mesh.nrm[p + 1];
            nz += mesh.nrm[p + 2];
            cr += static_cast<float>(mesh.col[p]);
            cg += static_cast<float>(mesh.col[p + 1]);
            cb += static_cast<float>(mesh.col[p + 2]);
        }
        const float inv = 1.f / static_cast<float>(loop.size());
        const float nl = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nl < 1e-6f) continue;
        const uint32_t centre = base + static_cast<uint32_t>(fillPos.size() / 3);
        fillPos.push_back(cx * inv);
        fillPos.push_back(cy * inv);
        fillPos.push_back(cz * inv);
        fillNrm.push_back(nx / nl);
        fillNrm.push_back(ny / nl);
        fillNrm.push_back(nz / nl);
        fillCol.push_back(static_cast<uint8_t>(cr * inv));
        fillCol.push_back(static_cast<uint8_t>(cg * inv));
        fillCol.push_back(static_cast<uint8_t>(cb * inv));

        // The rim was closed along succ, i.e. the hole is wound opposite to the
        // surface, so the fan uses the reversed pair.
        for (size_t i = 0; i < loop.size(); ++i) {
            const uint32_t a = loop[i];
            const uint32_t b = loop[(i + 1) % loop.size()];
            fillIndex.push_back(centre);
            fillIndex.push_back(b);
            fillIndex.push_back(a);
        }
    }

    if (fillIndex.empty()) return;

    mesh.index.insert(mesh.index.end(), fillIndex.begin(), fillIndex.end());
    mesh.pos.insert(mesh.pos.end(), fillPos.begin(), fillPos.end());
    mesh.nrm.insert(mesh.nrm.end(), fillNrm.begin(), fillNrm.end());
    mesh.col.insert(mesh.col.end(), fillCol.begin(), fillCol.end());
}

// Debris removal for the exported file. Earlier thresholds (1000 faces / 30 cm)
// deleted entire chairs, plants and wall pieces that were not yet bridged to the
// main body, so the rule is now conservative: only a speck that is both tiny in
// triangle count and a few centimetres across is dropped.
void dropSmallBlobs(WeldedMesh& mesh) {
    constexpr uint32_t kMinBlobFaces = 12u;
    constexpr float kMinBlobExtent = 0.04f;

    const size_t verts = mesh.vertexCount();
    const size_t faces = mesh.faceCount();
    if (verts == 0 || faces == 0) return;

    std::vector<uint32_t> parent(verts);
    for (uint32_t i = 0; i < verts; ++i) parent[i] = i;
    for (size_t i = 0; i < faces; ++i) {
        const uint32_t a = findRoot(parent, mesh.index[i * 3]);
        for (int k = 1; k < 3; ++k) {
            const uint32_t b = findRoot(parent, mesh.index[i * 3 + k]);
            if (b != a) parent[b] = a;
        }
    }
    for (uint32_t i = 0; i < verts; ++i) parent[i] = findRoot(parent, i);

    std::vector<uint32_t> facesPerRoot(verts, 0);
    std::vector<float> lo(static_cast<size_t>(verts) * 3, 0.f);
    std::vector<float> hi(static_cast<size_t>(verts) * 3, 0.f);
    std::vector<uint8_t> seen(verts, 0);
    for (size_t i = 0; i < faces; ++i) ++facesPerRoot[parent[mesh.index[i * 3]]];
    for (uint32_t v = 0; v < verts; ++v) {
        const uint32_t r = parent[v];
        for (int k = 0; k < 3; ++k) {
            const float c = mesh.pos[static_cast<size_t>(v) * 3 + static_cast<size_t>(k)];
            if (!seen[r]) {
                lo[static_cast<size_t>(r) * 3 + static_cast<size_t>(k)] = c;
                hi[static_cast<size_t>(r) * 3 + static_cast<size_t>(k)] = c;
            } else {
                lo[static_cast<size_t>(r) * 3 + static_cast<size_t>(k)] =
                    std::min(lo[static_cast<size_t>(r) * 3 + static_cast<size_t>(k)], c);
                hi[static_cast<size_t>(r) * 3 + static_cast<size_t>(k)] =
                    std::max(hi[static_cast<size_t>(r) * 3 + static_cast<size_t>(k)], c);
            }
        }
        seen[r] = 1;
    }

    uint32_t mainRoot = 0;
    for (uint32_t v = 0; v < verts; ++v) {
        if (facesPerRoot[v] > facesPerRoot[mainRoot]) mainRoot = v;
    }

    std::vector<uint32_t> kept;
    kept.reserve(mesh.index.size());
    size_t droppedFaces = 0;
    for (size_t i = 0; i < faces; ++i) {
        const uint32_t r = parent[mesh.index[i * 3]];
        bool keep = r == mainRoot || facesPerRoot[r] >= kMinBlobFaces;
        if (!keep) {
            const float dx = hi[static_cast<size_t>(r) * 3] - lo[static_cast<size_t>(r) * 3];
            const float dy = hi[static_cast<size_t>(r) * 3 + 1] - lo[static_cast<size_t>(r) * 3 + 1];
            const float dz = hi[static_cast<size_t>(r) * 3 + 2] - lo[static_cast<size_t>(r) * 3 + 2];
            keep = std::sqrt(dx * dx + dy * dy + dz * dz) >= kMinBlobExtent;
        }
        if (keep) {
            kept.push_back(mesh.index[i * 3]);
            kept.push_back(mesh.index[i * 3 + 1]);
            kept.push_back(mesh.index[i * 3 + 2]);
        } else {
            ++droppedFaces;
        }
    }
    if (droppedFaces > 0) mesh.index.swap(kept);
}

WeldedMesh weldMesh(const MeshData& mesh) {
    WeldedMesh out;
    out.pos.reserve(mesh.vertices.size());
    out.nrm.reserve(mesh.vertices.size());
    out.col.reserve(mesh.vertices.size());

    std::unordered_map<uint64_t, uint32_t> lookup;
    lookup.reserve(mesh.vertices.size() / 4 + 1);

    out.index.reserve(mesh.indices.size());
    for (uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) continue;
        out.index.push_back(weldVertex(out, lookup, mesh.vertices[index]));
    }

    const size_t verts = out.vertexCount();
    for (size_t i = 0; i < verts; ++i) {
        float& nx = out.nrm[i * 3 + 0];
        float& ny = out.nrm[i * 3 + 1];
        float& nz = out.nrm[i * 3 + 2];
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-6f) {
            nx /= len;
            ny /= len;
            nz /= len;
        } else {
            nx = 0.f;
            ny = 1.f;
            nz = 0.f;
        }
    }

    filterMesh(out);
    dropSmallBlobs(out);
    decimateFlatRegions(out);
    fillSmallHoles(out);
    return out;
}

bool exportObj(const WeldedMesh& mesh, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;

    std::fprintf(f, "# Mobile3DRecon fused TSDF surface\n");
    std::fprintf(f, "# %zu vertices, %zu triangles (welded)\n", mesh.vertexCount(),
                 mesh.faceCount());

    for (size_t i = 0; i < mesh.vertexCount(); ++i) {
        std::fprintf(f, "v %.5f %.5f %.5f %u %u %u\n", mesh.pos[i * 3],
                     mesh.pos[i * 3 + 1], mesh.pos[i * 3 + 2],
                     static_cast<unsigned>(mesh.col[i * 3]),
                     static_cast<unsigned>(mesh.col[i * 3 + 1]),
                     static_cast<unsigned>(mesh.col[i * 3 + 2]));
    }
    for (size_t i = 0; i < mesh.vertexCount(); ++i) {
        std::fprintf(f, "vn %.4f %.4f %.4f\n", mesh.nrm[i * 3], mesh.nrm[i * 3 + 1],
                     mesh.nrm[i * 3 + 2]);
    }
    for (size_t i = 0; i + 2 < mesh.index.size(); i += 3) {
        const uint32_t a = mesh.index[i] + 1;
        const uint32_t b = mesh.index[i + 1] + 1;
        const uint32_t c = mesh.index[i + 2] + 1;
        std::fprintf(f, "f %u//%u %u//%u %u//%u\n", a, a, b, b, c, c);
    }

    const bool ok = std::ferror(f) == 0;
    std::fclose(f);
    return ok;
}

bool exportPly(const WeldedMesh& mesh, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;

    std::fprintf(f, "ply\n");
    std::fprintf(f, "format ascii 1.0\n");
    std::fprintf(f, "comment Mobile3DRecon fused TSDF surface\n");
    std::fprintf(f, "element vertex %zu\n", mesh.vertexCount());
    std::fprintf(f, "property float x\n");
    std::fprintf(f, "property float y\n");
    std::fprintf(f, "property float z\n");
    std::fprintf(f, "property float nx\n");
    std::fprintf(f, "property float ny\n");
    std::fprintf(f, "property float nz\n");
    std::fprintf(f, "property uchar red\n");
    std::fprintf(f, "property uchar green\n");
    std::fprintf(f, "property uchar blue\n");
    std::fprintf(f, "element face %zu\n", mesh.faceCount());
    std::fprintf(f, "property list uchar int vertex_indices\n");
    std::fprintf(f, "end_header\n");

    for (size_t i = 0; i < mesh.vertexCount(); ++i) {
        std::fprintf(f, "%.5f %.5f %.5f %.4f %.4f %.4f %u %u %u\n", mesh.pos[i * 3],
                     mesh.pos[i * 3 + 1], mesh.pos[i * 3 + 2], mesh.nrm[i * 3],
                     mesh.nrm[i * 3 + 1], mesh.nrm[i * 3 + 2],
                     static_cast<unsigned>(mesh.col[i * 3]),
                     static_cast<unsigned>(mesh.col[i * 3 + 1]),
                     static_cast<unsigned>(mesh.col[i * 3 + 2]));
    }
    for (size_t i = 0; i + 2 < mesh.index.size(); i += 3) {
        std::fprintf(f, "3 %u %u %u\n", mesh.index[i], mesh.index[i + 1],
                     mesh.index[i + 2]);
    }

    const bool ok = std::ferror(f) == 0;
    std::fclose(f);
    return ok;
}

}  // namespace

bool exportMesh(const MeshData& mesh, const std::string& path,
                const std::string& format) {
    if (mesh.indices.empty()) return false;
    WeldedMesh welded = weldMesh(mesh);
    if (welded.index.empty()) return false;
    if (format == "PLY" || format == "ply") return exportPly(welded, path);
    return exportObj(welded, path);
}

}  // namespace recon
