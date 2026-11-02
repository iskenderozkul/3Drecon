#include "core/engine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_set>

#include "io/exporter.h"
#include "mvs/census.h"

namespace recon {
namespace {

// A cube on a block face samples voxels from the neighbouring block, so a
// changed block also invalidates the cached geometry of its six face
// neighbours (otherwise block borders keep one-voxel cracks).
void expandDirtyNeighbours(VoxelHash& hash, std::vector<VoxelBlock*>& dirty) {
    if (dirty.empty()) return;
    // The mesher reads raw samples two voxels into the neighbouring block (the
    // smoothed field is 11^3 with an offset of one), so a change on a block's
    // boundary alters the surface in every block that shares a face, edge or
    // corner with it. Expanding only along the six faces left the diagonal
    // neighbours holding their old triangles, which showed up as a step at
    // block corners every time the surface moved under carving.
    static const int kOffsets[26][3] = {
        {1, 0, 0},  {-1, 0, 0}, {0, 1, 0},  {0, -1, 0}, {0, 0, 1},  {0, 0, -1},
        {1, 1, 0},  {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
        {1, 0, 1},  {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
        {0, 1, 1},  {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
        {1, 1, 1},  {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
        {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};
    std::unordered_set<VoxelBlock*> seen(dirty.begin(), dirty.end());
    const size_t base = dirty.size();
    for (size_t i = 0; i < base; ++i) {
        const VoxelBlock* block = dirty[i];
        for (const auto& offset : kOffsets) {
            VoxelBlock* neighbour = hash.get(block->bx + offset[0],
                                             block->by + offset[1],
                                             block->bz + offset[2]);
            if (neighbour && seen.insert(neighbour).second) {
                neighbour->state = VoxelState::UPDATE;
                dirty.push_back(neighbour);
            }
        }
    }
}

inline float nowSeconds() {
    using namespace std::chrono;
    return duration<float>(steady_clock::now().time_since_epoch()).count();
}

// Float seconds lose millisecond resolution once the monotonic clock passes a
// few hundred thousand seconds, so short intervals (fusion cost) are measured
// in double.
inline double nowSecondsPrecise() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

inline void yuvToRgb(int y, int u, int v, uint8_t& r, uint8_t& g, uint8_t& b) {
    const float yf = 1.164f * (static_cast<float>(y) - 16.f);
    const float uf = static_cast<float>(u) - 128.f;
    const float vf = static_cast<float>(v) - 128.f;
    r = static_cast<uint8_t>(clampf(yf + 1.596f * vf, 0.f, 255.f));
    g = static_cast<uint8_t>(clampf(yf - 0.813f * vf - 0.391f * uf, 0.f, 255.f));
    b = static_cast<uint8_t>(clampf(yf + 2.018f * uf, 0.f, 255.f));
}

inline uint32_t turboColor(float t) {
    static const float kStops[5][3] = {{0.08f, 0.06f, 0.35f},
                                       {0.10f, 0.55f, 0.72f},
                                       {0.20f, 0.85f, 0.45f},
                                       {0.95f, 0.83f, 0.22f},
                                       {0.90f, 0.18f, 0.15f}};
    t = clampf(t, 0.f, 0.999f) * 4.f;
    const int i = static_cast<int>(t);
    const float f = t - static_cast<float>(i);
    const float* a = kStops[i];
    const float* b = kStops[i + 1];
    const uint32_t r = static_cast<uint32_t>(clampf(a[0] + (b[0] - a[0]) * f, 0.f, 1.f) * 255.f);
    const uint32_t g = static_cast<uint32_t>(clampf(a[1] + (b[1] - a[1]) * f, 0.f, 1.f) * 255.f);
    const uint32_t bl = static_cast<uint32_t>(clampf(a[2] + (b[2] - a[2]) * f, 0.f, 1.f) * 255.f);
    return 0xFF000000u | (r << 16) | (g << 8) | bl;
}

}  // namespace

Engine::Engine() {
    worker_ = std::thread([this] { workerLoop(); });
}

Engine::~Engine() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void Engine::submitFrame(FrameInput&& frame) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = std::move(frame);
        hasPending_ = true;
    }
    cv_.notify_one();
}

void Engine::setScanning(bool scanning) { scanning_.store(scanning); }

void Engine::reset() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        resetRequested_ = true;
        hasPending_ = false;
    }
    cv_.notify_one();
}

bool Engine::exportMesh(const std::string& path, const std::string& format) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        remeshRequested_ = true;
        remeshDone_ = false;
        cv_.notify_one();
        // The worker owns the reconstruction state, so the full re-mesh has to
        // happen there. If it cannot finish in time the live snapshot is still
        // a valid (older) mesh, which beats handing the user no file at all.
        remeshCv_.wait_for(lock, std::chrono::seconds(20), [this] { return remeshDone_; });
    }
    std::shared_ptr<const MeshData> snapshot;
    {
        std::lock_guard<std::mutex> lock(exportMutex_);
        snapshot = exportSnapshot_;
        exportSnapshot_.reset();
    }
    if (!snapshot || snapshot->indices.empty()) snapshot = mesh();
    if (!snapshot || snapshot->indices.empty()) return false;
    return recon::exportMesh(*snapshot, path, format);
}

std::shared_ptr<const MeshData> Engine::mesh() const {
    if (tangoActive_.load()) return tango_.mesh();
    return mesher_.snapshot();
}

int Engine::triangleCount() const {
    return tangoActive_.load() ? tango_.triangleCount() : mesher_.triangleCount();
}

int Engine::vertexCount() const {
    return tangoActive_.load() ? tango_.vertexCount() : mesher_.vertexCount();
}

void Engine::preview(uint32_t* depthOut, uint32_t* confidenceOut) const {
    std::lock_guard<std::mutex> lock(previewMutex_);
    if (depthOut) {
        std::copy(previewDepth_.begin(), previewDepth_.end(), depthOut);
    }
    if (confidenceOut) {
        std::copy(previewConfidence_.begin(), previewConfidence_.end(), confidenceOut);
    }
}

void Engine::workerLoop() {
    float fpsWindowStart = nowSeconds();
    int fpsWindowFrames = 0;

    while (true) {
        FrameInput frame;
        bool doReset = false;
        bool doRemesh = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] {
                return !running_ || hasPending_ || resetRequested_ || remeshRequested_;
            });
            if (!running_) break;
            if (resetRequested_) {
                resetRequested_ = false;
                doReset = true;
            }
            if (remeshRequested_) {
                remeshRequested_ = false;
                doRemesh = true;
            }
            if (!doReset && !doRemesh) {
                frame = std::move(pending_);
                hasPending_ = false;
            }
        }

        if (doReset) {
            performReset();
            fpsWindowStart = nowSeconds();
            fpsWindowFrames = 0;
        }
        if (doRemesh) {
            const float start = nowSeconds();
            if (tangoActive_.load()) {
                tango_.publish(true);
                auto full = std::make_shared<MeshData>();
                const bool ok = tango_.buildFullMesh(*full);
                if (!ok || full->indices.empty()) {
                    const auto live = tango_.mesh();
                    if (live) *full = *live;
                }
                {
                    std::lock_guard<std::mutex> lock(exportMutex_);
                    exportSnapshot_ = full;
                }
                blockCount_.store(tango_.segmentCount());
                float bx0 = 0.f, by0 = 0.f, bz0 = 0.f;
                float bx1 = 0.f, by1 = 0.f, bz1 = 0.f;
                int bv = 0;
                for (const MeshVertex& v : full->vertices) {
                    if (bv++ == 0) {
                        bx0 = bx1 = v.px;
                        by0 = by1 = v.py;
                        bz0 = bz1 = v.pz;
                    } else {
                        bx0 = std::min(bx0, v.px); bx1 = std::max(bx1, v.px);
                        by0 = std::min(by0, v.py); by1 = std::max(by1, v.py);
                        bz0 = std::min(bz0, v.pz); bz1 = std::max(bz1, v.pz);
                    }
                }
                char line[224];
                std::snprintf(line, sizeof(line),
                              "remesh source=t3dr seg=%d tris=%d ms=%.0f "
                              "bbox=%.1f,%.1f,%.1f..%.1f,%.1f,%.1f",
                              tango_.segmentCount(), tango_.triangleCount(),
                              static_cast<double>((nowSeconds() - start) * 1000.f),
                              static_cast<double>(bx0), static_cast<double>(by0),
                              static_cast<double>(bz0), static_cast<double>(bx1),
                              static_cast<double>(by1), static_cast<double>(bz1));
                pushDiag(line);
            } else {
                mesher_.rebuildAll(hash_);
                // The full pass just consumed every change in the hash, so the
                // incremental queue has nothing left to do.
                pendingDirty_.clear();
                pendingSet_.clear();
                blockCount_.store(static_cast<int>(hash_.size()));
                {
                    std::lock_guard<std::mutex> lock(exportMutex_);
                    exportSnapshot_ = mesher_.snapshot();
                }
                char line[128];
                std::snprintf(line, sizeof(line), "remesh blocks=%d tris=%d ms=%.0f",
                              static_cast<int>(hash_.size()), mesher_.triangleCount(),
                              static_cast<double>((nowSeconds() - start) * 1000.f));
                pushDiag(line);
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                remeshDone_ = true;
            }
            remeshCv_.notify_all();
        }
        if (doReset || doRemesh) continue;

        const float start = nowSeconds();
        const bool processed = processFrame(frame);
        if (processed) {
            const float elapsedMs = (nowSeconds() - start) * 1000.f;
            const float prev = processMs_.load();
            processMs_.store(prev <= 0.f ? elapsedMs : prev * 0.85f + elapsedMs * 0.15f);
        }

        ++fpsWindowFrames;
        const float span = nowSeconds() - fpsWindowStart;
        if (span >= 1.f) {
            fps_.store(static_cast<float>(fpsWindowFrames) / span);
            fpsWindowFrames = 0;
            fpsWindowStart = nowSeconds();
        }
        framesProcessed_.fetch_add(1);
    }
}

void Engine::performReset() {
    keyframes_.clear();
    hash_.clear();
    mesher_.reset();
    // A reset is also how the user moves between a room and the street; the
    // volumetric profile is fixed (reset() re-initialises it) and the scene
    // statistics only choose how sparse samples are cleaned.
    tango_.reset();
    tangoReady_ = tango_.ready();
    tangoActive_.store(false);
    tangoFailStreak_ = 0;
    outdoor_ = false;
    sceneDecided_ = false;
    sceneSamples_ = 0;
    sceneSkySum_ = 0.f;
    sceneFarSum_ = 0.f;
    sceneMedianSum_ = 0.f;
    sceneLumaSum_ = 0.f;
    lastSpeed_ = 0.f;
    feedScale_ = 1;
    updateMsEma_ = 0.f;
    feedQuietFrames_ = 0;
    fusedDepth_.clear();
    fusedVar_.clear();
    fusedAge_.clear();
    lumaGrid_.clear();
    predDepth_.clear();
    predVar_.clear();
    predValid_.clear();
    feedDepthMm_.clear();
    feedW_ = 0;
    feedH_ = 0;
    {
        std::lock_guard<std::mutex> lock(exportMutex_);
        exportSnapshot_.reset();
    }
    pendingDirty_.clear();
    pendingSet_.clear();
    nextKeyframeId_ = 0;
    lastCamera_ = Vec3{0.f, 0.f, 0.f};
    keyframeCount_.store(0);
    blockCount_.store(0);
    depthSource_.store(0);
    hrDepthValid_.store(0.f);
    hasPrevFrame_ = false;
    stableFrames_ = 3;
    hasAccepted_ = false;
    lastCarveSeconds_ = 0.f;
    lastExtractSeconds_ = 0.f;
    lastDiagSeconds_ = 0.f;
    hasCarvePosition_ = false;
    prevDepthW_ = 0;
    prevDepthH_ = 0;
    fuseSeeds_ = 0;
    fuseRuns_ = 0;
    fuseWarpSkips_ = 0;
    lastRelLength_ = 0.f;
    lastRelAngle_ = 0.f;
    motionScore_.clear();
    dynamicPixels_.store(0);
    {
        std::lock_guard<std::mutex> lock(dynamicMutex_);
        dynamicSnapshot_ = std::make_shared<MeshData>();
    }
    {
        std::lock_guard<std::mutex> lock(diagMutex_);
        diagLines_.clear();
    }
    processMs_.store(0.f);
    fps_.store(0.f);
    {
        std::lock_guard<std::mutex> lock(previewMutex_);
        std::fill(previewDepth_.begin(), previewDepth_.end(), 0xFF14181Cu);
        std::fill(previewConfidence_.begin(), previewConfidence_.end(), 0xFF14181Cu);
    }
}

void Engine::buildKeyframe(const FrameInput& frame, Keyframe& kf) const {
    const int hw = frame.width / 2;
    const int hh = frame.height / 2;
    const int cw = frame.width / 2;
    const int ch = frame.height / 2;

    kf.timestampNs = frame.timestampNs;
    kf.w = hw;
    kf.h = hh;
    kf.R_gl = matFromQuat(frame.qx, frame.qy, frame.qz, frame.qw);
    kf.t = Vec3{frame.tx, frame.ty, frame.tz};
    kf.K.fx = frame.K.fx * 0.5f;
    kf.K.fy = frame.K.fy * 0.5f;
    kf.K.cx = frame.K.cx * 0.5f;
    kf.K.cy = frame.K.cy * 0.5f;
    kf.K.width = hw;
    kf.K.height = hh;

    kf.luma.resize(static_cast<size_t>(hw) * hh);
    kf.rgb.resize(static_cast<size_t>(hw) * hh * 3);

    const uint8_t* ySrc = frame.y.data();
    const uint8_t* uSrc = frame.u.data();
    const uint8_t* vSrc = frame.v.data();
    const int yStride = frame.width;

    for (int y = 0; y < hh; ++y) {
        const int sy = y * 2;
        const int row0 = sy * yStride;
        const int row1 = (sy + 1) * yStride;
        const int uvRow = std::min(y, ch - 1) * cw;
        for (int x = 0; x < hw; ++x) {
            const int sx = x * 2;
            const int sum = ySrc[row0 + sx] + ySrc[row0 + sx + 1] +
                            ySrc[row1 + sx] + ySrc[row1 + sx + 1];
            const int luma = sum / 4;
            const size_t idx = static_cast<size_t>(y) * hw + x;
            kf.luma[idx] = static_cast<uint8_t>(luma);

            const int cx = std::min(x, cw - 1);
            uint8_t r, g, b;
            yuvToRgb(ySrc[row0 + sx], uSrc[uvRow + cx], vSrc[uvRow + cx], r, g, b);
            kf.rgb[idx * 3 + 0] = r;
            kf.rgb[idx * 3 + 1] = g;
            kf.rgb[idx * 3 + 2] = b;
        }
    }

    computeCensus(kf.luma.data(), hw, hh, kf.census);
}

bool Engine::processFrame(const FrameInput& frame) {
    if (!frame.valid()) return false;
    if (!frame.tracking) return false;

    const Vec3 position{frame.tx, frame.ty, frame.tz};
    lastCamera_ = position;
    if (!scanning_.load()) return false;

    const double ts = static_cast<double>(frame.timestampNs) * 1e-9;
    double frameDt = 0.0;
    float frameJump = 0.f;
    if (hasPrevFrame_) {
        frameDt = static_cast<double>(frame.timestampNs - prevFrameTs_) * 1e-9;
        frameJump = (position - prevFrameT_).length();
    }
    prevFrameT_ = position;
    prevFrameTs_ = frame.timestampNs;
    hasPrevFrame_ = true;
    lastSpeed_ = (frameDt > 1e-4 && frameDt < 0.5)
                     ? static_cast<float>(frameJump / static_cast<float>(frameDt))
                     : 0.f;

    const int kfId = nextKeyframeId_ + 1;
    // ARCore relocalising shows up as an impossible apparent speed between two
    // consecutive camera frames. Integrating that frame (and the one right
    // after it) pins a whole depth map at the wrong pose, so wait a few frames
    // for the pose to settle before fusing again.
    if (frameDt > 0.0 && frameDt < 0.5 && frameJump / static_cast<float>(frameDt) > 2.5f) {
        stableFrames_ = 0;
        char line[192];
        std::snprintf(line, sizeof(line),
                      "kf=%d ts=%.3f skip=jump jump=%.2f dt=%.2f pos=%.2f,%.2f,%.2f", kfId, ts,
                      static_cast<double>(frameJump), frameDt, static_cast<double>(position.x),
                      static_cast<double>(position.y), static_cast<double>(position.z));
        pushDiag(line);
        return false;
    }
    if (stableFrames_ < 3) {
        ++stableFrames_;
        return false;
    }

    if (frame.hasDepth()) return processDepthFrame(frame);

    if (frame.depthApiActive) {
        // Depth is expected on this session, so a missing image is a transient
        // acquisition gap. MVS on these devices produced near-empty maps that
        // were fused as if they were measurements.
        const float now = nowSeconds();
        if (now - lastDiagSeconds_ >= 1.f) {
            lastDiagSeconds_ = now;
            char line[128];
            std::snprintf(line, sizeof(line), "ts=%.3f skip=nodepth", ts);
            pushDiag(line);
        }
        return false;
    }
    return processMvsFrame(frame);
}

bool Engine::processMvsFrame(const FrameInput& frame) {
    // Once the Tango core owns the volume, stereo keyframes would feed a second
    // mesh that mesh() never exposes; skip them instead of blurring two worlds.
    if (tangoActive_.load()) return false;

    const Vec3 position{frame.tx, frame.ty, frame.tz};
    const double ts = static_cast<double>(frame.timestampNs) * 1e-9;
    const int kfId = nextKeyframeId_ + 1;

    Keyframe candidate;
    candidate.timestampNs = frame.timestampNs;
    candidate.w = frame.width / 2;
    candidate.h = frame.height / 2;
    candidate.R_gl = matFromQuat(frame.qx, frame.qy, frame.qz, frame.qw);
    candidate.t = position;

    const double sinceLastKeyframe =
        hasAccepted_ ? static_cast<double>(frame.timestampNs - lastAcceptedTs_) * 1e-9 : 1.0;
    if (!keyframes_.empty() && !keyframes_.shouldAdd(candidate, sinceLastKeyframe)) return false;

    candidate.id = kfId;
    buildKeyframe(frame, candidate);

    bool haveDepth = false;
    {
        std::vector<std::pair<const Keyframe*, float>> refs;
        // The paper uses the top two reference keyframes as a trade-off between
        // accuracy and the cost of stereo matching on a phone.
        keyframes_.selectReferences(candidate, 2, refs);
        if (!refs.empty() && stereo_.compute(candidate, refs, depth_)) {
            refiner_.refine(candidate.luma.data(), candidate.w, candidate.h, depth_);
            int valid = 0;
            for (const float d : depth_.depth) {
                if (d > 0.f) ++valid;
            }
            if (valid < (depth_.w * depth_.h) / 20) {
                char line[160];
                std::snprintf(line, sizeof(line), "kf=%d ts=%.3f skip=sparse valid=%d/%d", kfId, ts,
                              valid, depth_.w * depth_.h);
                pushDiag(line);
                return false;
            }
            depthSource_.store(2);
            haveDepth = true;
        }
    }

    if (haveDepth) {
        dirty_.clear();
        integrator_.integrate(candidate, depth_, hash_, dirty_);
        queueDirty(dirty_);
        dirty_.clear();
        integrator_.visibilityCarve(candidate, depth_, hash_, dirty_);
        queueDirty(dirty_);
        flushDirty();
        evictIfNeeded(candidate.t);
        updatePreview(depth_);
        candidate.depthMap = depth_.depth;
        blockCount_.store(static_cast<int>(hash_.size()));
    }

    {
        // Depth statistics sampled every 16th pixel; cheap and enough to see
        // whether a frame contributed real metric geometry.
        float dmin = 0.f, dmax = 0.f, dmed = 0.f;
        int valid = 0, total = 0;
        if (haveDepth) {
            std::vector<float> samples;
            samples.reserve(4096);
            const int stride = 16;
            for (size_t i = 0; i < depth_.depth.size(); i += stride) {
                const float d = depth_.depth[i];
                ++total;
                if (d <= 0.f) continue;
                ++valid;
                if (dmin == 0.f || d < dmin) dmin = d;
                if (d > dmax) dmax = d;
                if (samples.size() < 4096) samples.push_back(d);
            }
            if (!samples.empty()) {
                const size_t mid = samples.size() / 2;
                std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
                dmed = samples[mid];
            }
        }
        const Vec3 fwd = candidate.R_cv() * Vec3{0.f, 0.f, -1.f};
        const double yaw = std::atan2(static_cast<double>(fwd.x), static_cast<double>(-fwd.z)) *
                           180.0 / 3.14159265358979323846;
        char line[448];
        std::snprintf(line, sizeof(line),
                      "mvs kf=%d ts=%.3f dep=%dx%d valid=%.1f%% dmin=%.2f dmed=%.2f "
                      "dmax=%.2f tris=%d blocks=%d ms=%.0f pos=%.2f,%.2f,%.2f yaw=%.1f",
                      kfId, ts, depth_.w, depth_.h,
                      total > 0 ? 100.0 * valid / total : 0.0,
                      static_cast<double>(dmin), static_cast<double>(dmed),
                      static_cast<double>(dmax), mesher_.triangleCount(),
                      static_cast<int>(hash_.size()), static_cast<double>(processMs_.load()),
                      static_cast<double>(candidate.t.x), static_cast<double>(candidate.t.y),
                      static_cast<double>(candidate.t.z), yaw);
        pushDiag(line);
    }
    nextKeyframeId_ = kfId;
    lastAcceptedTs_ = frame.timestampNs;
    hasAccepted_ = true;
    keyframes_.add(std::move(candidate));
    keyframeCount_.store(nextKeyframeId_);
    return true;
}

void Engine::pushDiag(std::string line) {
    std::lock_guard<std::mutex> lock(diagMutex_);
    if (diagLines_.size() >= 600) diagLines_.pop_front();
    diagLines_.push_back(std::move(line));
}

std::string Engine::drainDiagnostics() {
    std::lock_guard<std::mutex> lock(diagMutex_);
    std::string out;
    for (const std::string& line : diagLines_) {
        out += line;
        out += '\n';
    }
    diagLines_.clear();
    return out;
}

bool Engine::processDepthFrame(const FrameInput& frame) {
    const float start = nowSeconds();
    const double ts = static_cast<double>(frame.timestampNs) * 1e-9;

    if (!tangoInitTried_) {
        tangoInitTried_ = true;
        tangoReady_ = tango_.init();
        if (!tangoReady_) pushDiag("t3dr=unavailable fallback=legacy");
    }
    if (!tangoReady_) return processLegacyDepthFrame(frame);

    // A depth map that trails the camera by most of a second would be fused at
    // the wrong pose. Walking outdoors the lag matters much sooner: even a few
    // centimetres of displacement between the depth capture and the pose turns
    // one map into a ghost sheet, so the threshold is tight and also scales
    // with how fast the phone is moving.
    double depthLagMs = std::numeric_limits<double>::quiet_NaN();
    if (frame.depthTimestampNs > 0) {
        depthLagMs = static_cast<double>(frame.timestampNs - frame.depthTimestampNs) * 1e-6;
        const double displacement = std::max(0.0, depthLagMs) * 0.001 * lastSpeed_;
        if (depthLagMs > 400.0 || displacement > 0.20) {
            if (start - lastDiagSeconds_ >= 1.f) {
                lastDiagSeconds_ = start;
                char line[192];
                std::snprintf(line, sizeof(line),
                              "ts=%.3f skip=staledepth lag=%.0fms pos=%.2f,%.2f,%.2f", ts,
                              depthLagMs, static_cast<double>(frame.tx),
                              static_cast<double>(frame.ty), static_cast<double>(frame.tz));
                pushDiag(line);
            }
            // The temporal filter keeps its history: the pose check inside
            // fuseDepth drops the warp if the camera really moved, and a
            // stationary gap does not make the previous surface wrong.
            return false;
        }
    }

    // The cleaned grid is what gets fused: median-rejected flyers would
    // otherwise form sheets between foreground and background, and moving
    // pixels are zeroed so a walking person does not smear the room.
    // Outdoors the depth map is sparse, so the dense path only needs 4 % valid
    // pixels to keep integrating; the old 12.5 % gate made meshing start only
    // after the user walked back inside.
    const bool depthUsable = prepareDepthGrid(frame, depth_, 0.04f);
    // Classify even from frames that are too sparse to fuse: a sunlit street
    // often sits right at the validity gate, and the scene statistics still
    // carry enough signal to choose the right cleaning rules. Only vote when
    // the cleaned grid actually belongs to this frame.
    if (depth_.w == frame.depthWidth && depth_.h == frame.depthHeight) {
        sampleScene(depth_, meanLuma(frame));
    }
    if (!depthUsable) {
        if (start - lastDiagSeconds_ >= 1.f) {
            lastDiagSeconds_ = start;
            char line[160];
            std::snprintf(line, sizeof(line), "ts=%.3f skip=thinDepth valid=%.1f%%", ts,
                          100.0 * static_cast<double>(hrDepthValid_.load()));
            pushDiag(line);
        }
        return false;
    }

    // One warp serves both jobs: the Kalman blend uses the predicted surface
    // as its prior, and the same residual identifies surfaces that moved. The
    // fused grid is what the reconstruction sees, so isolated depth noise is
    // averaged out instead of triangulated into spikes.
    const double fuseStart = nowSecondsPrecise();
    const int movingPixels = fuseDepth(frame, depth_, motionMask_);
    const float fuseMs =
        static_cast<float>((nowSecondsPrecise() - fuseStart) * 1000.0);
    buildFusionFeed(frame);
    buildDynamicMesh(frame, depth_, motionMask_);

    TangoRecon::DepthFeed feed;
    feed.mm = &feedDepthMm_;
    feed.width = feedW_;
    feed.height = feedH_;
    feed.K = feedK_;
    if (!tango_.integrate(frame, &feed)) {
        ++tangoFailStreak_;
        if (tangoFailStreak_ >= 30) {
            // A library that keeps rejecting every frame would otherwise leave
            // the user with a live camera and no geometry at all.
            tangoReady_ = false;
            tangoActive_.store(false);
            pushDiag("t3dr=disabled after 30 failures fallback=legacy");
        }
        if (start - lastDiagSeconds_ >= 1.f) {
            lastDiagSeconds_ = start;
            char line[192];
            std::snprintf(line, sizeof(line),
                          "ts=%.3f skip=t3dr dep=%dx%d valid=%.1f%% pos=%.2f,%.2f,%.2f", ts,
                          frame.depthWidth, frame.depthHeight,
                          100.0 * static_cast<double>(hrDepthValid_.load()),
                          static_cast<double>(frame.tx), static_cast<double>(frame.ty),
                          static_cast<double>(frame.tz));
            pushDiag(line);
        }
        return false;
    }
    tangoFailStreak_ = 0;
    if (tango_.triangleCount() == 0 && tango_.integratedFrames() >= 120) {
        // Successful updates but not one triangle after four seconds of metric
        // depth means the core is not geometrically usable on this device.
        tangoReady_ = false;
        tangoActive_.store(false);
        pushDiag("t3dr=no_geometry fallback=legacy");
        return false;
    }

    // The reconstruction cost scales with the number of depth rays and the
    // cell count it can touch. When it dominates the frame, halve the grid
    // handed to it: the fused surface is already smoothed and hole-filled, so
    // the half-resolution copy carries nearly the same geometry for a quarter
    // of the work. Hysteresis keeps it from flapping between the two.
    const float upd = static_cast<float>(tango_.lastUpdateMs());
    updateMsEma_ = updateMsEma_ > 0.f ? updateMsEma_ * 0.8f + upd * 0.2f : upd;
    if (feedScale_ == 1 && updateMsEma_ > 70.f) {
        feedScale_ = 2;
        feedQuietFrames_ = 0;
        pushDiag("feed=half_depth");
    } else if (feedScale_ == 2) {
        if (updateMsEma_ < 20.f) {
            if (++feedQuietFrames_ >= 150) {
                feedScale_ = 1;
                feedQuietFrames_ = 0;
                pushDiag("feed=full_depth");
            }
        } else {
            feedQuietFrames_ = 0;
        }
    }

    tangoActive_.store(true);
    depthSource_.store(1);
    ++nextKeyframeId_;
    keyframeCount_.store(nextKeyframeId_);
    blockCount_.store(tango_.segmentCount());

    if (start - lastExtractSeconds_ >= 0.20f) {
        lastExtractSeconds_ = start;
        updatePreview(depth_);
    }

    if (start - lastDiagSeconds_ >= 0.5f) {
        lastDiagSeconds_ = start;
        float dmin = 0.f, dmax = 0.f, dmed = 0.f;
        int valid = 0, total = 0;
        std::vector<float> samples;
        samples.reserve(2048);
        for (size_t i = 0; i < depth_.depth.size(); i += 8) {
            const float d = depth_.depth[i];
            ++total;
            if (d <= 0.f) continue;
            ++valid;
            if (dmin == 0.f || d < dmin) dmin = d;
            if (d > dmax) dmax = d;
            if (samples.size() < 2048) samples.push_back(d);
        }
        if (!samples.empty()) {
            const size_t mid = samples.size() / 2;
            std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
            dmed = samples[mid];
        }

        // If depth intrinsics did not arrive from the bridge, report the ones
        // TangoRecon derives so a wrongly scaled FOV is visible in the log.
        float kfx = frame.Kdepth.fx, kfy = frame.Kdepth.fy;
        float kcx = frame.Kdepth.cx, kcy = frame.Kdepth.cy;
        if (!(kfx > 1.f && kfy > 1.f && kcx > 0.f && kcy > 0.f)) {
            const float sx = static_cast<float>(frame.depthWidth) /
                             static_cast<float>(frame.width);
            const float sy = static_cast<float>(frame.depthHeight) /
                             static_cast<float>(frame.height);
            kfx = frame.K.fx * sx;
            kfy = frame.K.fy * sy;
            kcx = frame.K.cx * sx;
            kcy = frame.K.cy * sy;
        }

        char line[512];
        const double dynamicPct =
            depth_.depth.empty()
                ? 0.0
                : 100.0 * static_cast<double>(movingPixels) /
                      static_cast<double>(depth_.depth.size());
        std::snprintf(line, sizeof(line),
                      "dense ts=%.3f src=t3dr dep=%dx%d feed=%dx%d valid=%.1f%% "
                      "dmin=%.2f dmed=%.2f dmax=%.2f tris=%d seg=%d pend=%d dyn=%.1f%% "
                      "ms=%.0f fuse=%.1f upd=%.1f ex=%.1f pos=%.2f,%.2f,%.2f lag=%.0f "
                      "K=%.1f,%.1f,%.1f,%.1f q=%.3f,%.3f,%.3f,%.3f "
                      "seed=%d run=%d wskip=%d rel=%.2f ang=%.2f",
                      ts, frame.depthWidth, frame.depthHeight, feedW_, feedH_,
                      total > 0 ? 100.0 * valid / total : 0.0,
                      static_cast<double>(dmin), static_cast<double>(dmed),
                      static_cast<double>(dmax), tango_.triangleCount(),
                      tango_.segmentCount(), tango_.pendingSegments(), dynamicPct,
                      static_cast<double>(processMs_.load()), static_cast<double>(fuseMs),
                      tango_.lastUpdateMs(), tango_.lastExtractMs(),
                      static_cast<double>(frame.tx),
                      static_cast<double>(frame.ty), static_cast<double>(frame.tz),
                      depthLagMs, static_cast<double>(kfx), static_cast<double>(kfy),
                      static_cast<double>(kcx), static_cast<double>(kcy),
                      static_cast<double>(frame.qx), static_cast<double>(frame.qy),
                      static_cast<double>(frame.qz), static_cast<double>(frame.qw),
                      fuseSeeds_, fuseRuns_, fuseWarpSkips_,
                      static_cast<double>(lastRelLength_),
                      static_cast<double>(lastRelAngle_));
        pushDiag(line);
    }
    return true;
}

bool Engine::processLegacyDepthFrame(const FrameInput& frame) {
    const float start = nowSeconds();
    const double ts = static_cast<double>(frame.timestampNs) * 1e-9;
    tangoActive_.store(false);

    // Timestamp-free depth exists on some devices; only reject a map that
    // provably trails the camera by most of a second, because fusing such a
    // map at the current pose smears geometry across the view.
    double depthLagMs = std::numeric_limits<double>::quiet_NaN();
    if (frame.depthTimestampNs > 0) {
        depthLagMs = static_cast<double>(frame.timestampNs - frame.depthTimestampNs) * 1e-6;
        if (depthLagMs > 750.0) {
            if (start - lastDiagSeconds_ >= 1.f) {
                lastDiagSeconds_ = start;
                char line[192];
                std::snprintf(line, sizeof(line),
                              "ts=%.3f skip=staledepth lag=%.0fms pos=%.2f,%.2f,%.2f", ts,
                              depthLagMs, static_cast<double>(frame.tx),
                              static_cast<double>(frame.ty), static_cast<double>(frame.tz));
                pushDiag(line);
            }
            return false;
        }
    }

    if (!prepareDepthGrid(frame, depth_)) {
        if (start - lastDiagSeconds_ >= 1.f) {
            lastDiagSeconds_ = start;
            char line[160];
            std::snprintf(line, sizeof(line), "ts=%.3f skip=thinDepth valid=%.1f%%", ts,
                          100.0 * static_cast<double>(hrDepthValid_.load()));
            pushDiag(line);
        }
        return false;
    }

    Keyframe kf;
    kf.id = nextKeyframeId_ + 1;
    buildDepthKeyframe(frame, kf);

    // Dense fusion: every frame is integrated, not just keyframes. The camera
    // moves a few millimetres between frames, so each observation adds a new
    // viewpoint and closes holes that a 1 Hz keyframe schedule left open.
    dirty_.clear();
    integrator_.integrate(kf, depth_, hash_, dirty_);
    queueDirty(dirty_);
    depthSource_.store(1);

    // Carving rewrites free space through the whole hash, which is far more
    // expensive than a single integration; once a second (or after moving far
    // enough for the old observation to matter) removes ghosts reliably.
    const bool moved = hasCarvePosition_ && (kf.t - lastCarvePosition_).length() > 0.35f;
    if (start - lastCarveSeconds_ >= 1.0f || moved) {
        dirty_.clear();
        integrator_.visibilityCarve(kf, depth_, hash_, dirty_);
        queueDirty(dirty_);
        lastCarveSeconds_ = start;
        lastCarvePosition_ = kf.t;
        hasCarvePosition_ = true;
    }

    // Extraction welds and simplifies the whole surface, so it is throttled as
    // well. The TSDF keeps improving between flushes; the visible mesh catches
    // up a few times per second.
    if (start - lastExtractSeconds_ >= 0.30f) {
        flushDirty();
        lastExtractSeconds_ = start;
        updatePreview(depth_);
    }

    nextKeyframeId_ = kf.id;
    keyframeCount_.store(nextKeyframeId_);
    blockCount_.store(static_cast<int>(hash_.size()));

    if (start - lastDiagSeconds_ >= 0.5f) {
        lastDiagSeconds_ = start;
        float dmin = 0.f, dmax = 0.f, dmed = 0.f;
        int valid = 0, total = 0;
        std::vector<float> samples;
        samples.reserve(2048);
        for (size_t i = 0; i < depth_.depth.size(); i += 8) {
            const float d = depth_.depth[i];
            ++total;
            if (d <= 0.f) continue;
            ++valid;
            if (dmin == 0.f || d < dmin) dmin = d;
            if (d > dmax) dmax = d;
            if (samples.size() < 2048) samples.push_back(d);
        }
        if (!samples.empty()) {
            const size_t mid = samples.size() / 2;
            std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
            dmed = samples[mid];
        }
        char line[448];
        std::snprintf(line, sizeof(line),
                      "dense ts=%.3f dep=%dx%d valid=%.1f%% dmin=%.2f dmed=%.2f dmax=%.2f "
                      "tris=%d blocks=%d ms=%.0f pos=%.2f,%.2f,%.2f q=%zu carved=%u lag=%.0f",
                      ts, frame.depthWidth, frame.depthHeight,
                      total > 0 ? 100.0 * valid / total : 0.0,
                      static_cast<double>(dmin), static_cast<double>(dmed),
                      static_cast<double>(dmax), mesher_.triangleCount(),
                      static_cast<int>(hash_.size()), static_cast<double>(processMs_.load()),
                      static_cast<double>(kf.t.x), static_cast<double>(kf.t.y),
                      static_cast<double>(kf.t.z), pendingDirty_.size(),
                      integrator_.carvedVoxels(), depthLagMs);
        pushDiag(line);
    }
    return true;
}

void Engine::buildDepthKeyframe(const FrameInput& frame, Keyframe& kf) const {
    const int dw = frame.depthWidth;
    const int dh = frame.depthHeight;
    kf.timestampNs = frame.timestampNs;
    kf.w = dw;
    kf.h = dh;
    kf.R_gl = matFromQuat(frame.qx, frame.qy, frame.qz, frame.qw);
    kf.t = Vec3{frame.tx, frame.ty, frame.tz};
    kf.K = frame.Kdepth;
    if (kf.K.fx <= 0.f || kf.K.fy <= 0.f) {
        // Older bridge or missing texture intrinsics: derive the depth-grid
        // intrinsics from the CPU intrinsics through the measured affine. Exact
        // for a crop/scale map, which is the normal case.
        const float a = frame.depthTransform[0];
        const float e = frame.depthTransform[4];
        const float c = frame.depthTransform[2];
        const float f = frame.depthTransform[5];
        if (std::fabs(a) > 1e-3f && std::fabs(e) > 1e-3f) {
            const float sx = static_cast<float>(dw) / static_cast<float>(frame.width);
            const float sy = static_cast<float>(dh) / static_cast<float>(frame.height);
            kf.K.fx = sx * a * frame.K.fx;
            kf.K.fy = sy * e * frame.K.fy;
            kf.K.cx = sx * a * frame.K.cx + static_cast<float>(dw) * c;
            kf.K.cy = sy * e * frame.K.cy + static_cast<float>(dh) * f;
        } else {
            kf.K = frame.K;
        }
    }
    kf.K.width = dw;
    kf.K.height = dh;

    // Colour for every depth pixel. The depth grid lives in texture
    // coordinates, so the inverse of the measured affine maps each cell back
    // into the full-resolution CPU image; this is the per-pixel version of the
    // image-region rectangle Google's raw-depth sample uses.
    kf.rgb.resize(static_cast<size_t>(dw) * dh * 3);
    const int W = frame.width;
    const int H = frame.height;
    const int cw = W / 2;
    const float* m = frame.depthTransform;
    const float det = m[0] * m[4] - m[1] * m[3];
    const bool invertible = std::fabs(det) > 1e-5f;
    for (int y = 0; y < dh; ++y) {
        const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(dh) - m[5];
        for (int x = 0; x < dw; ++x) {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(dw) - m[2];
            float nx, ny;
            if (invertible) {
                nx = (m[4] * u - m[1] * v) / det;
                ny = (m[0] * v - m[3] * u) / det;
            } else {
                nx = (static_cast<float>(x) + 0.5f) / static_cast<float>(dw);
                ny = (static_cast<float>(y) + 0.5f) / static_cast<float>(dh);
            }
            const int px = std::max(0, std::min(W - 1, static_cast<int>(nx * W)));
            const int py = std::max(0, std::min(H - 1, static_cast<int>(ny * H)));
            uint8_t r, g, b;
            yuvToRgb(frame.y[static_cast<size_t>(py) * W + px],
                     frame.u[static_cast<size_t>(py / 2) * cw + (px / 2)],
                     frame.v[static_cast<size_t>(py / 2) * cw + (px / 2)], r, g, b);
            const size_t dst = (static_cast<size_t>(y) * dw + x) * 3;
            kf.rgb[dst + 0] = r;
            kf.rgb[dst + 1] = g;
            kf.rgb[dst + 2] = b;
        }
    }
}

bool Engine::prepareDepthGrid(const FrameInput& frame, DepthResult& depth,
                              float minValidFraction) {
    const int dw = frame.depthWidth;
    const int dh = frame.depthHeight;
    if (dw <= 1 || dh <= 1) return false;
    const size_t count = static_cast<size_t>(dw) * dh;
    if (frame.depthMm.size() != count) return false;

    // ARCore extrapolates beyond ~6 m and the reconstruction's own range ends
    // there, so farther samples only cost fusion time and import noise.
    const uint16_t maxMm = 6000;
    std::vector<float> src(count, 0.f);
    for (size_t i = 0; i < count; ++i) {
        const uint16_t mm = frame.depthMm[i];
        if (mm > 0 && mm <= maxMm) src[i] = static_cast<float>(mm) * 0.001f;
    }

    depth.resize(dw, dh);
    int valid = 0;
    for (int y = 0; y < dh; ++y) {
        for (int x = 0; x < dw; ++x) {
            float window[9];
            int n = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= dh) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= dw) continue;
                    const float d = src[static_cast<size_t>(yy) * dw + xx];
                    if (d > 0.f) window[n++] = d;
                }
            }
            const size_t q = static_cast<size_t>(y) * dw + x;
            const float centre = src[q];
            float value = 0.f;
            if (centre > 0.f) {
                if (n >= 3) {
                    // Insertion sort on at most nine values.
                    for (int i = 1; i < n; ++i) {
                        const float key = window[i];
                        int j = i - 1;
                        while (j >= 0 && window[j] > key) {
                            window[j + 1] = window[j];
                            --j;
                        }
                        window[j + 1] = key;
                    }
                    const float median = window[n / 2];
                    // A sample far from its neighbourhood is a flying pixel on
                    // a silhouette, not a surface; the TSDF would drag it into
                    // a sheet between foreground and background.
                    const float tol = std::max(0.04f, 0.05f * median);
                    if (std::fabs(centre - median) <= tol) value = median;
                } else {
                    value = centre;
                }
            } else if (n >= 6) {
                float lo = window[0], hi = window[0];
                for (int i = 1; i < n; ++i) {
                    lo = std::min(lo, window[i]);
                    hi = std::max(hi, window[i]);
                }
                // Only a one-pixel dropout on a continuous surface is filled;
                // a gap at a depth step is a real hole in the observation.
                if (hi - lo <= 0.08f) {
                    std::sort(window, window + n);
                    value = window[n / 2];
                }
            }
            if (value > 0.f) {
                depth.depth[q] = value;
                depth.confidence[q] = 1.f;
                ++valid;
            }
        }
    }
    // Outdoors a large share of samples are isolated speckles: sunlight and
    // glossy asphalt break the projected pattern and ARCore returns single
    // pixels that the 3x3 median cannot reject because they have no
    // neighbourhood. Two valid 8-neighbours is enough to keep a real surface
    // at this resolution; junk without support becomes a hole, not a sheet.
    if (outdoor_) {
        std::vector<uint8_t> keep(count, 0);
        for (int y = 0; y < dh; ++y) {
            for (int x = 0; x < dw; ++x) {
                const size_t q = static_cast<size_t>(y) * dw + x;
                if (depth.depth[q] <= 0.f) continue;
                int support = 0;
                for (int dy = -1; dy <= 1 && support < 2; ++dy) {
                    const int yy = y + dy;
                    if (yy < 0 || yy >= dh) continue;
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int xx = x + dx;
                        if (xx < 0 || xx >= dw) continue;
                        if (depth.depth[static_cast<size_t>(yy) * dw + xx] > 0.f) {
                            ++support;
                            if (support >= 2) break;
                        }
                    }
                }
                if (support >= 2) keep[q] = 1;
            }
        }
        int kept = 0;
        for (size_t q = 0; q < count; ++q) {
            if (depth.depth[q] <= 0.f) continue;
            if (keep[q]) {
                ++kept;
            } else {
                depth.depth[q] = 0.f;
                depth.confidence[q] = 0.f;
            }
        }
        valid = kept;
    }
    hrDepthValid_.store(static_cast<float>(valid) / static_cast<float>(count));
    return valid >= static_cast<int>(static_cast<float>(count) * minValidFraction);
}

float Engine::meanLuma(const FrameInput& frame) const {
    const size_t count = frame.y.size();
    if (count == 0) return 0.f;
    uint64_t sum = 0;
    for (size_t i = 0; i < count; i += 13) sum += frame.y[i];
    return static_cast<float>(sum) / static_cast<float>((count + 12) / 13);
}

void Engine::sampleScene(const DepthResult& depth, float luma) {
    if (sceneDecided_) return;
    const int w = depth.w;
    const int h = depth.h;
    if (w <= 2 || h <= 2) return;
    const int skyRows = std::max(1, h / 4);
    int skyTotal = 0;
    int skyInvalid = 0;
    int valid = 0;
    int far = 0;
    std::vector<float> samples;
    samples.reserve(static_cast<size_t>(w + 1) * (h + 1) / 4);
    for (int y = 0; y < h; y += 2) {
        for (int x = 0; x < w; x += 2) {
            const float d = depth.depth[static_cast<size_t>(y) * w + x];
            const bool skyRow = y < skyRows;
            if (skyRow) ++skyTotal;
            if (d <= 0.f) {
                if (skyRow) ++skyInvalid;
                continue;
            }
            ++valid;
            if (d > 4.5f) ++far;
            samples.push_back(d);
        }
    }
    // A frame with almost no usable depth (direct sunlight) carries no
    // information about the scene; counting it would bias every average.
    if (valid < 60) return;
    float median = 0.f;
    const size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    median = samples[mid];

    sceneSkySum_ += skyTotal > 0
                        ? static_cast<float>(skyInvalid) / static_cast<float>(skyTotal)
                        : 0.f;
    sceneFarSum_ += static_cast<float>(far) / static_cast<float>(valid);
    sceneMedianSum_ += median;
    sceneLumaSum_ += luma;
    if (++sceneSamples_ < 20) return;

    sceneDecided_ = true;
    const float sky = sceneSkySum_ / static_cast<float>(sceneSamples_);
    const float farFraction = sceneFarSum_ / static_cast<float>(sceneSamples_);
    const float medianDepth = sceneMedianSum_ / static_cast<float>(sceneSamples_);
    const float meanBrightness = sceneLumaSum_ / static_cast<float>(sceneSamples_);
    // Sky + distant surfaces is a street; a large median alone also counts
    // (an open yard without visible sky in the crop). Sunlit pavement pointed
    // straight down has neither, so brightness is the last vote: outdoors the
    // camera exposes the scene several stops brighter than a room.
    outdoor_ = (sky > 0.5f && farFraction > 0.10f) || farFraction > 0.30f ||
               medianDepth > 3.0f ||
               (meanBrightness > 145.f && farFraction > 0.02f && medianDepth > 1.4f);
    char line[224];
    std::snprintf(line, sizeof(line),
                  "scene=%s sky=%.2f far=%.2f med=%.2f luma=%.0f frames=%d",
                  outdoor_ ? "outdoor" : "indoor", static_cast<double>(sky),
                  static_cast<double>(farFraction), static_cast<double>(medianDepth),
                  static_cast<double>(meanBrightness), sceneSamples_);
    pushDiag(line);
}

int Engine::fuseDepth(const FrameInput& frame, const DepthResult& measured,
                      std::vector<uint8_t>& mask) {
    const int w = measured.w;
    const int h = measured.h;
    const size_t count = measured.depth.size();
    mask.assign(count, 0);
    if (w <= 1 || h <= 1 || count == 0) return 0;

    Intrinsics k = frame.Kdepth;
    if (!(k.fx > 1.f && k.fy > 1.f && k.cx > 0.f && k.cy > 0.f)) {
        const float sx = static_cast<float>(w) / static_cast<float>(frame.width);
        const float sy = static_cast<float>(h) / static_cast<float>(frame.height);
        k.fx = frame.K.fx * sx;
        k.fy = frame.K.fy * sy;
        k.cx = frame.K.cx * sx;
        k.cy = frame.K.cy * sy;
    }
    k.width = w;
    k.height = h;
    fusionK_ = k;

    // Depth noise grows with range; the measurement variance drives how much
    // of each new sample the filter accepts.
    auto measSigma = [](float z) { return 0.007f + 0.0045f * z * z; };

    // The filter state is only invalid when the grid itself changed size; a
    // large pose step does not wipe the history. It only invalidates the warp
    // for that frame, and the measurement is used directly, which is what a
    // reseed would have done anyway - without throwing away every pixel's
    // accumulated confidence.
    const bool haveGrid = fusedDepth_.size() == count && prevDepthW_ == w &&
                          prevDepthH_ == h;
    const bool reseed = !haveGrid;
    float relLength = 0.f;
    float relAngle = 0.f;
    bool dropPrior = false;
    if (haveGrid) {
        const Mat3 rPrev = matFromQuat(prevQ_[0], prevQ_[1], prevQ_[2], prevQ_[3]);
        const Mat3 rCur = matFromQuat(frame.qx, frame.qy, frame.qz, frame.qw);
        const Mat3 rCurT = rCur.transposed();
        const Vec3 tRel = rCurT * (Vec3(prevT_[0], prevT_[1], prevT_[2]) -
                                   Vec3(frame.tx, frame.ty, frame.tz));
        relLength = tRel.length();
        const Mat3 rRel = rCurT * rPrev;
        const float trace = rRel.at(0, 0) + rRel.at(1, 1) + rRel.at(2, 2);
        relAngle = std::acos(clampf((trace - 1.f) * 0.5f, -1.f, 1.f));
        // A relocalisation or a dropped frame is not a small pose change; the
        // prediction would land somewhere else entirely.
        dropPrior = relLength > 0.35f || relAngle > 0.35f ||
                    !(prevKdepth_.fx > 1.f && prevKdepth_.fy > 1.f);
    }

    auto seed = [&]() {
        fusedDepth_.assign(count, 0.f);
        fusedVar_.assign(count, 1.f);
        fusedAge_.assign(count, 0);
        for (size_t i = 0; i < count; ++i) {
            const float z = measured.depth[i];
            if (z <= 0.f) continue;
            fusedDepth_[i] = z;
            const float s = measSigma(z);
            fusedVar_[i] = s * s;
        }
        predDepth_.assign(count, 0.f);
        predVar_.assign(count, 1.f);
        predValid_.assign(count, 0);
        lumaGrid_.assign(count, 0);
        motionScore_.assign(count, 0);
        prevDepthW_ = w;
        prevDepthH_ = h;
        prevKdepth_ = k;
        prevQ_[0] = frame.qx; prevQ_[1] = frame.qy;
        prevQ_[2] = frame.qz; prevQ_[3] = frame.qw;
        prevT_[0] = frame.tx; prevT_[1] = frame.ty; prevT_[2] = frame.tz;
    };

    if (reseed) {
        seed();
        ++fuseSeeds_;
        return 0;
    }

    ++fuseRuns_;
    lastRelLength_ = relLength;
    lastRelAngle_ = relAngle;
    if (dropPrior) ++fuseWarpSkips_;

    for (uint8_t& score : motionScore_) {
        if (score > 0) --score;
    }

    // Warp the previous fused surface into the current view. The nearest
    // sample wins when several previous pixels project to the same one, so a
    // surface behind a silhouette cannot overwrite the foreground.
    const Mat3 rPrev = matFromQuat(prevQ_[0], prevQ_[1], prevQ_[2], prevQ_[3]);
    const Mat3 rCur = matFromQuat(frame.qx, frame.qy, frame.qz, frame.qw);
    const Mat3 rCurT = rCur.transposed();
    const Vec3 prevT(prevT_[0], prevT_[1], prevT_[2]);
    const Vec3 curT(frame.tx, frame.ty, frame.tz);
    const Intrinsics& kPrev = prevKdepth_;

    std::fill(predValid_.begin(), predValid_.end(), 0);
    // Pose/registration error grows with how far the camera moved; that error
    // is process noise, so the filter trusts the measurement more while the
    // user walks and averages harder while the phone is still.
    const float qProcess = (0.01f + 0.15f * relLength) * (0.01f + 0.15f * relLength);
    if (!dropPrior) {
        for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            const size_t p = static_cast<size_t>(v) * w + u;
            const float zPrev = fusedDepth_[p];
            if (zPrev <= 0.f) continue;
            if (fusedAge_[p] > 4) continue;
            const float xo = (static_cast<float>(u) - kPrev.cx) * zPrev / kPrev.fx;
            const float yo = (static_cast<float>(v) - kPrev.cy) * zPrev / kPrev.fy;
            const Vec3 world = rotate(rPrev, Vec3(xo, -yo, -zPrev)) + prevT;
            const Vec3 pCam = rotate(rCurT, world - curT);
            const float zExpected = -pCam.z;
            if (zExpected < 0.15f) continue;
            const int u2 = static_cast<int>(
                std::lround(k.fx * (pCam.x / zExpected) + k.cx));
            const int v2 = static_cast<int>(
                std::lround(k.fy * (-pCam.y / zExpected) + k.cy));
            if (u2 < 0 || u2 >= w || v2 < 0 || v2 >= h) continue;
            const size_t q = static_cast<size_t>(v2) * w + u2;
            if (predValid_[q] && predDepth_[q] <= zExpected) continue;
            predValid_[q] = 1;
            predDepth_[q] = zExpected;
            predVar_[q] = std::min(0.09f, fusedVar_[p] + qProcess);
        }
        }
    }

    int moving = 0;
    const float tolScale = outdoor_ ? 1.35f : 1.f;
    // Holes are held only across a small pose change. Holding a surface while
    // the phone pans would paint the old view into the new one.
    const bool holdAllowed = relLength < 0.12f && relAngle < 0.10f;
    constexpr int kMaxHoldFrames = 4;
    for (size_t i = 0; i < count; ++i) {
        const float z = measured.depth[i];
        if (z > 0.f) {
            const float s = measSigma(z);
            const float varM = s * s;
            if (predValid_[i]) {
                const float pred = predDepth_[i];
                const float varP = predVar_[i];
                const float diff = std::fabs(z - pred);
                const float gate =
                    std::max(std::max(0.08f, 0.035f * z) * tolScale,
                             2.5f * std::sqrt(varP + varM));
                if (diff > gate) {
                    // Outside what noise can explain: the predicted surface is
                    // displaced. The measurement wins at once and the pixel
                    // votes for the moving-object mask.
                    fusedDepth_[i] = z;
                    fusedVar_[i] = varM;
                    fusedAge_[i] = 0;
                    if (motionScore_[i] < 3) ++motionScore_[i];
                    continue;
                }
                const float gain = varP / (varP + varM);
                fusedDepth_[i] = pred + gain * (z - pred);
                fusedVar_[i] = std::max(varM * 0.2f, (1.f - gain) * varP);
                fusedAge_[i] = 0;
            } else {
                fusedDepth_[i] = z;
                fusedVar_[i] = varM;
                fusedAge_[i] = 0;
            }
        } else if (predValid_[i] && holdAllowed && fusedAge_[i] < kMaxHoldFrames) {
            // The sensor dropped this pixel. Holding the prediction closes a
            // hole in the observation instead of carving a hole in the mesh.
            fusedDepth_[i] = predDepth_[i];
            fusedVar_[i] = std::min(0.09f, predVar_[i] * 1.2f);
            ++fusedAge_[i];
        } else {
            fusedDepth_[i] = 0.f;
            fusedVar_[i] = 1.f;
            fusedAge_[i] = 0;
        }
    }

    // Only connected patches large enough to be a body, vehicle or animal are
    // treated as moving. Single-pixel depth noise near silhouettes used to
    // spawn the scattered amber quads ("orange tickets") seen on the street.
    constexpr int kMinMovingPixels = 96;
    std::vector<uint8_t> visited(count, 0);
    std::vector<int> stack;
    std::vector<int> component;
    for (size_t start = 0; start < count; ++start) {
        if (visited[start] || motionScore_[start] < 3) continue;
        component.clear();
        stack.clear();
        stack.push_back(static_cast<int>(start));
        visited[start] = 1;
        while (!stack.empty()) {
            const int idx = stack.back();
            stack.pop_back();
            component.push_back(idx);
            const int x = idx % w;
            const int y = idx / w;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    const int n = yy * w + xx;
                    if (visited[n] || motionScore_[n] < 3) continue;
                    visited[n] = 1;
                    stack.push_back(n);
                }
            }
        }
        if (static_cast<int>(component.size()) < kMinMovingPixels) continue;
        for (const int idx : component) {
            mask[idx] = 1;
            if (measured.depth[idx] > 0.f) ++moving;
        }
    }

    prevDepthW_ = w;
    prevDepthH_ = h;
    prevKdepth_ = k;
    prevQ_[0] = frame.qx; prevQ_[1] = frame.qy;
    prevQ_[2] = frame.qz; prevQ_[3] = frame.qw;
    prevT_[0] = frame.tx; prevT_[1] = frame.ty; prevT_[2] = frame.tz;
    return moving;
}

void Engine::buildFusionFeed(const FrameInput& frame) {
    const int w = depth_.w;
    const int h = depth_.h;
    feedW_ = 0;
    feedH_ = 0;
    feedDepthMm_.clear();
    if (w <= 1 || h <= 1 || fusedDepth_.size() != static_cast<size_t>(w) * h) return;
    const size_t count = fusedDepth_.size();
    if (lumaGrid_.size() != count) lumaGrid_.assign(count, 0);
    const bool haveMask = motionMask_.size() == count;
    feedDepth_ = fusedDepth_;

    // CPU luma resampled onto the depth grid through the inverse of the
    // measured depth transform, so the fill can follow colour edges.
    const float* m = frame.depthTransform;
    const float det = m[0] * m[4] - m[1] * m[3];
    const bool affine = frame.width > 1 && frame.height > 1 &&
                        std::fabs(det) > 1e-6f &&
                        frame.y.size() == static_cast<size_t>(frame.width) * frame.height;
    if (affine) {
        const float i00 = m[4] / det;
        const float i01 = -m[1] / det;
        const float i02 = (m[1] * m[5] - m[4] * m[2]) / det;
        const float i10 = -m[3] / det;
        const float i11 = m[0] / det;
        const float i12 = (m[3] * m[2] - m[0] * m[5]) / det;
        for (int y = 0; y < h; ++y) {
            const float dv = (static_cast<float>(y) + 0.5f) / static_cast<float>(h);
            for (int x = 0; x < w; ++x) {
                const float du = (static_cast<float>(x) + 0.5f) / static_cast<float>(w);
                const float cu = i00 * du + i01 * dv + i02;
                const float cv = i10 * du + i11 * dv + i12;
                const int px = static_cast<int>(
                    clampf(cu * static_cast<float>(frame.width), 0.f,
                           static_cast<float>(frame.width - 1)));
                const int py = static_cast<int>(
                    clampf(cv * static_cast<float>(frame.height), 0.f,
                           static_cast<float>(frame.height - 1)));
                lumaGrid_[static_cast<size_t>(y) * w + x] =
                    frame.y[static_cast<size_t>(py) * frame.width + px];
            }
        }
    } else {
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                lumaGrid_[static_cast<size_t>(y) * w + x] =
                    frame.y[static_cast<size_t>(y * frame.height / h) * frame.width +
                            x * frame.width / w];
            }
        }
    }

    // Two joint-bilateral fill passes: a hole is closed only from neighbours
    // with a similar colour and a consistent depth, so a doorway or a rim is
    // not bridged. This is the image side of the fusion: where the sensor has
    // no return, the colour image decides where the surface continues.
    for (int pass = 0; pass < 2; ++pass) {
        const std::vector<float> prev = feedDepth_;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const size_t i = static_cast<size_t>(y) * w + x;
                if (feedDepth_[i] > 0.f) continue;
                if (haveMask && motionMask_[i]) continue;
                const uint8_t luma = lumaGrid_[i];
                float donors[8];
                int n = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    const int yy = y + dy;
                    if (yy < 0 || yy >= h) continue;
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int xx = x + dx;
                        if (xx < 0 || xx >= w) continue;
                        const size_t j = static_cast<size_t>(yy) * w + xx;
                        if (haveMask && motionMask_[j]) continue;
                        const float d = prev[j];
                        if (d <= 0.f) continue;
                        const int dl = lumaGrid_[j] > luma ? lumaGrid_[j] - luma
                                                           : luma - lumaGrid_[j];
                        if (dl > 16) continue;
                        donors[n++] = d;
                    }
                }
                if (n < 2) continue;
                float lo = donors[0], hi = donors[0], sum = 0.f;
                for (int k = 0; k < n; ++k) {
                    lo = std::min(lo, donors[k]);
                    hi = std::max(hi, donors[k]);
                    sum += donors[k];
                }
                // Donors on both sides of a depth step would average into a
                // sheet; leave the pixel as a real hole instead.
                if (hi - lo > std::max(0.06f, 0.06f * lo)) continue;
                feedDepth_[i] = sum / static_cast<float>(n);
            }
        }
    }

    // A light guided smooth on what is left: the temporal filter removes most
    // of the noise, and this takes out the pixel steps that marching cubes
    // turns into the lumpy "melted" surface seen on grazing ground. It runs on
    // this copy only, never on the filter state.
    const std::vector<float> in = feedDepth_;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t i = static_cast<size_t>(y) * w + x;
            const float centre = in[i];
            if (centre <= 0.f) continue;
            if (haveMask && motionMask_[i]) continue;
            const uint8_t luma = lumaGrid_[i];
            const float scale = 0.012f + 0.012f * centre;
            float sum = centre;
            float weightSum = 1.f;
            for (int dy = -1; dy <= 1; ++dy) {
                const int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    const size_t j = static_cast<size_t>(yy) * w + xx;
                    if (haveMask && motionMask_[j]) continue;
                    const float d = in[j];
                    if (d <= 0.f) continue;
                    const float dl = static_cast<float>(lumaGrid_[j]) - luma;
                    const float dd = (d - centre) / scale;
                    const float weight = 1.f / (1.f + (dl / 14.f) * (dl / 14.f) + dd * dd);
                    sum += d * weight;
                    weightSum += weight;
                }
            }
            feedDepth_[i] = sum / weightSum;
        }
    }

    const int s = feedScale_ == 2 && (w % 2) == 0 && (h % 2) == 0 ? 2 : 1;
    feedScale_ = s;
    feedW_ = w / s;
    feedH_ = h / s;
    feedDepthMm_.assign(static_cast<size_t>(feedW_) * feedH_, 0);

    auto toMm = [](float d) {
        return static_cast<uint16_t>(std::min(65535.f, std::round(d * 1000.f)));
    };

    if (s == 1) {
        for (size_t i = 0; i < count; ++i) {
            if (haveMask && motionMask_[i]) continue;
            const float d = feedDepth_[i];
            if (d > 0.f) feedDepthMm_[i] = toMm(d);
        }
    } else {
        for (int y = 0; y < feedH_; ++y) {
            for (int x = 0; x < feedW_; ++x) {
                float lo = 0.f, hi = 0.f, sum = 0.f;
                int n = 0;
                for (int dy = 0; dy < 2; ++dy) {
                    for (int dx = 0; dx < 2; ++dx) {
                        const size_t p = static_cast<size_t>(y * 2 + dy) * w + (x * 2 + dx);
                        if (haveMask && motionMask_[p]) continue;
                        const float d = feedDepth_[p];
                        if (d <= 0.f) continue;
                        if (n == 0) {
                            lo = hi = d;
                        } else {
                            lo = std::min(lo, d);
                            hi = std::max(hi, d);
                        }
                        sum += d;
                        ++n;
                    }
                }
                if (n == 0) continue;
                // A block that straddles a depth step takes the foreground
                // surface; averaging would round the silhouette away.
                const float value = hi - lo > 0.12f ? lo : sum / static_cast<float>(n);
                feedDepthMm_[static_cast<size_t>(y) * feedW_ + x] = toMm(value);
            }
        }
    }

    feedK_ = fusionK_;
    const float inv = 1.f / static_cast<float>(s);
    feedK_.fx = fusionK_.fx * inv;
    feedK_.fy = fusionK_.fy * inv;
    feedK_.cx = (fusionK_.cx + 0.5f * static_cast<float>(s - 1)) * inv;
    feedK_.cy = (fusionK_.cy + 0.5f * static_cast<float>(s - 1)) * inv;
    feedK_.width = feedW_;
    feedK_.height = feedH_;
}

void Engine::buildDynamicMesh(const FrameInput& frame, const DepthResult& depth,
                              const std::vector<uint8_t>& mask) {
    auto mesh = std::make_shared<MeshData>();
    const int w = depth.w;
    const int h = depth.h;
    const Intrinsics& k = frame.Kdepth;
    const Mat3 r = matFromQuat(frame.qx, frame.qy, frame.qz, frame.qw);
    const Vec3 right = rotate(r, Vec3(1.f, 0.f, 0.f));
    const Vec3 up = rotate(r, Vec3(0.f, 1.f, 0.f));
    const Vec3 camera(frame.tx, frame.ty, frame.tz);

    if (w > 1 && h > 1 && k.fx > 1.f && k.fy > 1.f &&
        mask.size() == depth.depth.size()) {
        // A moving body is far sparser than the room; a few thousand camera-
        // facing quads show it live without a second reconstruction volume.
        constexpr int kMaxQuads = 4096;
        mesh->vertices.reserve(static_cast<size_t>(kMaxQuads) * 4);
        mesh->indices.reserve(static_cast<size_t>(kMaxQuads) * 6);
        int quads = 0;
        for (int v = 0; v < h && quads < kMaxQuads; ++v) {
            for (int u = 0; u < w && quads < kMaxQuads; ++u) {
                const size_t idx = static_cast<size_t>(v) * w + u;
                if (!mask[idx]) continue;
                const float z = depth.depth[idx];
                if (z <= 0.f) continue;
                const float xo = (static_cast<float>(u) - k.cx) * z / k.fx;
                const float yo = (static_cast<float>(v) - k.cy) * z / k.fy;
                const Vec3 centre =
                    rotate(r, Vec3(xo, -yo, -z)) + camera;
                const float half = 0.75f * z / std::min(k.fx, k.fy);
                const Vec3 normal = (camera - centre).normalized();
                const Vec3 corners[4] = {
                    centre - right * half - up * half,
                    centre + right * half - up * half,
                    centre + right * half + up * half,
                    centre - right * half + up * half,
                };
                const uint32_t base = static_cast<uint32_t>(mesh->vertices.size());
                for (const Vec3& p : corners) {
                    MeshVertex vertex{};
                    vertex.px = p.x; vertex.py = p.y; vertex.pz = p.z;
                    vertex.nx = normal.x; vertex.ny = normal.y; vertex.nz = normal.z;
                    vertex.r = 255; vertex.g = 168; vertex.b = 56; vertex.a = 255;
                    mesh->vertices.push_back(vertex);
                }
                mesh->indices.push_back(base + 0);
                mesh->indices.push_back(base + 1);
                mesh->indices.push_back(base + 2);
                mesh->indices.push_back(base + 0);
                mesh->indices.push_back(base + 2);
                mesh->indices.push_back(base + 3);
                ++quads;
            }
        }
        mesh->triangleCount = static_cast<uint32_t>(mesh->indices.size() / 3);
    }

    mesh->version = ++dynamicVersion_;
    {
        std::lock_guard<std::mutex> lock(dynamicMutex_);
        dynamicSnapshot_ = mesh;
    }
    dynamicPixels_.store(static_cast<int>(mesh->indices.size() / 6));
}

std::shared_ptr<const MeshData> Engine::dynamicMesh() const {
    std::lock_guard<std::mutex> lock(dynamicMutex_);
    return dynamicSnapshot_;
}

void Engine::queueDirty(const std::vector<VoxelBlock*>& blocks) {
    for (VoxelBlock* block : blocks) {
        if (!block) continue;
        if (pendingSet_.insert(block).second) pendingDirty_.push_back(block);
    }
}

void Engine::flushDirty() {
    if (pendingDirty_.empty()) return;
    std::vector<VoxelBlock*> dirty = std::move(pendingDirty_);
    pendingDirty_.clear();
    pendingSet_.clear();
    expandDirtyNeighbours(hash_, dirty);
    mesher_.extractDirty(hash_, dirty);
    // Eviction removes blocks, so it must never run while dirty pointers from
    // an earlier frame are still queued.
    evictIfNeeded(lastCamera_);
}

void Engine::evictIfNeeded(const Vec3& camera) {
    const size_t maxBlocks = hash_.maxBlocks();
    if (hash_.size() <= maxBlocks * 9 / 10) return;
    hash_.evictFarthest(camera, 10.f, maxBlocks * 4 / 5);
    mesher_.pruneBlocks(hash_);
    blockCount_.store(static_cast<int>(hash_.size()));
}

void Engine::updatePreview(const DepthResult& depth) {
    if (depth.w <= 0 || depth.h <= 0) return;
    const bool haveFused = fusedDepth_.size() == depth.depth.size();
    const bool haveVar = fusedVar_.size() == depth.depth.size();

    std::vector<uint32_t> depthPixels(static_cast<size_t>(kPreviewW) * kPreviewH);
    std::vector<uint32_t> confPixels(static_cast<size_t>(kPreviewW) * kPreviewH);

    const float invNear = 1.f / 0.5f;
    const float invFar = 1.f / 5.0f;

    for (int py = 0; py < kPreviewH; ++py) {
        const int sy = std::min(depth.h - 1, py * depth.h / kPreviewH);
        for (int px = 0; px < kPreviewW; ++px) {
            const int sx = std::min(depth.w - 1, px * depth.w / kPreviewW);
            const size_t src = static_cast<size_t>(sy) * depth.w + sx;
            const size_t dst = static_cast<size_t>(py) * kPreviewW + px;
            const float d = haveFused ? fusedDepth_[src] : depth.depth[src];
            if (d <= 0.f) {
                depthPixels[dst] = 0xFF0E1114u;
            } else {
                const float invD = clampf(1.f / d, invFar, invNear);
                const float t = (invD - invFar) / (invNear - invFar);
                depthPixels[dst] = turboColor(t);
            }
            // Confidence is the fused uncertainty, not a constant: bright where
            // many observations agreed, dark where the value is a short-lived
            // prediction or a colour-guided fill.
            float c = 0.f;
            if (haveVar && d > 0.f) {
                const float variance = fusedVar_[src];
                c = 1.f / (1.f + 300.f * variance);
            }
            const uint32_t gray = static_cast<uint32_t>(clampf(c, 0.f, 1.f) * 255.f);
            confPixels[dst] = 0xFF000000u | (gray << 16) | (gray << 8) | gray;
        }
    }

    std::lock_guard<std::mutex> lock(previewMutex_);
    previewDepth_.swap(depthPixels);
    previewConfidence_.swap(confPixels);
}

}  // namespace recon
