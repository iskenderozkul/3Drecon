#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/frame.h"
#include "core/keyframe_manager.h"
#include "mesh/mesher.h"
#include "mvs/refine.h"
#include "mvs/stereo.h"
#include "tango/tango_recon.h"
#include "tsdf/integrator.h"
#include "tsdf/voxel_hash.h"

namespace recon {

// Owns the whole reconstruction state and runs it on a dedicated worker
// thread. With ARCore depth available every frame is fused straight from the
// depth grid into the TSDF (dense fusion, like 3DLiveScanner); visibility
// carving and Marching Cubes are throttled so the worker keeps up with the
// camera. Without depth it falls back to half-res keyframe MVS.
// The GL thread only submits frames and reads the immutable mesh snapshot.
class Engine {
public:
    static constexpr int kPreviewW = 128;
    static constexpr int kPreviewH = 96;

    Engine();
    ~Engine();

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Copies nothing: the frame planes are moved out of the caller's buffers.
    void submitFrame(FrameInput&& frame);

    void setScanning(bool scanning);
    bool scanning() const { return scanning_.load(); }

    // Clears keyframes, TSDF and mesh on the worker thread.
    void reset();

    std::shared_ptr<const MeshData> mesh() const;
    // Moving surfaces (people, animals, vehicles) are kept out of the static
    // volume and published as their own short-lived mesh, so they are visible
    // without smearing the room around them.
    std::shared_ptr<const MeshData> dynamicMesh() const;
    // Re-meshes every block on the worker thread first, so the file reflects
    // all observations in one state rather than the incremental edit history.
    bool exportMesh(const std::string& path, const std::string& format);

    // Depth / confidence preview buffers, ARGB (kPreviewW * kPreviewH).
    void preview(uint32_t* depthOut, uint32_t* confidenceOut) const;

    float processMs() const { return processMs_.load(); }
    float fps() const { return fps_.load(); }
    // Total keyframes fused since the last reset (not the reference pool size).
    int keyframeCount() const { return keyframeCount_.load(); }
    int framesProcessed() const { return framesProcessed_.load(); }
    int blockCount() const { return blockCount_.load(); }
    int triangleCount() const;
    int vertexCount() const;
    // 0 = none yet, 1 = ARCore Depth API, 2 = on-device MVS fallback.
    int depthSource() const { return depthSource_.load(); }
    // Stage the worker is in right now. A frozen scan leaves this at the phase
    // that never finished, which is the only clue available without logcat.
    // 0 idle, 1 prepare, 2 fuse, 3 feed, 4 integrate, 5 mesh, 6 remesh.
    int phase() const { return phase_.load(); }
    // One line per processed keyframe, drained by the diagnostics log.
    std::string drainDiagnostics();

private:
    void workerLoop();
    bool processFrame(const FrameInput& frame);
    // Dense path: every frame with an ARCore depth map is fused by the Tango
    // 3DR core. Returns false when the frame was skipped (stale depth,
    // relocalisation, too few valid pixels).
    bool processDepthFrame(const FrameInput& frame);
    // Fallback used only if the external reconstruction core cannot be loaded.
    bool processLegacyDepthFrame(const FrameInput& frame);
    // Sparse fallback: keyframe selection + MVS + refinement.
    bool processMvsFrame(const FrameInput& frame);
    void performReset();
    // Packs the depth grid as a keyframe: depth-resolution pose, depth-grid
    // intrinsics and colour sampled through the measured CPU-image affine.
    void buildDepthKeyframe(const FrameInput& frame, Keyframe& kf) const;
    // Converts millimetres to metres and removes single-pixel flyers. Works at
    // the native depth resolution, so no upsampling is involved. When ARCore
    // supplies a raw-depth confidence image, pixels the sensor marks as
    // unreliable are dropped before the median pass.
    bool prepareDepthGrid(const FrameInput& frame, DepthResult& depth,
                          float minValidFraction = 0.125f);
    // Decides once per session whether the phone is in a room or on a street
    // (sky under the top quarter, far surfaces, median range, brightness) and
    // uses it for the depth cleaning rules. No user-facing setting.
    void sampleScene(const DepthResult& depth, float meanLuma);
    // Per-pixel temporal fusion. The previous fused surface is warped into the
    // current camera and blended with the measurement through a Kalman step:
    // standing still averages many frames into one smooth surface, and the
    // process noise grows with camera motion so walking follows the sensor.
    // Pixels the sensor drops are held for a few frames. Also returns the
    // moving-object mask. The filter state stays unsmoothed: the edge-preserving
    // pass runs on the copy handed to the reconstruction every frame, so
    // repeated smoothing cannot slowly melt detail away.
    int fuseDepth(const FrameInput& frame, const DepthResult& measured,
                  std::vector<uint8_t>& mask);
    // Builds the reconstruction feed from the filter state: colour-guided hole
    // fill and an edge-preserving smooth, then resampling to half resolution
    // when the fusion step is the frame bottleneck.
    void buildFusionFeed(const FrameInput& frame);
    float meanLuma(const FrameInput& frame) const;
    // Builds the transient mesh for the pixels masked as moving.
    void buildDynamicMesh(const FrameInput& frame, const DepthResult& depth,
                          const std::vector<uint8_t>& mask);
    void buildKeyframe(const FrameInput& frame, Keyframe& kf) const;
    void queueDirty(const std::vector<VoxelBlock*>& blocks);
    // Re-triangulates every queued block in one go and publishes the snapshot.
    void flushDirty();
    void updatePreview(const DepthResult& depth);
    void evictIfNeeded(const Vec3& camera);
    void pushDiag(std::string line);

    KeyframeManager keyframes_{14};
    StereoEngine stereo_;
    DepthRefiner refiner_;
    // 2 cm voxels with a 5 cm truncation band. The depth itself is smooth at
    // this range (ARCore 160x90, ~1 m median in the measured runs), so the grid
    // is not the accuracy limit; the truncation band is what inflates objects.
    // A 3 cm grid with 9 cm truncation rounded every rim by up to a voxel and
    // grew a lip around each silhouette, which read as "the mesh is bigger than
    // the object". Halving both keeps the surface inside a couple of
    // centimetres of the measured rim, and the block budget grows to match the
    // smaller block footprint (16 cm at this grid instead of 24 cm).
    VoxelHash hash_{0.02f, 20000};
    TSDFIntegrator integrator_{3};
    MeshBuilder mesher_;
    // Production depth path: Tango 3DR's projective TSDF + marching cubes.
    // The members above are the fallback (or the MVS path) and stay compiled so
    // a device where the library cannot load still reconstructs something.
    TangoRecon tango_;
    bool tangoInitTried_ = false;
    bool tangoReady_ = false;
    int tangoFailStreak_ = 0;

    // Room vs street, decided automatically from the first ~20 depth frames.
    // In a street most of the top quarter is sky (no depth return), the median
    // surface is far away and a large share of samples sits past 4.5 m; a room
    // has a ceiling, nearby walls and a much shorter median. The signal only
    // changes how aggressively sparse samples are rejected: the volumetric
    // profile is fixed, because recreating the reconstruction mid-scan wiped
    // everything the user had captured when the decision flipped.
    bool outdoor_ = false;
    bool sceneDecided_ = false;
    int sceneSamples_ = 0;
    float sceneSkySum_ = 0.f;
    float sceneFarSum_ = 0.f;
    float sceneMedianSum_ = 0.f;
    float sceneLumaSum_ = 0.f;
    // Camera speed from the last accepted frame, used to reject depth maps
    // whose lag would place them more than a few centimetres away.
    float lastSpeed_ = 0.f;

    DepthResult depth_;
    // Fused grid (metres) and its uncertainty. fusedAge_ counts how many frames
    // a pixel is running on prediction alone, so a hole is held only briefly.
    std::vector<float> fusedDepth_;
    std::vector<float> fusedVar_;
    std::vector<uint8_t> fusedAge_;
    std::vector<uint8_t> lumaGrid_;
    std::vector<float> predDepth_;
    std::vector<float> predVar_;
    std::vector<uint8_t> predValid_;
    Intrinsics fusionK_;
    // Smoothed copy handed to the reconstruction (metres).
    std::vector<float> feedDepth_;
    // Millimetre grid handed to the reconstruction. The Tango core is
    // initialised with one depth resolution and does not survive a mid-session
    // switch to another one, so this is always the native 160x90 grid.
    std::vector<uint16_t> feedDepthMm_;
    Intrinsics feedK_;
    int feedW_ = 0;
    int feedH_ = 0;
    std::atomic<int> phase_{0};
    std::vector<uint8_t> motionMask_;
    int prevDepthW_ = 0;
    int prevDepthH_ = 0;
    float prevQ_[4] = {0.f, 0.f, 0.f, 1.f};
    float prevT_[3] = {0.f, 0.f, 0.f};
    Intrinsics prevKdepth_;
    std::vector<uint8_t> motionScore_;
    // What the temporal filter did last frame: seeds (no usable history),
    // blended runs, and runs where the prior was dropped after a pose jump.
    // Logged so a silently dead filter is visible in scan.txt.
    int fuseSeeds_ = 0;
    int fuseRuns_ = 0;
    int fuseWarpSkips_ = 0;
    float lastRelLength_ = 0.f;
    float lastRelAngle_ = 0.f;
    std::shared_ptr<const MeshData> dynamicSnapshot_ = std::make_shared<MeshData>();
    mutable std::mutex dynamicMutex_;
    uint32_t dynamicVersion_ = 0;
    std::atomic<int> dynamicPixels_{0};
    std::vector<VoxelBlock*> dirty_;
    // Blocks changed since the last mesh extraction. Blocks are only removed by
    // eviction, which happens right after a flush, so these pointers stay valid.
    std::vector<VoxelBlock*> pendingDirty_;
    std::unordered_set<VoxelBlock*> pendingSet_;
    Vec3 lastCamera_{0.f, 0.f, 0.f};
    int nextKeyframeId_ = 0;

    // Throttles for the expensive steps. Integration runs every frame, carving
    // about once a second, extraction a few times per second. Seconds are from
    // steady_clock.
    float lastCarveSeconds_ = 0.f;
    float lastExtractSeconds_ = 0.f;
    float lastDiagSeconds_ = 0.f;
    Vec3 lastCarvePosition_{0.f, 0.f, 0.f};
    bool hasCarvePosition_ = false;

    // Frame-to-frame continuity: a relocalisation shows up as an impossible
    // apparent speed, and the Depth API sometimes republishes an unchanged map.
    // Either one fused at a new pose smears geometry into sheets.
    Vec3 prevFrameT_{0.f, 0.f, 0.f};
    int64_t prevFrameTs_ = 0;
    bool hasPrevFrame_ = false;
    int stableFrames_ = 3;
    int64_t lastAcceptedTs_ = 0;
    bool hasAccepted_ = false;

    std::thread worker_;
    std::mutex mutex_;
    std::mutex exportMutex_;
    std::shared_ptr<const MeshData> exportSnapshot_;
    std::condition_variable cv_;
    std::condition_variable remeshCv_;
    bool running_ = true;
    bool hasPending_ = false;
    bool resetRequested_ = false;
    bool remeshRequested_ = false;
    bool remeshDone_ = false;
    FrameInput pending_;

    std::atomic<bool> scanning_{false};
    std::atomic<float> processMs_{0.f};
    std::atomic<float> fps_{0.f};
    std::atomic<int> keyframeCount_{0};
    std::atomic<int> framesProcessed_{0};
    std::atomic<int> blockCount_{0};
    std::atomic<int> depthSource_{0};
    // True while the Tango 3DR core owns the geometry, so the GL thread knows
    // whether to read its snapshot or the fallback mesher's.
    std::atomic<bool> tangoActive_{false};
    // Fraction of the native depth grid that carries a metric sample after the
    // 3x3 flyer filter. Pixels dropped at silhouettes are holes in the
    // observation, so this is the number to watch when a surface is perforated.
    mutable std::atomic<float> hrDepthValid_{0.f};
    // Share of depth samples the ARCore confidence gate dropped in the last
    // grid; negative when the device supplies no confidence image. Logged so
    // a gate that rejects almost everything is visible without the device.
    mutable std::atomic<float> lastConfidenceReject_{-1.f};

    std::mutex diagMutex_;
    std::deque<std::string> diagLines_;

    mutable std::mutex previewMutex_;
    std::vector<uint32_t> previewDepth_ =
        std::vector<uint32_t>(kPreviewW * kPreviewH, 0xFF14181C);
    std::vector<uint32_t> previewConfidence_ =
        std::vector<uint32_t>(kPreviewW * kPreviewH, 0xFF14181C);
};

}  // namespace recon
