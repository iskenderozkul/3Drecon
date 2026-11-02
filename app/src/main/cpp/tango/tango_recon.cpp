#include "tango/tango_recon.h"

#include <android/log.h>
#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <utility>

#include "tango_3d_reconstruction_api.h"

#define LOG_TAG "TangoRecon"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace recon {
namespace {

inline double nowSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The profile is chosen by the engine (room vs street) and passed to init();
// the grid cell used for nearest-first ordering follows the active profile.

// Packs a 16^3 grid index into one key so the pending queue can deduplicate
// cells that the core reports again before they are extracted.
inline uint64_t gridKey(const std::array<int, 3>& index) {
    const uint64_t x = static_cast<uint64_t>(index[0] & 0x1FFFFF);
    const uint64_t y = static_cast<uint64_t>(index[1] & 0x1FFFFF);
    const uint64_t z = static_cast<uint64_t>(index[2] & 0x1FFFFF);
    return (x << 42) | (y << 21) | z;
}

// The library has no C++ ABI of its own (plain C entry points), and loading it
// with DT_NEEDED would make the whole app fail to start if the device cannot
// load it. All entry points are resolved through dlopen instead; a partially
// resolved table leaves TangoRecon unready and the engine falls back.
struct TangoApi {
    void* lib = nullptr;

    decltype(&Tango3DR_Config_create) configCreate = nullptr;
    decltype(&Tango3DR_Config_setBool) configSetBool = nullptr;
    decltype(&Tango3DR_Config_setInt32) configSetInt32 = nullptr;
    decltype(&Tango3DR_Config_setDouble) configSetDouble = nullptr;
    decltype(&Tango3DR_Config_destroy) configDestroy = nullptr;
    decltype(&Tango3DR_ReconstructionContext_create) contextCreate = nullptr;
    decltype(&Tango3DR_ReconstructionContext_destroy) contextDestroy = nullptr;
    decltype(&Tango3DR_ReconstructionContext_setDepthCalibration) setDepthCalibration = nullptr;
    decltype(&Tango3DR_ReconstructionContext_setColorCalibration) setColorCalibration = nullptr;
    decltype(&Tango3DR_updateFromDepthImage) updateFromDepthImage = nullptr;
    decltype(&Tango3DR_extractMeshSegment) extractMeshSegment = nullptr;
    decltype(&Tango3DR_extractFullMesh) extractFullMesh = nullptr;
    decltype(&Tango3DR_Mesh_initEmpty) meshInitEmpty = nullptr;
    decltype(&Tango3DR_Mesh_destroy) meshDestroy = nullptr;
    decltype(&Tango3DR_GridIndexArray_initEmpty) gridInitEmpty = nullptr;
    decltype(&Tango3DR_GridIndexArray_destroy) gridDestroy = nullptr;

    bool ok = false;

    void load() {
        if (lib) return;
        lib = dlopen("libtango_3d_reconstruction.so", RTLD_NOW | RTLD_LOCAL);
        if (!lib) {
            LOGE("dlopen failed: %s", dlerror());
            return;
        }
        auto sym = [this](const char* name) { return dlsym(lib, name); };
#define RESOLVE(member, type) \
    member = reinterpret_cast<decltype(member)>(sym(#type))
        RESOLVE(configCreate, Tango3DR_Config_create);
        RESOLVE(configSetBool, Tango3DR_Config_setBool);
        RESOLVE(configSetInt32, Tango3DR_Config_setInt32);
        RESOLVE(configSetDouble, Tango3DR_Config_setDouble);
        RESOLVE(configDestroy, Tango3DR_Config_destroy);
        RESOLVE(contextCreate, Tango3DR_ReconstructionContext_create);
        RESOLVE(contextDestroy, Tango3DR_ReconstructionContext_destroy);
        RESOLVE(setDepthCalibration, Tango3DR_ReconstructionContext_setDepthCalibration);
        RESOLVE(setColorCalibration, Tango3DR_ReconstructionContext_setColorCalibration);
        RESOLVE(updateFromDepthImage, Tango3DR_updateFromDepthImage);
        RESOLVE(extractMeshSegment, Tango3DR_extractMeshSegment);
        RESOLVE(extractFullMesh, Tango3DR_extractFullMesh);
        RESOLVE(meshInitEmpty, Tango3DR_Mesh_initEmpty);
        RESOLVE(meshDestroy, Tango3DR_Mesh_destroy);
        RESOLVE(gridInitEmpty, Tango3DR_GridIndexArray_initEmpty);
        RESOLVE(gridDestroy, Tango3DR_GridIndexArray_destroy);
#undef RESOLVE
        ok = configCreate && configSetBool && configSetInt32 && configSetDouble &&
             configDestroy && contextCreate && contextDestroy && setDepthCalibration &&
             setColorCalibration && updateFromDepthImage && extractMeshSegment &&
             extractFullMesh && meshInitEmpty && meshDestroy && gridInitEmpty && gridDestroy;
        if (!ok) {
            LOGE("libtango_3d_reconstruction.so is missing entry points");
        } else {
            LOGI("Tango 3DR loaded");
        }
    }
};

TangoApi& api() {
    static TangoApi instance = [] {
        TangoApi a;
        a.load();
        return a;
    }();
    return instance;
}

Tango3DR_CameraCalibration makeCalibration(const Intrinsics& K, int width, int height) {
    Tango3DR_CameraCalibration calib{};
    calib.calibration_type = TANGO_3DR_CALIBRATION_POLYNOMIAL_3_PARAMETERS;
    calib.width = static_cast<uint32_t>(width);
    calib.height = static_cast<uint32_t>(height);
    calib.fx = static_cast<double>(K.fx);
    calib.fy = static_cast<double>(K.fy);
    calib.cx = static_cast<double>(K.cx);
    calib.cy = static_cast<double>(K.cy);
    calib.distortion[0] = 0.0;
    calib.distortion[1] = 0.0;
    calib.distortion[2] = 0.0;
    calib.distortion[3] = 0.0;
    calib.distortion[4] = 0.0;
    return calib;
}

// ARCore unprojects its depth map with the camera-texture intrinsics scaled to
// the depth grid. Older bridges send no Kdepth; then the CPU intrinsics scaled
// to the same grid are the best available estimate.
Intrinsics depthIntrinsics(const FrameInput& frame) {
    Intrinsics K = frame.Kdepth;
    const bool valid = K.fx > 1.f && K.fy > 1.f && K.cx > 0.f && K.cy > 0.f &&
                       K.width == frame.depthWidth && K.height == frame.depthHeight;
    if (valid) return K;
    const float sx = static_cast<float>(frame.depthWidth) /
                     static_cast<float>(frame.width);
    const float sy = static_cast<float>(frame.depthHeight) /
                     static_cast<float>(frame.height);
    K.fx = frame.K.fx * sx;
    K.fy = frame.K.fy * sy;
    K.cx = frame.K.cx * sx;
    K.cy = frame.K.cy * sy;
    K.width = frame.depthWidth;
    K.height = frame.depthHeight;
    return K;
}

}  // namespace

TangoRecon::~TangoRecon() {
    if (!context_) return;
    TangoApi& a = api();
    if (a.ok && a.contextDestroy) a.contextDestroy(context_);
    context_ = nullptr;
}

bool TangoRecon::init(double resolution, double maxDepth) {
    resolution_ = resolution;
    maxDepth_ = maxDepth;
    if (context_) return true;
    TangoApi& a = api();
    if (!a.ok) return false;

    Tango3DR_Config config = a.configCreate(TANGO_3DR_CONFIG_RECONSTRUCTION);
    if (!config) {
        LOGE("Tango3DR_Config_create failed");
        return false;
    }
    // 3 cm voxels keep leaves, mug rims and chair edges; the grid is fed by a
    // 160x90 metric depth map whose footprint is 1-2 cm at room range. The
    // 6 m range matches what ARCore still resolves: farther samples are
    // extrapolation, and integrating them only added noise and fusion cost.
    a.configSetDouble(config, "resolution", resolution_);
    a.configSetDouble(config, "min_depth", 0.20);
    a.configSetDouble(config, "max_depth", maxDepth_);
    a.configSetBool(config, "generate_color", true);
    // Tango's own visibility clearing replaces our hand-written carve; it is
    // the same step that makes the reference scanner survive walking around a
    // room without turning surfaces into ghosts.
    a.configSetBool(config, "use_space_clearing", true);
    a.configSetBool(config, "use_parallel_integration", true);
    // Reject only tiny extraction debris, not real objects.
    a.configSetInt32(config, "min_num_vertices", 9);
    a.configSetInt32(config, "update_method", TANGO_3DR_PROJECTIVE_UPDATE);

    context_ = a.contextCreate(config);
    a.configDestroy(config);
    if (!context_) {
        LOGE("Tango3DR_ReconstructionContext_create failed");
        return false;
    }
    LOGI("Tango 3DR context ready (%.0f mm voxels, max %.1f m)",
         resolution_ * 1000.0, maxDepth_);
    return true;
}

void TangoRecon::applyProfile(double resolution, double maxDepth) {
    if (context_ &&
        std::fabs(resolution - resolution_) < 1e-6 &&
        std::fabs(maxDepth - maxDepth_) < 1e-6) {
        return;
    }
    TangoApi& a = api();
    if (context_ && a.ok) a.contextDestroy(context_);
    context_ = nullptr;
    segments_.clear();
    segmentVertices_ = 0;
    std::priority_queue<SegmentTask, std::vector<SegmentTask>, SegmentTaskGreater> empty;
    pending_.swap(empty);
    pendingSet_.clear();
    pendingCount_.store(0);
    dirty_ = false;
    lastPublishSeconds_ = 0.0;
    lastUpdateMs_.store(0.0);
    lastExtractMs_.store(0.0);
    integrated_.store(0);
    clearSnapshot();
    init(resolution, maxDepth);
}

void TangoRecon::clearSnapshot() {
    {
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        snapshot_ = std::make_shared<MeshData>();
    }
    triangles_.store(0);
    vertices_.store(0);
    segmentCount_.store(0);
}

void TangoRecon::reset() {
    TangoApi& a = api();
    if (context_ && a.ok) a.contextDestroy(context_);
    context_ = nullptr;
    segments_.clear();
    segmentVertices_ = 0;
    std::priority_queue<SegmentTask, std::vector<SegmentTask>, SegmentTaskGreater> empty;
    pending_.swap(empty);
    pendingSet_.clear();
    pendingCount_.store(0);
    dirty_ = false;
    lastPublishSeconds_ = 0.0;
    lastUpdateMs_.store(0.0);
    lastExtractMs_.store(0.0);
    integrated_.store(0);
    clearSnapshot();
    init();
}

bool TangoRecon::integrate(const FrameInput& frame, const DepthFeed* feed) {
    if (!context_) return false;
    if (!frame.hasDepth() || !frame.valid()) return false;
    int dw = frame.depthWidth;
    int dh = frame.depthHeight;
    Intrinsics depthK = depthIntrinsics(frame);
    const uint16_t* depthSource = frame.depthMm.data();
    if (feed && feed->mm && feed->width > 1 && feed->height > 1 &&
        feed->mm->size() == static_cast<size_t>(feed->width) * feed->height) {
        dw = feed->width;
        dh = feed->height;
        depthK = feed->K;
        depthK.width = dw;
        depthK.height = dh;
        depthSource = feed->mm->data();
    }
    const int w = frame.width;
    const int h = frame.height;
    if (w < 2 || h < 2 || (w & 1) != 0 || (h & 1) != 0) return false;

    TangoApi& a = api();
    if (!a.ok) return false;

    lastCamera_[0] = frame.tx;
    lastCamera_[1] = frame.ty;
    lastCamera_[2] = frame.tz;

    const double timestamp =
        static_cast<double>(frame.depthTimestampNs > 0 ? frame.depthTimestampNs
                                                       : frame.timestampNs) * 1e-9;

    // DEPTH16 image, millimetres. The engine hands over its filtered copy
    // (temporal fusion, hole filling, moving surfaces zeroed), so single-frame
    // noise never reaches the volume.
    depthImage_.resize(static_cast<size_t>(dw) * dh);
    std::memcpy(depthImage_.data(), depthSource,
                static_cast<size_t>(dw) * dh * sizeof(uint16_t));

    // NV21: Y plane followed by interleaved V,U. Tango samples vertex colours
    // from this image with the colour intrinsics, so the 4:3 CPU frame keeps
    // its own calibration instead of being force-fitted to the depth crop.
    const size_t ySize = static_cast<size_t>(w) * h;
    colorImage_.resize(ySize + ySize / 2);
    std::memcpy(colorImage_.data(), frame.y.data(), ySize);
    uint8_t* vu = colorImage_.data() + ySize;
    const int cw = w / 2;
    const int ch = h / 2;
    for (int i = 0; i < cw * ch; ++i) {
        vu[i * 2 + 0] = frame.v[static_cast<size_t>(i)];
        vu[i * 2 + 1] = frame.u[static_cast<size_t>(i)];
    }

    Tango3DR_ImageBuffer depthImage{};
    depthImage.width = static_cast<uint32_t>(dw);
    depthImage.height = static_cast<uint32_t>(dh);
    depthImage.stride = static_cast<uint32_t>(dw * 2);
    depthImage.timestamp = timestamp;
    depthImage.format = TANGO_3DR_HAL_PIXEL_FORMAT_DEPTH16;
    depthImage.data = reinterpret_cast<uint8_t*>(depthImage_.data());

    Tango3DR_ImageBuffer colorImage{};
    colorImage.width = static_cast<uint32_t>(w);
    colorImage.height = static_cast<uint32_t>(h);
    colorImage.stride = static_cast<uint32_t>(w);
    colorImage.timestamp = timestamp;
    colorImage.format = TANGO_3DR_HAL_PIXEL_FORMAT_YCrCb_420_SP;
    colorImage.data = colorImage_.data();

    // ARCore reports the camera pose in the OpenGL frame (X right, Y up,
    // -Z forward). Tango 3DR reconstructs in the camera optical frame
    // (X right, Y down, +Z forward), so the pose must be pre-rotated by 180
    // degrees about the camera X axis. Without this every depth map is fused
    // behind the camera and the volume fills with mirrored shells; this is
    // exactly the conversion 3DLiveScanner applies before Extract3DRPose.
    // q_t = q_gl * q_x180  =>  (x,y,z,w) = (qw, qz, -qy, -qx).
    Tango3DR_Pose pose{};
    pose.translation[0] = static_cast<double>(frame.tx);
    pose.translation[1] = static_cast<double>(frame.ty);
    pose.translation[2] = static_cast<double>(frame.tz);
    pose.orientation[0] = static_cast<double>(frame.qw);
    pose.orientation[1] = static_cast<double>(frame.qz);
    pose.orientation[2] = -static_cast<double>(frame.qy);
    pose.orientation[3] = -static_cast<double>(frame.qx);

    const Tango3DR_CameraCalibration depthCalib =
        makeCalibration(depthK, dw, dh);
    const Tango3DR_CameraCalibration colorCalib = makeCalibration(frame.K, w, h);
    a.setDepthCalibration(context_, &depthCalib);
    a.setColorCalibration(context_, &colorCalib);

    Tango3DR_GridIndexArray updated{};
    if (a.gridInitEmpty(&updated) != TANGO_3DR_SUCCESS) return false;

    const double start = nowSeconds();
    const Tango3DR_Status status =
        a.updateFromDepthImage(context_, &depthImage, &pose, &colorImage, &pose, &updated);
    lastUpdateMs_.store((nowSeconds() - start) * 1000.0);
    if (status != TANGO_3DR_SUCCESS) {
        static int failures = 0;
        if (failures < 3) {
            LOGE("updateFromDepthImage failed: %d (depth %dx%d, color %dx%d)", status, dw, dh,
                 w, h);
            ++failures;
        }
        a.gridDestroy(&updated);
        return false;
    }

    // A fast sweep over a large room can enqueue more segments than the frame
    // budget can ever drain; a stale backlog would then delay the surface the
    // user is actually aiming at. The cap is kept small enough that the queue
    // never grows into a multi-second lag: a dropped cell re-enters the queue
    // the next time the depth frame touches it.
    constexpr size_t kMaxPending = 768;
    if (pending_.size() > kMaxPending) {
        pending_ = std::priority_queue<SegmentTask, std::vector<SegmentTask>,
                                       SegmentTaskGreater>();
        pendingSet_.clear();
    }

    for (uint32_t i = 0; i < updated.num_indices; ++i) {
        const Tango3DR_GridIndex& index = updated.indices[i];
        const std::array<int, 3> key{index[0], index[1], index[2]};
        if (!pendingSet_.insert(gridKey(key)).second) continue;

        // Nearest-first: the user sees the surface he is aiming at grow, not a
        // segment from a corner scanned a minute ago.
        const float cell = static_cast<float>(16.0 * resolution_);
        const float cx = (static_cast<float>(index[0]) + 0.5f) * cell;
        const float cy = (static_cast<float>(index[1]) + 0.5f) * cell;
        const float cz = (static_cast<float>(index[2]) + 0.5f) * cell;
        const float dx = cx - frame.tx;
        const float dy = cy - frame.ty;
        const float dz = cz - frame.tz;
        pending_.push(SegmentTask{key, dx * dx + dy * dy + dz * dz});
    }
    a.gridDestroy(&updated);
    pendingCount_.store(static_cast<int>(pending_.size()));

    // Meshing must never stall depth fusion: spend a bounded slice of the
    // frame and let the rest of the queue wait for the next one. At 10-15 Hz
    // a 30 ms slice keeps the surface in front of the camera growing while
    // leaving the worker enough budget to accept the next depth map.
    constexpr double kExtractBudgetMs = 30.0;
    const double extractStart = nowSeconds();
    bool extracted = false;
    while (!pending_.empty()) {
        if ((nowSeconds() - extractStart) * 1000.0 >= kExtractBudgetMs) break;
        const SegmentTask task = pending_.top();
        pending_.pop();
        pendingSet_.erase(gridKey(task.index));
        extractSegment(task.index);
        extracted = true;
    }
    lastExtractMs_.store((nowSeconds() - extractStart) * 1000.0);
    pendingCount_.store(static_cast<int>(pending_.size()));

    if (extracted) dirty_ = true;
    evictSegments();
    integrated_.fetch_add(1);
    publish();
    return true;
}

void TangoRecon::extractSegment(const std::array<int, 3>& key) {
    if (!context_) return;
    TangoApi& a = api();
    if (!a.ok) return;

    const Tango3DR_GridIndex index{key[0], key[1], key[2]};
    Tango3DR_Mesh mesh{};
    if (a.meshInitEmpty(&mesh) != TANGO_3DR_SUCCESS) return;
    if (a.extractMeshSegment(context_, index, &mesh) != TANGO_3DR_SUCCESS) {
        a.meshDestroy(&mesh);
        return;
    }

    const uint32_t numVertices = mesh.num_vertices;
    const uint32_t numFaces = mesh.num_faces;

    // A reflective screen, a ceiling or a floor-to-balcony transition can make
    // the core advertise more geometry than it can actually deliver, and a
    // cell is bounded by construction. An empty or absurd mesh is dropped
    // instead of being copied (null pointers here used to be a SIGSEGV).
    constexpr uint32_t kMaxSegmentVertices = 60000;
    constexpr uint32_t kMaxSegmentFaces = 120000;
    const bool sane = mesh.vertices && mesh.faces && numVertices > 0 && numFaces > 0 &&
                      numVertices <= kMaxSegmentVertices && numFaces <= kMaxSegmentFaces;

    if (!sane) {
        // An empty cell is real (the surface moved out of it), so forget the
        // cached copy; an unusable non-empty answer keeps the last good one.
        if (numFaces == 0) {
            auto it = segments_.find(key);
            if (it != segments_.end()) {
                segmentVertices_ -= it->second.vertices.size();
                segments_.erase(it);
            }
        }
        a.meshDestroy(&mesh);
        return;
    }

    Segment segment;
    segment.vertices.reserve(numVertices);
    segment.indices.reserve(static_cast<size_t>(numFaces) * 3);
    for (uint32_t v = 0; v < numVertices; ++v) {
        MeshVertex vertex{};
        vertex.px = mesh.vertices[v][0];
        vertex.py = mesh.vertices[v][1];
        vertex.pz = mesh.vertices[v][2];
        if (mesh.normals) {
            vertex.nx = mesh.normals[v][0];
            vertex.ny = mesh.normals[v][1];
            vertex.nz = mesh.normals[v][2];
        } else {
            vertex.nx = 0.f;
            vertex.ny = 1.f;
            vertex.nz = 0.f;
        }
        if (mesh.colors) {
            vertex.r = mesh.colors[v][0];
            vertex.g = mesh.colors[v][1];
            vertex.b = mesh.colors[v][2];
            vertex.a = 255;
        } else {
            vertex.r = 200;
            vertex.g = 200;
            vertex.b = 200;
            vertex.a = 255;
        }
        segment.vertices.push_back(vertex);
    }
    for (uint32_t f = 0; f < numFaces; ++f) {
        const uint32_t i0 = mesh.faces[f][0];
        const uint32_t i1 = mesh.faces[f][1];
        const uint32_t i2 = mesh.faces[f][2];
        if (i0 >= numVertices || i1 >= numVertices || i2 >= numVertices) continue;
        segment.indices.push_back(i0);
        segment.indices.push_back(i1);
        segment.indices.push_back(i2);
    }
    if (segment.indices.empty()) {
        a.meshDestroy(&mesh);
        return;
    }
    Segment& slot = segments_[key];
    segmentVertices_ -= slot.vertices.size();
    slot = std::move(segment);
    segmentVertices_ += slot.vertices.size();
    a.meshDestroy(&mesh);
}

void TangoRecon::evictSegments() {
    // The core keeps integrating the whole walk, but the cached copy that is
    // rebuilt and uploaded for display is bounded: past the budget the cells
    // farthest from the current camera are dropped first, so the surface the
    // user is looking at always survives. A dropped cell comes back the next
    // time the depth frame touches it.
    constexpr size_t kMaxSegments = 1500;
    constexpr size_t kMaxVertices = 1000000;
    if (segments_.size() <= kMaxSegments && segmentVertices_ <= kMaxVertices) return;

    const float cell = static_cast<float>(16.0 * resolution_);
    std::vector<std::pair<float, std::array<int, 3>>> order;
    order.reserve(segments_.size());
    for (const auto& entry : segments_) {
        const float cx = (static_cast<float>(entry.first[0]) + 0.5f) * cell;
        const float cy = (static_cast<float>(entry.first[1]) + 0.5f) * cell;
        const float cz = (static_cast<float>(entry.first[2]) + 0.5f) * cell;
        const float dx = cx - lastCamera_[0];
        const float dy = cy - lastCamera_[1];
        const float dz = cz - lastCamera_[2];
        order.emplace_back(dx * dx + dy * dy + dz * dz, entry.first);
    }
    std::sort(order.begin(), order.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    const size_t targetSegments = kMaxSegments * 4 / 5;
    const size_t targetVertices = kMaxVertices * 4 / 5;
    size_t removed = 0;
    for (const auto& item : order) {
        if (segments_.size() <= targetSegments && segmentVertices_ <= targetVertices) break;
        auto it = segments_.find(item.second);
        if (it == segments_.end()) continue;
        segmentVertices_ -= it->second.vertices.size();
        segments_.erase(it);
        ++removed;
    }
    if (removed > 0) {
        LOGI("evicted %zu far cells (%zu cells, %zu verts remain)", removed,
             segments_.size(), segmentVertices_);
    }
}

bool TangoRecon::publishLocked() {
    auto mesh = std::make_shared<MeshData>();
    size_t totalVertices = 0;
    size_t totalFaces = 0;
    for (const auto& entry : segments_) {
        totalVertices += entry.second.vertices.size();
        totalFaces += entry.second.indices.size() / 3;
    }
    mesh->vertices.reserve(totalVertices);
    mesh->indices.reserve(totalFaces * 3);

    for (const auto& entry : segments_) {
        const Segment& segment = entry.second;
        const uint32_t base = static_cast<uint32_t>(mesh->vertices.size());
        mesh->vertices.insert(mesh->vertices.end(), segment.vertices.begin(),
                              segment.vertices.end());
        for (const uint32_t index : segment.indices) {
            mesh->indices.push_back(base + index);
        }
    }
    mesh->triangleCount = static_cast<uint32_t>(totalFaces);
    static uint32_t version = 1;
    mesh->version = ++version;

    {
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        snapshot_ = mesh;
    }
    triangles_.store(static_cast<int>(totalFaces));
    vertices_.store(static_cast<int>(totalVertices));
    segmentCount_.store(static_cast<int>(segments_.size()));
    dirty_ = false;
    lastPublishSeconds_ = nowSeconds();
    return true;
}

void TangoRecon::publish(bool force) {
    if (!dirty_ && !force) return;
    // A 0.15 s cadence is one publish per frame or two at 10-15 Hz: the mesh
    // grows visibly while the copies stay cheap.
    if (!force && nowSeconds() - lastPublishSeconds_ < 0.15) return;
    publishLocked();
}

std::shared_ptr<const MeshData> TangoRecon::mesh() const {
    std::lock_guard<std::mutex> lock(snapshotMutex_);
    return snapshot_;
}

bool TangoRecon::buildFullMesh(MeshData& out) const {
    out = MeshData();
    if (!context_) return false;
    TangoApi& a = api();
    if (!a.ok) return false;

    Tango3DR_Mesh mesh{};
    if (a.meshInitEmpty(&mesh) != TANGO_3DR_SUCCESS) return false;
    if (a.extractFullMesh(context_, &mesh) != TANGO_3DR_SUCCESS) {
        a.meshDestroy(&mesh);
        return false;
    }

    // Export is the one place that copies the entire volume; a huge or
    // malformed result is refused instead of doubling the peak memory.
    constexpr uint32_t kMaxFullVertices = 3000000;
    constexpr uint32_t kMaxFullFaces = 5000000;
    if (!mesh.vertices || !mesh.faces || mesh.num_vertices == 0 || mesh.num_faces == 0 ||
        mesh.num_vertices > kMaxFullVertices || mesh.num_faces > kMaxFullFaces) {
        LOGE("extractFullMesh returned an unusable mesh (%u verts, %u faces)",
             mesh.num_vertices, mesh.num_faces);
        a.meshDestroy(&mesh);
        return false;
    }

    out.vertices.reserve(mesh.num_vertices);
    for (uint32_t v = 0; v < mesh.num_vertices; ++v) {
        MeshVertex vertex{};
        vertex.px = mesh.vertices[v][0];
        vertex.py = mesh.vertices[v][1];
        vertex.pz = mesh.vertices[v][2];
        if (mesh.normals) {
            vertex.nx = mesh.normals[v][0];
            vertex.ny = mesh.normals[v][1];
            vertex.nz = mesh.normals[v][2];
        } else {
            vertex.ny = 1.f;
        }
        if (mesh.colors) {
            vertex.r = mesh.colors[v][0];
            vertex.g = mesh.colors[v][1];
            vertex.b = mesh.colors[v][2];
        } else {
            vertex.r = vertex.g = vertex.b = 200;
        }
        vertex.a = 255;
        out.vertices.push_back(vertex);
    }
    out.indices.reserve(static_cast<size_t>(mesh.num_faces) * 3);
    for (uint32_t f = 0; f < mesh.num_faces; ++f) {
        const uint32_t i0 = mesh.faces[f][0];
        const uint32_t i1 = mesh.faces[f][1];
        const uint32_t i2 = mesh.faces[f][2];
        if (i0 >= mesh.num_vertices || i1 >= mesh.num_vertices || i2 >= mesh.num_vertices) {
            continue;
        }
        out.indices.push_back(i0);
        out.indices.push_back(i1);
        out.indices.push_back(i2);
    }
    out.triangleCount = static_cast<uint32_t>(out.indices.size() / 3);
    a.meshDestroy(&mesh);
    return !out.indices.empty();
}

}  // namespace recon
