# Mobile 3D Scanner — Monocular Dense Reconstruction (Android)

> **⬇ [En güncel APK'yı indir — 3Drecon-latest.apk](https://github.com/iskenderozkul/3Drecon/raw/main/dist/3Drecon-latest.apk)**
> *(Download the latest APK. Bu bağlantı her zaman repodaki en son derlemeyi verir.)*
>
> Android 8.0+ · ARCore (Google Play Services for AR) gerekli.
>
> **Sürüm v2.9** (versionCode 11) — APK içinde ve oturum logunun başında da yazar.
> v2.9 fixes the point cloud v2.8 handed to the core. `Tango3DR_updateFromPointCloud`
> expects the points in the **depth camera's own frame** — the reference scanner
> multiplies its world points by `inverse(pose)` before the call — and applies the
> pose itself. v2.8 sent world-space points together with the pose, so every point
> was transformed twice and landed roughly twice the pose offset away from the
> camera, outside `max_depth`: the core reported zero updated cells for the whole
> session (`tris=0`, ~1 ms `upd=`), and after four seconds the engine switched to
> the legacy TSDF with `t3dr=no_geometry fallback=legacy`. The "cloud-like" mesh in
> that build was never Tango output at all. Points are now unprojected straight into
> the optical camera frame (X right, Y down, Z forward) and the pose is left to the
> core. The dense log line gained `cells=` (grid cells the update touched) so an
> empty extraction can be told apart from an update that never reached a voxel.
> v2.8 remains available as `dist/3Drecon-v2.8.apk`, v2.7 as `dist/3Drecon-v2.7.apk`,
> v2.6 as `dist/3Drecon-v2.6.apk`, v2.5 as `dist/3Drecon-v2.5.apk`, v2.4 as
> `dist/3Drecon-v2.4.apk`, v2.2 as `dist/3Drecon-best-v2.2.apk`.

Real-time dense 3D reconstruction on a phone: ARCore provides live 6DoF tracking,
camera frames and intrinsics; Google's Apache-2.0 **Tango 3D Reconstruction** core
fuses the ARCore depth stream into a TSDF and extracts a coloured mesh.
Everything is computed from the live camera — there is no simulated data anywhere
in the project. When the device supports the ARCore Depth API its metric depth is
used directly; otherwise the built-in multi-view stereo reconstructs depth. If the
prebuilt reconstruction core cannot be loaded on a device, the engine falls back to
its own TSDF pipeline rather than leaving the app without geometry.

## Requirements

- Android device with **Google Play Services for AR (ARCore)** — required in the manifest
- Android 8.0 (API 26) or newer, OpenGL ES 3.0
- ARCore Depth API support is used when present; the app also runs without it
- Android SDK 35, NDK 27.0.12077973, CMake 3.22.1 (installed automatically by Android Studio)
- JDK 17

## Build & run

```bash
./gradlew :app:assembleDebug        # APK -> app/build/outputs/apk/debug/
./gradlew :app:installDebug         # install on a connected ARCore device
```

The Gradle wrapper must run on JDK 17+. Android Studio uses its bundled JDK automatically;
from a plain shell set `JAVA_HOME` first, e.g.
`JAVA_HOME="C:/Program Files/Android/Android Studio/jbr" ./gradlew :app:assembleDebug`.

A ready-built debug APK is committed at `dist/3Drecon-latest.apk`, so it can be
downloaded directly on the phone without a local build. Every new version replaces
that file, so the link at the top of this README always points to the current build.

On first launch the app asks for camera permission, installs/updates ARCore if needed,
and starts scanning automatically. Point the phone around the room; the fused surface
appears over the camera feed. The bottom bar has `Reset`, `Anchor`, `Anchors`, the central
scan/pause button, a `Mode` cycler (`AR` → wireframe → normals → depth heat map) and
`Export`. There is no on-screen tutorial or step-by-step coach — the scanner is judged by
the surface it produces, not by an instruction card.

### Moving surfaces

People, animals and passing vehicles are separated from the room instead of being fused
into it. Every frame's filtered depth is warped into the new camera pose and compared with
the depth measured now; residual pixels must stay inconsistent for three consecutive
frames and form a connected patch of at least 96 pixels before they count as dynamic.
Those pixels are excluded from the static volume, so a walking person does not smear the
wall behind them, and a couple of noisy pixels near a silhouette no longer appear as
stray amber squares. The mask is dilated by one pixel before it reaches the static
volume, so the contaminated halo around the patch does not stay fused into the wall. The
same patch is published as a second mesh built from the live depth as one connected
shared-vertex grid: neighbouring depth samples are joined only across small depth steps,
normals come from the image-axis tangents and face the camera, and vertices are shared
between triangles, so the moving body is drawn as a surface instead of a scatter of
stretched quads. It is amber and clears as soon as the residual goes away.

### Room or street, chosen automatically

There is still no mode switch. The engine continuously votes on the scene from the depth
map: outdoors the top quarter is mostly empty (the sky returns no depth), the median
surface is metres away and a large share of samples sits past 4.5 m; a room has a ceiling
and nearby walls. Mean image brightness is the last vote, because pointing the phone down
at sunlit pavement hides both the sky and the distance. The vote is an exponential average
of every usable frame and is re-checked every 2 s after a 20-frame warm-up; entering the
street rules needs **four consecutive votes (~8 s)** and leaving them needs three (~6 s),
so a lit doorway or a long hallway does not flip the rules mid-room. v2.3 fixed the part
that never worked: the decision used to be taken once from the first 20 frames, so a scan
that started indoors stayed "indoor" after the user walked outside and none of the street
handling ever engaged.

The signal changes only the **cleaning rules** — isolated speckle removal on asphalt and
the residual tolerance — never the depth range or the volume. Depth stops at 6 m in every
scene (v2.3's 7 m street range fused ARCore's extrapolation and filled the sky with
mesh). The volumetric profile stays fixed (3 cm voxels, 6 m ceiling) because rebuilding
the volume when the decision flipped erased everything captured so far. `Reset` clears
the decision.

The display is kept awake for as long as the activity is in front (`FLAG_KEEP_SCREEN_ON`),
because a scan has no touch input and the system timeout used to turn the screen — and the
AR session — off mid-scan.

### Temporal fusion and image-guided completion

The reconstruction never sees a raw depth map. Each frame runs a per-pixel Kalman-style
blend between the measurement and the previous fused surface warped into the current pose:

- Measurement noise grows with range (`σ = 7 mm + 0.45 %·z²`), and process noise grows
  with camera motion. Standing still therefore averages several frames into one smooth
  surface, while walking leans on the new measurement so the surface does not lag.
- An innovation larger than what noise can explain (max(8 cm, 3.5 %·z), widened on a
  street) means the predicted surface was displaced. The measurement wins immediately,
  which is also the signal that feeds the moving-object mask.
- Pixels the sensor drops are held for up to four frames, and only while the pose barely
  changed, so a thin hole in a depth map does not become a hole in the mesh and a pan does
  not paint the old view into the new one. The held value stays **inside the filter**:
  pixels running on prediction alone are zeroed in the grid handed to the reconstruction,
  because a prediction warped from the previous pose is not a measurement and used to
  become the thin shell a few centimetres above a table.
- The reconstruction never sees unchecked depth: raw depth is used when available and a
  pixel is rejected when ARCore's confidence image is below 128, which drops the
  low-confidence flying pixels that ARCore produces while it smooths across a depth step.
  The gate is strict and all-or-nothing: if fewer than 4 % of the frame survives threshold
  128, that frame is fused ungated (the median pass still removes single-pixel flyers) and
  the next frame starts again at 128. v2.3 relaxed the threshold in steps (128 → 96 → 64)
  to keep more pixels, but on this device that let ARCore's interpolated ramps through and
  turned flat surfaces lumpy, so v2.4 restored the strict gate. The dense log line shows
  the rejection share as `gate=` and the threshold actually used as `cthr=`.
- **Smoothed depth as the measurement, raw as the authority.** ARCore also returns a
  filtered depth map for the same grid and frame. A raw sample is replaced by the
  smoothed value when they agree within three voxels (9 cm): on a wall, a tabletop or
  the floor they do, so the TSDF receives the filtered surface instead of per-pixel
  noise, which is what marching cubes was turning into a cloudy, cauliflower surface.
  On a silhouette the smoothed map bridges the step and the two disagree by more, so
  the raw value survives there. Where raw has no sample the pixel stays a hole unless
  the surrounding raw depths form a continuous surface and the smoothed value fits
  inside it, so the smoothed map cannot invent a surface across a doorway or a rim.
  This is the `maxErrorFilter = 3·resolution` step of the reference scanner, and the
  dense log line reports it as `smooth=`. If a device returns no second map, the
  engine silently keeps the raw-only path.
- Cleaning tolerances scale with distance and are capped (median 2.5–10 cm, single-pixel
  fill 2–6 cm), so a ~6 cm object on a table is no longer bridged into a membrane by an
  absolute centimetre threshold meant for walls.
- A pose step too large for the warp to be meaningful (relocalisation, a dropped frame)
  drops the prior for that one frame instead of reseeding the whole grid: the measurement
  is used directly, but every pixel keeps its accumulated confidence. Reseeding on a pose
  jump was what silently disabled the filter during normal scanning. The dense log line
  carries cumulative `seed=`, `run=` and `wskip=` counters plus the last `rel=`/`ang=`, so
  a dead filter is visible in the session log rather than only in the mesh.
- Remaining holes are filled by **one** joint-bilateral pass guided by CPU luma resampled
  onto the depth grid: a pixel is closed only from neighbours with a similar colour
  (`Δluma ≤ 8`) and a consistent depth (`≤ 3 cm`, typically `1.2 cm + 1.5 %·z`), so a
  doorway, a rim or a chair silhouette is not bridged. It used to run twice with loose
  tolerances; on a textured wall that grew a membrane across a hole, which then appeared
  in the volume as a surface in air.
- A local **inverse-depth plane fit** runs on measured pixels only: a 3D plane projects
  to a linear function of image coordinates in `1/z`, so a 5×5 least-squares fit removes
  the per-pixel ripple that marching cubes turns into waves on a wall, ceiling, tabletop
  or chair seat. The plane is estimated from the whole fused grid (held predictions sit
  on the same surface) and written only to pixels that were actually measured. It
  replaces the centre pixel only when the samples agree with the plane within
  max(15 mm, 0.7 %·z) RMS (max error 2.5× that), at least 10 of the 25 samples are
  valid, and the correction is at most max(15 mm, 1 %·z); an edge, a rim or a small
  object departs from the plane and keeps its measured depth. It runs on the feed copy,
  never on the filter state. v2.5 shipped this with a unit mismatch (metres against
  inverse metres) in the residual test, which rejected every patch; v2.6 fixes it.
- **One integration per new depth map.** ARCore returns the same depth image for two or
  three consecutive render frames (its timestamp advances in ~33 ms steps at 60 fps).
  Re-fusing that grid at each newer pose stamps one measurement several times at slightly
  different places, which is what makes a surface thick and lumpy; the reference scanner
  only updates when the depth timestamp changes, and the engine now does the same. The
  temporal filter still keeps its state, so nothing is lost when a duplicate is skipped.
- One light edge-preserving smooth runs on the copy handed to the reconstruction. It does
  not touch the filter state, so repeated smoothing cannot slowly melt detail away. Its
  tolerance follows range on a fixed curve (`8 mm + 0.8 %·z`). v2.3 widened it with the
  sensor noise term (`0.8 × (7 mm + 0.45 %·z²)`), which at 4–6 m blurred across real depth
  steps and made flat walls lumpy; v2.4 restored the fixed curve.

This is the part that removes the lumpy, "melted" ground seen on grazing pavement: the
sensor's per-frame noise is averaged in time before marching cubes ever triangulates it.

### What the hardware can resolve

The Depth API grid is 160×90 and the reconstruction voxel is 3 cm. A small object lying
on a table (a mouse is about 6 cm tall) therefore spans roughly two voxels: its top
surface is measured, but the sensor rarely returns enough angled samples to grow distinct
side walls between the top and the table. A 1 cm volume would multiply the work by about
27× and cannot run in real time on this class of phone, so the engine does not promise
mouse-sized side walls. What it does fix is what produced wrong geometry anyway — phantom
sheets in mid-air (confidence gating and range-capped cleaning), anchors in the wrong
place (measured surface or no anchor), and the moving surface being drawn as scattered
quads (connected grid with shared vertices).

### Session log and safe mode

There is no test button. Every launch writes a continuous log to
`Android/data/com.example.mobile3drecon/files/logs/scan_<timestamp>.txt`: app version,
startup stages (`activity_created`, camera permission, ARCore install), then metrics and
engine diagnostics twice per second while the scanner runs. The newest 12 logs are kept.
`LOG` in the top bar shares the current session log at any time — a normal exit is no
longer required to obtain it. Each half-second line carries `frames=` and `phase=`
(1 prepare, 2 fuse, 3 feed, 4 integrate, 5 mesh, 6 remesh); when frames stop advancing
while tracking is on, a `stall frames=… phase=…` line is written so a freeze names the
native stage that never finished. The dense line also splits the frame cost into
`prep=`, `fuse=`, `feed=`, `dynms=`, `upd=`, `pts=`, `cells=` and `ex=` (milliseconds,
points, affected grid cells), so the next optimisation can be aimed at whichever stage
actually dominates instead of guessing. `cells=0` with a healthy `pts=` means the
update never reached a voxel.
`hold=` is the share of pixels running on a held prediction (excluded from the feed),
`fill=` the share synthesised by the colour-guided fill, `plane=` the share snapped
to the local plane fit, `smooth=` the share taken from ARCore's smoothed depth map,
and `dup=` the cumulative number of frames skipped because ARCore returned a depth
map it had already delivered.

The log is closed with `session_end=clean` in `onStop`, i.e. when the app leaves the
foreground, and the same file is reopened with `session_resume=` when it returns. Swiping
the app away from recents therefore leaves a finished log rather than an apparently
crashed one.

If the process really dies — native crash, OOM kill, hang — that line is missing. On the
next launch, if the newest log is less than 24 hours old, the app opens in **safe mode**
before touching ARCore: it shows that the previous session ended unexpectedly and offers
its log as a text file through the system share sheet (WhatsApp accepts it), or continues
to the scanner. Uncaught Java exceptions are additionally written to
`crash/java_crash_<stamp>.txt` and are appended to the shared log when they are newer than
the session. The 24-hour rule exists because an old build's leftover log without a marker
used to force safe mode after an update.

An earlier build installed a native SIGSEGV handler to capture native crashes. It
replaced ART's own fault handler, so the app crashed immediately at startup, and it was
removed. The session log replaces it: after a native crash it still shows the last stage
reached, the last fused frames and the engine counters, which is what makes the crash
diagnosable from the shared file alone.

## Pipeline

| Stage | Where | What actually happens |
|---|---|---|
| Tracking & frames | `ar/ReconGlRenderer.kt`, `core/frame.h` | ARCore `Session` at `LATEST_CAMERA_IMAGE`; every tracked frame delivers the YUV_420_888 CPU image (de-strided in JNI), the camera pose quaternion/translation and pinhole intrinsics (scaled to the image size). When the device supports the ARCore Depth API (`Config.DepthMode.AUTOMATIC`), the matching depth and confidence images are delivered too (raw depth preferred, smooth depth as a fallback). Consecutive frames are checked for pose continuity: an apparent speed above 2.5 m/s means ARCore relocalised, so that frame and the next three are not fused. Anchors are placed on real geometry: the screen centre is hit-tested with `Frame.hitTest`, a tracked plane hit is preferred and a depth point second; an arbitrary feature-point hit is rejected because it carries no metric depth. If neither exists, the anchor uses the depth measured under the crosshair (median of a 15×15 patch, 0.25–7.5 m). If that is unknown too, no anchor is created and a short notice asks the user to aim at a surface — the old fixed 1.2 m guess is what put markers in mid-air. Plane finding runs in `HORIZONTAL_AND_VERTICAL` mode, and markers are drawn with depth testing disabled so the ring is visible on top of the surface it is attached to. |
| Reconstruction core (primary) | `tango/tango_recon.cpp`, `tango/tango_recon.h` | Google Tango 3D Reconstruction (Apache 2.0, the same core used by 3DLiveScanner) loaded through `dlopen`. Every held frame is fused with `Tango3DR_updateFromPointCloud`: the filtered 160×90 depth grid is unprojected to **camera-space** XYZC points with its own intrinsics (X right, Y down, Z forward — the frame the core expects, matching the reference's `inverse(pose) * worldPoint`), and the ARCore camera pose is converted from the OpenGL frame (Y up, −Z forward) to Tango's optical camera frame by a 180° rotation about the camera X axis — the same conversion 3DLiveScanner applies. Without it every map fuses mirrored behind the camera. The core applies the pose to the points itself; pre-transforming them into world coordinates (v2.8) double-transformed them and no voxel was ever updated. Depth-grid intrinsics are derived from ARCore's texture intrinsics. The colour image is the full 640×480 YUV frame converted to NV21 and paired with its own pinhole intrinsics, so vertex colours are sampled with the correct calibration. Config: **3 cm voxels, 6 m range ceiling**, traversal update (`TANGO_3DR_TRAVERSAL_UPDATE`), space clearing, parallel integration, debris components under 9 vertices removed. One frame is integrated per **new depth timestamp**, never the same grid twice. It is fed the temporally fused grid with measured pixels only (held predictions zeroed, conservative hole fill, plane-snapped planar regions) at the native 160×90; the engine itself stops at 6 m in every scene, because past that ARCore extrapolates and the extra volume only stores noise. An earlier build halved the feed to 80×45 when update time climbed above 70 ms; the Tango core is initialised with one depth resolution and the process died on the first frame after that switch — every session log stopped immediately after `feed=half_depth`. The halving path is gone, so fusion costs 60–120 ms per frame but the scan stays up. Tango does its own visibility clearing and marching cubes; we only cache the 16³ grid segments it reports as changed and publish the concatenated mesh as an immutable snapshot. Extract order and cadence are what make the surface appear quickly: every segment the update reports is pushed into a nearest-first priority queue (deduplicated by grid key) and the same key is appended to an insertion-order queue; each frame spends at most a fixed 30 ms slice extracting it, and every fourth slot is served **oldest-first** instead of nearest-first. Nearest-first alone starved everything behind the camera, which is why parts of a room could stay unmeshed no matter how long the scan ran. The queue cap is **1536** and overflow discards only the farthest quarter, so the cells around the user and the recent backlog survive (the previous policy emptied the whole queue). v2.3 stretched the slice to 44 ms when the backlog was large; v2.4 keeps it fixed at 30 ms because the extra time came out of depth fusion and dropped frames. Remaining segments publish every 0.2 s. Extracted cells are validated before they are copied: a null buffer, an implausible vertex/face count or an out-of-range face index is rejected instead of dereferenced, and the cached display copy is capped at **1500 cells / 1 000 000 vertices**, dropping the cells farthest from the current camera first, so a long outdoor walk cannot grow the published mesh without bound (the core volume keeps its state). The exporter refuses a full-volume extraction above 3 000 000 vertices / 5 000 000 faces rather than doubling peak memory. If the library cannot be loaded, the engine logs `t3dr=unavailable fallback=legacy` and uses the self-written TSDF path below. |
| Keyframes | `core/keyframe_manager.cpp` | **MVS path only** — the ARCore-depth path integrates every frame and does not gate on keyframes. Sliding reference window of 14 keyframes plus a lifetime counter. A new keyframe is taken at most every 0.25 s and only once the baseline exceeds 15 cm or the view rotates by 8°; near-duplicate viewpoints cost 50–300 ms of fusion each without adding information. Reference selection follows Mobile3DRecon Eqs. (1)–(3): `wb = exp(−(b−0.6)²/0.2²)` around a 0.6 m baseline and `wv = max(10°/α, 1)`; the top two references are kept and weighted `S/ΣS` in the cost (Eq. 7). References beyond 75° are rejected as uninformative. |
| Depth input | `mvs/census.cpp`, `mvs/stereo.cpp`, `core/engine.cpp` | **Primary: ARCore Depth API.** On the Tango path the *cleaned* copy is fused: the same map that drives previews and statistics is handed to the reconstruction core, so median-rejected flyers and moving pixels never enter the static volume. `prepareDepthGrid` converts millimetres to metres (up to 6 m), runs a 3×3 median, rejects a centre sample that departs from its neighbourhood by more than max(4 cm, 5 % of depth) as a flying pixel on a silhouette, and fills a single-pixel dropout only when at least six neighbours agree within 8 cm. A frame with under 4 % valid depth is skipped — indoors the map is usually dense, and the old 12.5 % gate delayed meshing until the user walked back inside. Samples past 6 m are dropped for every scene (ARCore depth is unreliable at that range and the reconstruction stops there anyway), and when the scene is classified as a street isolated pixels without two valid neighbours are also removed, because sunlight and glossy asphalt return single-pixel speckles that the 3×3 median cannot see. A depth image that trails the camera by more than 400 ms — or by more than 20 cm at the current walking speed — is not fused against the current pose. The depth map is aligned to the camera texture, not the 4:3 CPU image, so unprojection uses ARCore's texture intrinsics scaled to the depth resolution and colour is sampled through the measured image-region affine — exactly what Google's raw-depth sample does. **Fallback when the device has no depth support:** half-resolution (320×240) 64-bit Census (8×8 minus centre, NEON `VCNT` Hamming), weighted multi-view cost over 96 inverse-depth planes in [0.3 m, 3.0 m], epipolar projection `K1 R1ᵀ R0 K0⁻¹`, 8-path SGM with gradient-adaptive `P2 = 40 + 0.45·|∇I|`, parabolic sub-pixel refinement, then a continuous ±1-plane photometric search with bilinear luma sampling. The match is kept only if the reference keyframe's own depth agrees (forward-backward check); confidence combines the photometric cost-valley sharpness with the 5×5 sub-level label disagreement `ω` (Eqs. 13–14) and drops `M < 0.15`. Results under 5 % valid pixels are discarded. |
| Refinement | `mvs/refine.cpp` | 3×3 median outlier removal + 5×5 joint bilateral filter guided by luminance, so planes flatten while silhouette edges stay sharp (MVS path only; ARCore depth is already filtered). |
| Fusion (fallback) | `tsdf/voxel_hash.cpp`, `tsdf/integrator.cpp` | Used only when the Tango core is unavailable. Scalable voxel hashing (8³ blocks, 2 cm voxels, ±τ = 5 cm, up to 20 000 live blocks), deterministic conflict-free probing over 21-bit-per-axis keys, every depth pixel sampled, and every tracked frame integrated densely — not one keyframe per second — with weight 1 (paper Eq. 21) and free space saturated at +Q, world→camera projection `R_cvᵀ·(p−t)`, parallel integration with per-2³-cube staging that rolls back any cube touching the depth-map border, an invalid pixel or an occluded surface (paper §5.2). Once per second (or after 35 cm of camera motion) every allocated block is projected into the current keyframe for the paper's visibility check (§5.1.2): voxels that the measurement sees through are written to +Q — the same value a ray passing through them would have written, i.e. measured free space — only inside the ±0.85 truncation band and with a 3×3 discontinuity guard, so a ghost sheet deflates back onto the real surface behind it and a removed object disappears. Zeroing the weight instead punched holes (marching cubes skipped those cubes on every conflicting frame), and writing −Q turned the voxel into the inside of a new phantom surface, so +Q is the only sign that keeps the field continuous. Blocks carry ADD/UPDATE/NORMAL/DELETE state. Marching Cubes and the live mesh publish run at most every 300 ms: dirty blocks are queued in `pendingDirty_` as frames arrive, so carving and meshing never stall the worker on every frame while the TSDF keeps improving. |
| Meshing (fallback) | `mesh/mesher.cpp` | Used only when the Tango core is unavailable. Incremental Marching Cubes over only ADD/UPDATE blocks. Before triangulation each block builds a `[1,2,1]³`-blended copy of the measured TSDF: the fused field is lumpy at the 2–3 voxel scale, and the renderer is lit by a headlight, so raw voxel noise shows up as speckle. A voxel the integrator never wrote (one invalid depth pixel skips its whole 2³ cube) inherits the mean of 18+ measured neighbours, which closes single-cube pinholes without filling the half-space beside a silhouette (that lower threshold used to grow a skirt around every object); the sign, and therefore any new surface, still comes only from real observations. Then per-block edge deduplication, normals interpolated from the central-difference gradient of the smoothed field at the two edge corners (so a shared edge shades smoothly instead of showing grid facets), a two-observation rule (every edge corner must carry weight ≥ 2) that rejects single-frame shards, and cached per-block geometry merged into an immutable snapshot with a version counter. The snapshot publishes every welded triangle: an earlier version kept only the component containing the seed block and discarded anything else, which deleted whole scanned objects (a chair, a plant) as "second surfaces"; that filter is gone, both live and at export. Cost is a few milliseconds per update at 60 k triangles. The merged mesh is then welded on a 1 mm grid and simplified adaptively: an edge collapses only when both endpoint normals agree to 5.7°, every incident face normal agrees, the merged point stays within 2.5 mm of the surrounding planes, and the two endpoint colours match, with facets capped at 9 cm. Flat walls and floors therefore grow into large facets while a leaf, a mug rim or a corner — where normals diverge and colour changes — keeps the 2 cm marching-cubes triangles, and rim vertices never move so silhouettes stay exact. A block whose surface is one closed, flat, convex patch and whose boundary loop spans at most 3 voxels is re-triangulated as a fan over that loop (boundary vertices untouched, so seams stay watertight); the span cap matters because a fan over a whole 24 cm block is one facet whose vertex colours interpolate across the entire panel, which is what made flat surfaces render as smeared grey slabs, so wide walls keep their marching-cubes tessellation and colour detail. Blocks re-extract whenever they change, and every change also invalidates all 26 neighbouring blocks because the mesher reads two voxels into them — without that the diagonal neighbours kept their old triangles and the surface stepped at block corners whenever carving moved it. Revisiting an area from a new pose therefore updates the live mesh instead of leaving stale sheets. At export `rebuildAll` re-triangulates every block from the current field so the final model is one consistent state, not an incremental edit history. |
| Rendering | `gl/renderer.cpp` | GLES 3.0: camera background (`samplerExternalOES` + ARCore UV transform), persistent mesh VBO/EBO, a second VBO for the moving-surface quads, wireframe/normal/depth-heatmap modes, world-locked anchor markers. In AR mode the surface is a **depth-tested, 50 % translucent overlay**: depth testing keeps a single nearest shell on screen (so overlapping sheets cannot turn into a mottled haze) and alpha blending lets the real camera image show through, instead of covering every object like an opaque blanket. Wireframe, normals and depth modes stay opaque. The moving-surface mesh is drawn in amber; both meshes only re-upload when their version counter changes. |
| Export | `io/exporter.cpp` | Wavefront OBJ with vertex colours and normals, or Stanford PLY (ASCII, coloured). Before writing, vertices are welded on a 1 mm grid (the mesher keeps a copy per 8³ block, so an unwelded export carried 1.0M vertices for 279k positions), normals are averaged per welded vertex, zero-area triangles are dropped and components under 8 triangles — silhouettes only one viewpoint ever saw — are removed. Disconnected pieces are then judged on real triangle connectivity: anything under 12 triangles and 4 cm across is debris and goes, so a chair, a plant or a separate wall chunk survives instead of being deleted as a "second surface". A final edge-collapse pass shrinks triangles only where the surface is locally flat, which makes the resolution adaptive: flat panels merge up to 8.5 cm facets while curvature and colour changes stop the merge (2.5 mm plane tolerance, 5.7° normal agreement, colour guard, link condition and no-fold checks); open-boundary vertices never move, so silhouettes stay exact. The caps sit just above one grid cell because an earlier 4.2 cm cap left a 3.5 cm median edge on a 3 cm grid and turned flat panels into 10 cm facets. Finally, boundary rims of at most 32 vertices and 12 cm are closed with a fan to their centroid — a real scan is sprinkled with one-triangle dropouts and decimation slits, and two thousand three-edge rims read as a torn mesh even when every neighbourhood is fine; of the measured rims 76 % were under 6 cm while everything past 10 cm was a real silhouette (rim length jumps by 20 m per extra 5 cm of threshold), so those stay open. Files land in `Android/data/com.example.mobile3drecon/files/exports/` and are shared via FileProvider. |
| Moving objects | `core/engine.cpp`, `gl/renderer.cpp` | Every cleaned depth frame is warped into the current camera with the relative ARCore pose (previous pixel → world → current camera) and compared with the depth measured now. A residual above max(8 cm, 3.5 %·z) — widened by 35 % on a street — scores the pixel; it must stay inconsistent for **three consecutive frames** and belong to a connected patch of **at least 96 pixels** before it counts as dynamic. Those pixels are excluded from the feed the static volume sees, so a person or vehicle cannot smear the room, while the static background behind them stays untouched because an invalid pixel writes nothing. The connected-component rule is what removes the scattered amber squares that single noisy pixels around ground silhouettes used to produce. The same pixels become a second mesh of camera-facing quads (capped at 4096, amber) rebuilt each frame and drawn with the static surface; the depth behind it is real, so the layer sits at the correct distance without a second TSDF volume. |
| Session log | `recon/ScanLog.kt`, `MainActivity.kt` | Started in `onCreate` with the app version and startup stages, suspended with `session_end=clean` in `onStop`, reopened on `onStart`, stopped in `onDestroy`. It records the engine's per-frame line plus diagnostics every 500 ms, and `LOG` in the HUD shares the live file immediately. If the newest log on the next launch is under 24 hours old and has no clean-end marker, the activity starts in safe mode and offers that file for sharing instead of starting ARCore. |

The UI polls the native engine twice per second for real statistics (FPS, worker
processing time, triangle/vertex/keyframe/block counts) and the actual depth and
confidence preview maps. The HUD also shows which depth source produced the last
frame (`ARCore depth` / `MVS depth`) and a `REC` badge while scanning is active.

## Toward Tesla-style perception

Tesla FSD is not one clever filter; it is a stack of learned perception, temporal tracking
and planning. The parts that matter for a scanner, and what is achievable on one phone:

| Layer | What the car does | Phone-sized equivalent |
|---|---|---|
| Sensors | 8 cameras + radar/IMU, tight time sync | ARCore poses + IMU + one depth camera. A **stereo pair with hardware sync** (or the phone's second rear camera once exposed) is the single biggest hardware upgrade: a 10–12 cm baseline gives metric depth at street range where the Depth API gives up. |
| Dense geometry | Surround occupancy network from all cameras | Already here: depth + traversal TSDF. The temporal Kalman filter is the same idea as Tesla's temporal occupancy fusion — average consistent observations, trust the measurement when it disagrees with the prediction. |
| Semantic completion | Networks infer what the geometry cannot see | A small on-device segmentation model (TFLite/MediaPipe) can label ground, sky, car, person, pole, then use those labels to **gate** depth: skip sky, extrapolate the ground plane, and fill occlusion holes from the labelled region instead of a colour-blind blur. That is the honest version of "complete what the sensor missed". |
| Tracking | Kalman/UKF tracks over time, object identity | A constant-velocity Kalman filter on the 3D centroid and bounding box of each dynamic component turns the current amber quads into stable tracked boxes with predicted motion. This is the next step after the depth Kalman filter that is already in the engine. |
| Planning | Occupancy + trajectories | Not applicable to a scanner; the equivalent is **next-best-view**: score unobserved voxels and tell the user where to point next. Optional, and it is guidance, which this build intentionally does not show. |

The order that pays off: (1) temporal depth fusion — done, (2) tracked moving objects
with a Kalman filter on boxes, (3) learned depth completion/segmentation, (4) stereo
hardware. Steps 2 and 3 run fine on the current phone; step 4 needs a second synchronized
camera and its own calibration.

## Honest limitations

- **Depth source depends on the device.** ARCore's Depth API is used when the phone
  supports it; otherwise the built-in multi-view stereo runs. On the MVS path, room-scale
  accuracy is a few centimetres near 1–2 m and degrades with distance (a 10 cm baseline
  and 320×240 matching cannot beat that); the ARCore depth path is markedly cleaner.
- **The reconstruction core is a prebuilt Apache-2.0 library.** `libtango_3d_reconstruction.so`
  is vendored for `arm64-v8a` and `armeabi-v7a` only (that is why the APK restricts its ABIs),
  and is loaded at runtime with `dlopen` so a load failure degrades to the fallback path
  instead of preventing launch. Its live mesh resolution is a uniform 3 cm grid; curved
  areas get more triangles automatically from marching cubes, and flat panels are merged
  into larger facets only when a mesh is exported.
- **Depth refinement is classical, not learned.** The reference design uses a TFLite
  U-Net; no trained network is bundled here because shipping one without real training
  data would mean fabricating weights. The joint-bilateral refiner is the honest
  deterministic substitute on the MVS path.
- **Session-scoped data.** The TSDF volume and anchors live in memory only; a new launch
  starts a fresh scan. Exported meshes are in the ARCore world frame of that session.
- **ARCore required.** There is no fallback to a fabricated pose/depth source, by design.
- **Street depth is the weak link.** In direct sunlight ARCore's depth map can be sparse
  and noisy; the app trims far samples, drops speckles and temporally fuses what remains, but it
  cannot make the sensor return surfaces it did not measure. Scanning the street at 1–4 m
  (shops, parked cars, kerbs) gives a much cleaner result than pointing down a long road;
  the `Depth` mode shows exactly what the sensor sees.
- **Moving objects are separated, not reconstructed.** Depth-residual detection needs the
  object to change depth at a pixel; a body that moves almost exactly parallel to the
  sensor and fills the frame can still leave a thin shell in the static volume until the
  next pose clears it. The amber layer shows the live silhouette, not a tracked 3D model,
  and it cannot name *what* moved.
- **Reflective and sub-range surfaces stay invisible.** A TV/mirror reflects the depth
  pattern instead of returning it, and macro subjects closer than the depth sensor's
  minimum (≈0.2–0.3 m) fall outside its range. The app cannot invent that geometry; the
  depth heat map (`Depth` mode) is the honest view of what the sensor saw.
- **Simplification only runs at export.** The live mesh is the 3 cm reconstruction; the
  adaptive edge-collapse pass (8.5 cm facets, 2.5 mm plane tolerance) runs when a mesh is
  written, so scan speed never depends on export detail.
- **Coverage still matters.** Without an on-screen coach the user must vary viewpoints
  themselves; surfaces seen from one angle keep their single-view noise, and thin
  structures and reflective surfaces stay rough.
- **Depth resolution is whatever the phone provides.** On the test device ARCore hands out
  a 160×90 depth image (the camera image is 640×480). The map is fused at that native
  resolution and never upscaled, so fine detail below a few depth pixels cannot be
  recovered — but no surface is interpolated between depth samples either.
- On the ARCore-depth path every tracked frame is integrated on a worker thread, with
  free-space carving and mesh extraction throttled; on the MVS fallback, dense stereo
  runs per keyframe. On slow devices the effective reconstruction rate drops while
  tracking and rendering stay at display rate.
