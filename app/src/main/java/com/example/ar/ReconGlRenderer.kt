package com.example.ar

import android.media.Image
import android.opengl.GLES11Ext
import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.util.Log
import com.example.recon.NativeRecon
import com.example.recon.ReconAnchor
import com.example.recon.RenderMode
import com.google.ar.core.Anchor
import com.google.ar.core.Camera
import com.google.ar.core.Coordinates2d
import com.google.ar.core.DepthPoint
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Pose
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * GL thread side of the scanner. Pulls live ARCore frames (YUV CPU image +
 * 6DoF pose + intrinsics) into the native engine, keeps world-locked anchors,
 * and draws the camera feed together with the fused TSDF surface.
 */
class ReconGlRenderer : GLSurfaceView.Renderer {

    @Volatile var session: Session? = null
    @Volatile var nativeHandle: Long = 0L
        private set
    @Volatile var tracking: Boolean = false
        private set
    // Set when the session was configured with Config.DepthMode.AUTOMATIC.
    @Volatile var depthApiEnabled: Boolean = false

    @Volatile private var scanning = true
    @Volatile private var renderMode = RenderMode.AR_OVERLAY
    @Volatile var displayRotation: Int = 0

    // Depth under the screen centre, in metres; 0 while unknown. ARCore hit
    // tests need a tracked plane or depth point, and a fresh scan often has
    // neither, so an anchor falls back to this measured distance instead of a
    // guess.
    @Volatile private var centerDepthM: Float = 0f

    var onError: ((String) -> Unit)? = null
    // Anchor requested while the crosshair had no measured surface under it.
    var onAnchorFailure: ((String) -> Unit)? = null

    private var cameraTextureId = 0
    private var textureBound = false
    private var viewportWidth = 1
    private var viewportHeight = 1

    private val viewMatrix = FloatArray(16)
    private val projectionMatrix = FloatArray(16)
    private val backgroundUv = FloatArray(8)
    private val backgroundQuad = floatArrayOf(-1f, 1f, -1f, -1f, 1f, 1f, 1f, -1f)

    // ARCore depth maps are aligned to the camera texture, which follows the
    // display aspect (e.g. 160x90 on a 16:9 screen), while the CPU camera image
    // can be 4:3 (640x480). The depth map is therefore a vertical crop of the
    // colour image, not a full-frame image. Elements 0..5 are the affine map
    // (CPU-image normalised -> texture normalised) measured from ARCore;
    // 6..9 are the depth grid's own pinhole intrinsics, obtained the same way
    // Google's raw-depth sample does it: texture intrinsics scaled to the depth
    // resolution. Unprojecting with the CPU intrinsics is what distorted the
    // surface on earlier runs.
    private val depthInfo = FloatArray(10)
    private var depthTransformKey: Long = 0L

    private class AnchorEntry(val id: Long, val anchor: Anchor, val label: String)

    private val anchors = ArrayList<AnchorEntry>()
    @Volatile private var anchorSnapshot: List<ReconAnchor> = emptyList()
    private val anchorAddRequest = AtomicBoolean(false)
    private val anchorRemoveQueue = ConcurrentLinkedQueue<Long>()
    private var anchorSeq = 0L

    fun setScanning(value: Boolean) {
        scanning = value
        val handle = nativeHandle
        if (handle != 0L) NativeRecon.setScanning(handle, value)
    }

    fun isScanning(): Boolean = scanning

    fun setRenderMode(mode: RenderMode) {
        renderMode = mode
    }

    fun requestAnchor() {
        anchorAddRequest.set(true)
    }

    fun removeAnchor(id: Long) {
        anchorRemoveQueue.add(id)
    }

    fun resetReconstruction() {
        val handle = nativeHandle
        if (handle != 0L) NativeRecon.reset(handle)
    }

    fun anchors(): List<ReconAnchor> = anchorSnapshot

    fun snapshotMetrics(): Pair<FloatArray, IntArray> {
        val floats = FloatArray(2)
        val ints = IntArray(6)
        val handle = nativeHandle
        if (handle != 0L) NativeRecon.getStats(handle, floats, ints)
        return floats to ints
    }

    fun fetchPreview(depthOut: IntArray, confidenceOut: IntArray) {
        val handle = nativeHandle
        if (handle != 0L) NativeRecon.depthPreview(handle, depthOut, confidenceOut)
    }

    fun drainDiagnostics(): String {
        val handle = nativeHandle
        return if (handle != 0L) NativeRecon.getDiagnostics(handle) else ""
    }

    fun exportMesh(path: String, format: String): Boolean {
        val handle = nativeHandle
        return handle != 0L && NativeRecon.exportMesh(handle, path, format)
    }

    fun hasGeometry(): Boolean {
        val handle = nativeHandle
        return handle != 0L && NativeRecon.hasMesh(handle)
    }

    fun release() {
        val handle = nativeHandle
        if (handle != 0L) {
            NativeRecon.destroy(handle)
            nativeHandle = 0L
        }
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        if (nativeHandle == 0L) {
            nativeHandle = NativeRecon.create()
        }
        NativeRecon.glShutdown(nativeHandle)
        if (!NativeRecon.glInit(nativeHandle)) {
            onError?.invoke("OpenGL 3.0 initialisation failed")
        }
        NativeRecon.setScanning(nativeHandle, scanning)

        cameraTextureId = createExternalTexture()
        textureBound = false
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewportWidth = width.coerceAtLeast(1)
        viewportHeight = height.coerceAtLeast(1)
        displayRotation = displayRotationOf()
    }

    override fun onDrawFrame(gl: GL10?) {
        val handle = nativeHandle
        val activeSession = session
        if (handle == 0L || activeSession == null) {
            GLES30.glViewport(0, 0, viewportWidth, viewportHeight)
            GLES30.glClearColor(0.02f, 0.03f, 0.05f, 1f)
            GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT or GLES30.GL_DEPTH_BUFFER_BIT)
            return
        }

        if (!textureBound) {
            activeSession.setCameraTextureName(cameraTextureId)
            textureBound = true
        }

        displayRotation = displayRotationOf()
        val frame = try {
            activeSession.setDisplayGeometry(displayRotation, viewportWidth, viewportHeight)
            activeSession.update()
        } catch (e: Exception) {
            Log.w(TAG, "ARCore update failed", e)
            tracking = false
            return
        }

        val camera = frame.camera
        tracking = camera.trackingState == TrackingState.TRACKING
        if (tracking) {
            processAnchorRequests(activeSession, camera, frame)
            submitCameraFrame(frame, camera, handle)
        }
        updateAnchorSnapshot()
        draw(frame, camera, handle)
    }

    private fun submitCameraFrame(frame: Frame, camera: Camera, handle: Long) {
        val image = try {
            frame.acquireCameraImage()
        } catch (e: NotYetAvailableException) {
            return
        } catch (e: Exception) {
            // Resource exhaustion and camera hiccups also surface here; one
            // skipped frame is better than taking the process down.
            Log.w(TAG, "camera image unavailable", e)
            return
        }

        // ARCore depth is optional: acquisition throws until the Depth API has
        // warmed up, and devices without support never produce an image.
        //
        // Raw depth + confidence is the reconstruction input: the confidence
        // marks flying pixels on silhouettes and the values ARCore blended
        // across a depth step, which are the sheets that used to appear in
        // mid-air. The smoothed image is the fallback when raw depth is not
        // available on the device.
        var depthImage: Image? = null
        var confidenceImage: Image? = null
        if (depthApiEnabled) {
            depthImage = runCatching { frame.acquireRawDepthImage16Bits() }.getOrNull()
            confidenceImage = runCatching { frame.acquireRawDepthConfidenceImage() }.getOrNull()
            if (depthImage == null || confidenceImage == null) {
                runCatching { depthImage?.close() }
                runCatching { confidenceImage?.close() }
                depthImage = null
                confidenceImage = null
                depthImage = runCatching { frame.acquireDepthImage16Bits() }.getOrNull()
            }
        }

        try {
            if (image.width <= 1 || image.height <= 1) return
            val intrinsics = camera.imageIntrinsics
            val focal = intrinsics.focalLength
            val principal = intrinsics.principalPoint
            val dimensions = intrinsics.imageDimensions
            val scaleX = image.width.toFloat() / dimensions[0].toFloat()
            val scaleY = image.height.toFloat() / dimensions[1].toFloat()

            val pose = camera.pose
            val q = pose.rotationQuaternion
            val t = pose.translation

            val y = image.planes[0]
            val u = image.planes[1]
            val v = image.planes[2]
            val dPlane = depthImage?.planes?.getOrNull(0)
            val cPlane = confidenceImage?.planes?.getOrNull(0)
            if (depthImage != null) {
                updateDepthTransform(frame, image.width, image.height)
                updateDepthIntrinsics(camera, depthImage.width, depthImage.height)
                sampleCenterDepth(depthImage)
            }

            NativeRecon.submitFrame(
                handle,
                y.buffer, y.rowStride, y.pixelStride,
                u.buffer, v.buffer, u.rowStride, u.pixelStride,
                image.width, image.height, frame.timestamp,
                q[0], q[1], q[2], q[3],
                t[0], t[1], t[2],
                focal[0] * scaleX, focal[1] * scaleY,
                principal[0] * scaleX, principal[1] * scaleY,
                true,
                dPlane?.buffer, dPlane?.rowStride ?: 0,
                depthImage?.width ?: 0, depthImage?.height ?: 0,
                depthImage?.timestamp ?: 0L,
                depthApiEnabled,
                cPlane?.buffer, cPlane?.rowStride ?: 0, cPlane?.pixelStride ?: 0,
                confidenceImage?.width ?: 0, confidenceImage?.height ?: 0,
                depthInfo
            )
        } finally {
            // A stale image that ARCore already reclaimed must not throw from
            // the finally block; that exception would escape the GL thread.
            runCatching { image.close() }
            runCatching { depthImage?.close() }
            runCatching { confidenceImage?.close() }
        }
    }

    /**
     * Measures where the depth image sits inside the CPU camera image. ARCore
     * crops the depth map to the display aspect ratio (on a 4:3 camera feed a
     * 16:9 depth map is missing the top and bottom rows), so a naive resize
     * would distort every reconstructed surface. Three corner correspondences
     * define the affine map u = ax + by + c, v = dx + ey + f, which is exact for
     * ARCore's crop/scale/rotation transform.
     */
    private fun updateDepthTransform(frame: Frame, imageWidth: Int, imageHeight: Int) {
        var key = imageWidth.toLong()
        key = key * 1000003L + imageHeight
        key = key * 1000003L + viewportWidth
        key = key * 1000003L + viewportHeight
        key = key * 1000003L + displayRotation
        if (key == depthTransformKey) return

        val src = floatArrayOf(
            0f, 0f,
            imageWidth.toFloat(), 0f,
            0f, imageHeight.toFloat()
        )
        val dst = FloatArray(6)
        try {
            frame.transformCoordinates2d(
                Coordinates2d.IMAGE_PIXELS,
                src,
                Coordinates2d.TEXTURE_NORMALIZED,
                dst
            )
            depthInfo[0] = dst[2] - dst[0]   // u: x coefficient
            depthInfo[1] = dst[4] - dst[0]   // u: y coefficient
            depthInfo[2] = dst[0]            // u: offset
            depthInfo[3] = dst[3] - dst[1]   // v: x coefficient
            depthInfo[4] = dst[5] - dst[1]   // v: y coefficient
            depthInfo[5] = dst[1]            // v: offset
            depthTransformKey = key
            val text = String.format(
                java.util.Locale.US,
                "depth transform u=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f) " +
                    "image=%dx%d view=%dx%d rot=%d",
                depthInfo[0], depthInfo[1], depthInfo[2],
                depthInfo[3], depthInfo[4], depthInfo[5],
                imageWidth, imageHeight, viewportWidth, viewportHeight, displayRotation
            )
            Log.i(TAG, text)
        } catch (e: Exception) {
            // Identity means "depth covers the whole CPU image", which is true
            // on devices whose camera and display aspect ratios match.
            depthInfo[0] = 1f; depthInfo[1] = 0f; depthInfo[2] = 0f
            depthInfo[3] = 0f; depthInfo[4] = 1f; depthInfo[5] = 0f
            depthTransformKey = key
            Log.w(TAG, "depth transform unavailable, using identity", e)
        }
    }

    /**
     * Pinhole intrinsics of the depth grid. The depth image is a resampled
     * copy of the camera *texture* (not of the CPU image), so the texture
     * intrinsics scaled by depth/texture resolution describe it exactly. This
     * is the same construction as PointCloudHelper in Google's raw-depth
     * sample.
     */
    private fun updateDepthIntrinsics(camera: Camera, depthWidth: Int, depthHeight: Int) {
        val tex = camera.textureIntrinsics
        val dims = tex.imageDimensions
        if (dims[0] <= 0 || dims[1] <= 0) return
        val sx = depthWidth.toFloat() / dims[0]
        val sy = depthHeight.toFloat() / dims[1]
        val f = tex.focalLength
        val p = tex.principalPoint
        depthInfo[6] = f[0] * sx
        depthInfo[7] = f[1] * sy
        depthInfo[8] = p[0] * sx
        depthInfo[9] = p[1] * sy
    }

    /**
     * Median of a 15x15 depth patch under the screen centre. A median, not a
     * mean, so a single flyer pixel (a window edge, a cable) cannot pull the
     * anchor metres off the surface the user is pointing at.
     */
    private fun sampleCenterDepth(depthImage: Image) {
        val plane = depthImage.planes[0]
        val buffer = plane.buffer
        val rowStride = plane.rowStride
        val pixelStride = plane.pixelStride
        val half = 7
        val cx = depthImage.width / 2
        val cy = depthImage.height / 2
        val values = ArrayList<Int>(15 * 15)
        for (dy in -half..half) {
            val y = (cy + dy).coerceIn(0, depthImage.height - 1)
            for (dx in -half..half) {
                val x = (cx + dx).coerceIn(0, depthImage.width - 1)
                val offset = y * rowStride + x * pixelStride
                if (offset + 1 >= buffer.limit()) continue
                val mm = buffer.getShort(offset).toInt() and 0xFFFF
                if (mm > 200) values.add(mm)
            }
        }
        centerDepthM = if (values.isEmpty()) {
            0f
        } else {
            values.sort()
            values[values.size / 2] / 1000f
        }
    }

    private fun draw(frame: Frame, camera: Camera, handle: Long) {
        val textureReady = textureBound
        if (textureReady) {
            try {
                frame.transformCoordinates2d(
                    Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES,
                    backgroundQuad,
                    Coordinates2d.TEXTURE_NORMALIZED,
                    backgroundUv
                )
            } catch (e: Exception) {
                Log.w(TAG, "background UV transform failed", e)
            }
        }

        camera.getViewMatrix(viewMatrix, 0)
        camera.getProjectionMatrix(projectionMatrix, 0, 0.05f, 100f)

        val anchorPositions = FloatArray(anchorSnapshot.size * 3)
        anchorSnapshot.forEachIndexed { index, anchor ->
            anchorPositions[index * 3] = anchor.x
            anchorPositions[index * 3 + 1] = anchor.y
            anchorPositions[index * 3 + 2] = anchor.z
        }

        NativeRecon.render(
            handle,
            if (textureReady) cameraTextureId else 0,
            viewMatrix,
            projectionMatrix,
            backgroundUv,
            viewportWidth,
            viewportHeight,
            renderMode.nativeId,
            anchorPositions
        )
    }

    private fun processAnchorRequests(session: Session, camera: Camera, frame: Frame) {
        var changed = false

        if (anchorAddRequest.getAndSet(false)) {
            // Place the anchor on a real surface (RoomAR's approach): hit-test
            // the screen centre and prefer plane hits, because an anchor on the
            // camera pose sits inside the phone and is invisible at any depth.
            val anchor = placeSurfaceAnchor(session, camera, frame)
            if (anchor != null) {
                anchorSeq++
                anchors.add(AnchorEntry(anchorSeq, anchor, "Anchor $anchorSeq"))
                changed = true
            } else {
                onAnchorFailure?.invoke(
                    "Nişangahın altında ölçülmüş yüzey yok. Masaya ya da duvara doğrultup tekrar dene."
                )
            }
        }

        while (true) {
            val id = anchorRemoveQueue.poll() ?: break
            val iterator = anchors.iterator()
            while (iterator.hasNext()) {
                val entry = iterator.next()
                if (entry.id == id) {
                    entry.anchor.detach()
                    iterator.remove()
                    changed = true
                }
            }
        }

        if (changed) updateAnchorSnapshot()
    }

    private fun placeSurfaceAnchor(session: Session, camera: Camera, frame: Frame): Anchor? {
        val centreX = viewportWidth / 2f
        val centreY = viewportHeight / 2f
        val hits = runCatching { frame.hitTest(centreX, centreY) }.getOrDefault(emptyList())
        val planeHit = hits.firstOrNull {
            it.trackable is Plane && it.trackable.trackingState == TrackingState.TRACKING
        }
        val depthHit = hits.firstOrNull { it.trackable is DepthPoint }
        // A feature-point hit has no metric depth, and a guessed distance puts
        // the marker in mid-air inside a wall. Only measured surfaces qualify.
        val hit = planeHit ?: depthHit
        if (hit != null) {
            runCatching { hit.createAnchor() }.getOrNull()?.let { return it }
        }
        // No plane or depth point under the crosshair yet: use the sensor depth
        // measured under it, and refuse the anchor when even that is unknown.
        val measured = centerDepthM
        if (measured < 0.25f || measured > 7.5f) return null
        return runCatching {
            session.createAnchor(
                camera.pose.compose(Pose.makeTranslation(0f, 0f, -measured))
            )
        }.getOrNull()
    }

    private fun updateAnchorSnapshot() {
        var removed = false
        val iterator = anchors.iterator()
        while (iterator.hasNext()) {
            val entry = iterator.next()
            if (entry.anchor.trackingState == TrackingState.STOPPED) {
                iterator.remove()
                removed = true
            }
        }

        val snapshot = anchors.map { entry ->
            val t = entry.anchor.pose.translation
            ReconAnchor(entry.id, entry.label, t[0], t[1], t[2])
        }
        if (removed || snapshot.size != anchorSnapshot.size || snapshot != anchorSnapshot) {
            anchorSnapshot = snapshot
        }
    }

    private fun displayRotationOf(): Int = displayRotation

    companion object {
        private const val TAG = "ReconGL"

        private fun createExternalTexture(): Int {
            val ids = IntArray(1)
            GLES30.glGenTextures(1, ids, 0)
            GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, ids[0])
            GLES30.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR
            )
            GLES30.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR
            )
            GLES30.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE
            )
            GLES30.glTexParameteri(
                GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE
            )
            return ids[0]
        }
    }
}
