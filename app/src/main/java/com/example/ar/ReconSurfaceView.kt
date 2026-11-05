package com.example.ar

import android.content.Context
import android.graphics.Bitmap
import android.hardware.display.DisplayManager
import android.opengl.GLSurfaceView
import android.util.AttributeSet
import android.util.Log
import android.view.Display
import android.view.Surface
import com.example.recon.ReconAnchor
import com.example.recon.ReconMetrics
import com.example.recon.RenderMode
import com.google.ar.core.Config
import com.google.ar.core.Session
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Hosts the ARCore session and the native GL renderer. The session is created
 * on the UI thread (ARCore requirement) while update/submit/render happen on
 * the GLSurfaceView thread inside [ReconGlRenderer].
 */
class ReconSurfaceView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : GLSurfaceView(context, attrs) {

    var onError: ((String) -> Unit)? = null
    var onAnchorFailure: ((String) -> Unit)? = null

    private val glRenderer = ReconGlRenderer()
    private var session: Session? = null
    private var sessionResumed = false
    private var scanning = true
    private var overlayMode = RenderMode.AR_OVERLAY

    private val depthPixels = IntArray(PREVIEW_W * PREVIEW_H)
    private val confidencePixels = IntArray(PREVIEW_W * PREVIEW_H)

    init {
        setEGLContextClientVersion(3)
        setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        preserveEGLContextOnPause = true
        setRenderer(glRenderer)
        renderMode = RENDERMODE_CONTINUOUSLY
        glRenderer.onError = { message -> post { onError?.invoke(message) } }
        glRenderer.onAnchorFailure = { message -> post { onAnchorFailure?.invoke(message) } }
    }

    fun onResumeSession() {
        super.onResume()
        glRenderer.displayRotation = currentDisplayRotation()

        val existing = session
        if (existing != null) {
            try {
                if (!sessionResumed) {
                    existing.resume()
                    sessionResumed = true
                }
                glRenderer.session = existing
                return
            } catch (e: Exception) {
                Log.w(TAG, "ARCore resume failed", e)
                closeSession()
            }
        }

        try {
            val created = Session(context.applicationContext)
            val config = Config(created).apply {
                updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                focusMode = Config.FocusMode.AUTO
                // Plane finding (RoomAR-style) gives the anchor placement a
                // surface to hit-test against instead of dead-reckoning a point
                // in front of the camera.
                planeFindingMode = Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
                // Real per-frame depth from ARCore's motion stereo when the
                // device supports it; the native engine falls back to its own
                // multi-view stereo otherwise.
                depthMode = if (created.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
                    Config.DepthMode.AUTOMATIC
                } else {
                    Config.DepthMode.DISABLED
                }
                lightEstimationMode = Config.LightEstimationMode.DISABLED
            }
            try {
                created.configure(config)
            } catch (e: Exception) {
                config.focusMode = Config.FocusMode.FIXED
                created.configure(config)
            }
            created.resume()
            session = created
            sessionResumed = true
            glRenderer.depthApiEnabled = created.isDepthModeSupported(Config.DepthMode.AUTOMATIC)
            glRenderer.session = created
        } catch (e: Exception) {
            Log.e(TAG, "ARCore session creation failed", e)
            onError?.invoke("ARCore is unavailable on this device: ${e.message}")
        }
    }

    fun onPauseSession() {
        // Stop the GL thread first so no session.update() can race the pause.
        super.onPause()
        glRenderer.session = null
        if (sessionResumed) {
            try {
                session?.pause()
            } catch (e: Exception) {
                Log.w(TAG, "ARCore pause failed", e)
            }
            sessionResumed = false
        }
    }

    fun release() {
        super.onPause()
        closeSession()
        glRenderer.release()
    }

    fun setScanning(value: Boolean) {
        scanning = value
        glRenderer.setScanning(value)
    }

    fun isScanning(): Boolean = scanning

    fun setRenderMode(mode: RenderMode) {
        overlayMode = mode
        glRenderer.setRenderMode(mode)
    }

    fun currentRenderMode(): RenderMode = overlayMode

    fun addAnchor() {
        glRenderer.requestAnchor()
    }

    fun removeAnchor(id: Long) {
        glRenderer.removeAnchor(id)
    }

    fun resetReconstruction() {
        glRenderer.resetReconstruction()
    }

    fun anchors(): List<ReconAnchor> = glRenderer.anchors()

    fun drainDiagnostics(): String = glRenderer.drainDiagnostics()

    fun depthApiSupported(): Boolean = glRenderer.depthApiEnabled

    fun isTracking(): Boolean = glRenderer.tracking

    fun snapshotMetrics(): ReconMetrics {
        val (floats, ints) = glRenderer.snapshotMetrics()
        val hasNative = glRenderer.nativeHandle != 0L

        var depthBitmap: Bitmap? = null
        var confidenceBitmap: Bitmap? = null
        if (hasNative) {
            glRenderer.fetchPreview(depthPixels, confidencePixels)
            depthBitmap = Bitmap.createBitmap(depthPixels, PREVIEW_W, PREVIEW_H, Bitmap.Config.ARGB_8888)
            confidenceBitmap = Bitmap.createBitmap(confidencePixels, PREVIEW_W, PREVIEW_H, Bitmap.Config.ARGB_8888)
        }

        return ReconMetrics(
            fps = floats[1],
            processMs = floats[0],
            triangles = ints[1],
            vertices = ints[2],
            keyframes = ints[0],
            blocks = ints[3],
            depthSource = ints[5],
            framesProcessed = ints[4],
            phase = ints[6],
            tracking = glRenderer.tracking,
            scanning = scanning,
            depthPreview = depthBitmap,
            confidencePreview = confidenceBitmap
        )
    }

    fun exportMesh(format: String): File? {
        val directory = File(context.getExternalFilesDir(null) ?: context.filesDir, "exports")
        if (!directory.exists() && !directory.mkdirs()) return null
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val file = File(directory, "scan_$stamp.${format.lowercase(Locale.US)}")
        return if (glRenderer.exportMesh(file.absolutePath, format)) file else null
    }

    fun hasGeometry(): Boolean = glRenderer.hasGeometry()

    override fun onDetachedFromWindow() {
        super.onDetachedFromWindow()
        release()
    }

    private fun closeSession() {
        val active = session ?: return
        try {
            if (sessionResumed) active.pause()
            active.close()
        } catch (e: Exception) {
            Log.w(TAG, "ARCore close failed", e)
        }
        session = null
        sessionResumed = false
        glRenderer.session = null
    }

    private fun currentDisplayRotation(): Int {
        val manager = context.getSystemService(Context.DISPLAY_SERVICE) as DisplayManager
        return when (manager.getDisplay(Display.DEFAULT_DISPLAY)?.rotation) {
            Surface.ROTATION_90 -> 1
            Surface.ROTATION_180 -> 2
            Surface.ROTATION_270 -> 3
            else -> 0
        }
    }

    companion object {
        private const val TAG = "ReconSurface"
        const val PREVIEW_W = 128
        const val PREVIEW_H = 96
    }
}
