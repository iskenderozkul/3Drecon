package com.example.recon

import android.app.Activity
import android.content.ClipData
import android.content.Context
import android.content.Intent
import android.graphics.Bitmap
import android.graphics.Canvas
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.view.PixelCopy
import android.view.SurfaceView
import androidx.core.content.FileProvider
import java.io.File
import java.io.FileOutputStream
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlin.coroutines.resume

/**
 * One test run: everything a single tap produces is collected into one zip so
 * the log, the exported mesh and the screenshots are never separate files the
 * user has to find and forward one by one.
 */
class TestSession(private val context: Context, private val log: ScanLog) {

    var startedAtMillis: Long = 0L
        private set

    private var lastMetrics: ReconMetrics? = null
    private var lastRenderMode: String = "AR"
    private var depthThumb: Bitmap? = null
    private var confidenceThumb: Bitmap? = null

    val running: Boolean get() = startedAtMillis != 0L

    /** Starts a fresh run: the caller resets the reconstruction separately. */
    fun start(): Boolean {
        if (log.start() == null) return false
        startedAtMillis = System.currentTimeMillis()
        lastMetrics = null
        depthThumb = null
        confidenceThumb = null
        return true
    }

    fun header(key: String, value: Any?) = log.header(key, value)

    fun observe(metrics: ReconMetrics, renderMode: String) {
        lastMetrics = metrics
        lastRenderMode = renderMode
        metrics.depthPreview?.let { depthThumb = it }
        metrics.confidencePreview?.let { confidenceThumb = it }
    }

    fun elapsedSeconds(): Long =
        if (startedAtMillis == 0L) 0L
        else ((System.currentTimeMillis() - startedAtMillis) / 1000L).coerceAtLeast(0L)

    /**
     * Zips the run into a single file. Intermediate files are only deleted once
     * the archive has been written successfully, so a failure never loses the log.
     */
    fun pack(
        stamp: String,
        durationSeconds: Long,
        meshFile: File?,
        screen: Bitmap?,
        notes: List<String>
    ): File? {
        val logFile = log.currentFile()
        val crashFiles = crashReports()
        val outDir = File(context.getExternalFilesDir(null) ?: context.filesDir, "exports")
        if (!outDir.exists() && !outDir.mkdirs()) return null
        val zip = File(outDir, "test_$stamp.zip")

        val written = try {
            ZipOutputStream(FileOutputStream(zip)).use { zos ->
                fun add(name: String, bytes: ByteArray) {
                    zos.putNextEntry(ZipEntry(name))
                    zos.write(bytes)
                    zos.closeEntry()
                }
                // Streamed: the mesh can be tens of megabytes.
                fun addFile(name: String, file: File) {
                    zos.putNextEntry(ZipEntry(name))
                    file.inputStream().use { it.copyTo(zos) }
                    zos.closeEntry()
                }
                fun addPng(name: String, bitmap: Bitmap, scale: Int = 1) {
                    val target =
                        if (scale > 1) Bitmap.createScaledBitmap(
                            bitmap,
                            bitmap.width * scale,
                            bitmap.height * scale,
                            false
                        ) else bitmap
                    val buffer = java.io.ByteArrayOutputStream()
                    target.compress(Bitmap.CompressFormat.PNG, 100, buffer)
                    add(name, buffer.toByteArray())
                }

                add(
                    "summary.txt",
                    summary(stamp, durationSeconds, meshFile, screen, notes, crashFiles).toByteArray()
                )
                if (logFile != null && logFile.exists()) addFile("scan.txt", logFile)
                if (meshFile != null && meshFile.exists()) addFile("mesh.obj", meshFile)
                crashFiles.forEach { addFile("crash/${it.name}", it) }
                screen?.let { addPng("screen.png", it) }
                depthThumb?.let { addPng("depth.png", it, 4) }
                confidenceThumb?.let { addPng("confidence.png", it, 4) }
            }
            true
        } catch (t: Throwable) {
            // OutOfMemoryError is an Error, not an Exception, and it is exactly
            // what a huge mesh can raise. Losing the archive is bad; killing
            // the app before the crash handler can report it is worse.
            runCatching { zip.delete() }
            false
        }

        if (!written) return null

        // The individual pieces now live inside the archive.
        logFile?.delete()
        meshFile?.delete()
        reset()
        return zip
    }

    fun reset() {
        startedAtMillis = 0L
        depthThumb = null
        confidenceThumb = null
        lastMetrics = null
    }

    private fun crashReports(): List<File> {
        val dir = File(context.getExternalFilesDir(null) ?: context.filesDir, "crash")
        if (!dir.isDirectory) return emptyList()
        val reports = dir.listFiles { file -> file.isFile && file.name.endsWith(".txt") }
            ?: return emptyList()
        return reports.sortedByDescending { it.lastModified() }.take(3)
    }

    private fun summary(
        stamp: String,
        durationSeconds: Long,
        meshFile: File?,
        screen: Bitmap?,
        notes: List<String>,
        crashFiles: List<File>
    ): String {
        val m = lastMetrics
        val sb = StringBuilder()
        sb.appendLine("# Mobile 3D Scanner test bundle")
        sb.appendLine("run=$stamp")
        sb.appendLine("started=${SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date(startedAtMillis))}")
        sb.appendLine("device=${Build.MANUFACTURER} ${Build.MODEL}")
        sb.appendLine("android=${Build.VERSION.RELEASE} sdk=${Build.VERSION.SDK_INT}")
        sb.appendLine("abi=${Build.SUPPORTED_ABIS.firstOrNull() ?: "?"}")
        val memory = android.app.ActivityManager.MemoryInfo()
        (context.getSystemService(Context.ACTIVITY_SERVICE) as? android.app.ActivityManager)
            ?.getMemoryInfo(memory)
        sb.appendLine("ram_mb=${memory.totalMem / (1024 * 1024)}")
        sb.appendLine("heap_max_mb=${Runtime.getRuntime().maxMemory() / (1024 * 1024)}")
        sb.appendLine("duration_s=$durationSeconds")
        sb.appendLine("render_mode=$lastRenderMode")
        if (m != null) {
            sb.appendLine("keyframes=${m.keyframes}")
            sb.appendLine("triangles=${m.triangles}")
            sb.appendLine("vertices=${m.vertices}")
            sb.appendLine("blocks=${m.blocks}")
            sb.appendLine("depth_source=${m.depthSource}")
            sb.appendLine("fps_last=${"%.1f".format(m.fps)}")
            sb.appendLine("process_ms_last=${"%.0f".format(m.processMs)}")
            sb.appendLine("tracking_last=${m.tracking}")
        }
        sb.appendLine("mesh=${if (meshFile != null) "mesh.obj" else "missing"}")
        sb.appendLine("screen=${if (screen != null) "screen.png" else "missing"}")
        sb.appendLine(
            "crash=${if (crashFiles.isEmpty()) "none" else crashFiles.joinToString(",") { it.name }}"
        )
        val files = mutableListOf("scan.txt", "mesh.obj", "screen.png", "depth.png", "confidence.png")
        if (crashFiles.isNotEmpty()) files.add("crash/*.txt")
        sb.appendLine("files=${files.joinToString(" ")}")
        notes.forEach { sb.appendLine(it) }
        return sb.toString()
    }
}

/** Compose can hand out a ContextWrapper, and the screenshot needs the Activity. */
tailrec fun Context.findActivity(): Activity? = when (this) {
    is Activity -> this
    is android.content.ContextWrapper -> baseContext.findActivity()
    else -> null
}

/**
 * Copies one layer. A SurfaceView renders into its own buffer, so the window
 * layer only contains the HUD and leaves a transparent hole where the camera
 * feed and the fused surface are.
 */
suspend fun captureLayer(view: SurfaceView): Bitmap? {
    if (view.width <= 0 || view.height <= 0) return null
    return suspendCancellableCoroutine { cont ->
        val bitmap = Bitmap.createBitmap(view.width, view.height, Bitmap.Config.ARGB_8888)
        try {
            PixelCopy.request(view, bitmap, { result ->
                cont.resume(if (result == PixelCopy.SUCCESS) bitmap else null)
            }, Handler(Looper.getMainLooper()))
        } catch (e: Exception) {
            bitmap.recycle()
            cont.resume(null)
        }
    }
}

/** Full-window screenshot used as the fallback when the AR layer cannot be read. */
suspend fun captureWindow(activity: Activity?): Bitmap? {
    if (activity == null) return null
    val view = activity.window.decorView
    if (view.width <= 0 || view.height <= 0) return null
    return suspendCancellableCoroutine { cont ->
        val bitmap = Bitmap.createBitmap(view.width, view.height, Bitmap.Config.ARGB_8888)
        try {
            PixelCopy.request(activity.window, bitmap, { result ->
                cont.resume(
                    if (result == PixelCopy.SUCCESS) bitmap else null
                )
            }, Handler(Looper.getMainLooper()))
        } catch (e: Exception) {
            bitmap.recycle()
            cont.resume(null)
        }
    }
}

/**
 * The AR layer holds the camera feed and the mesh, the window layer holds the
 * HUD. Drawn in that order the transparent hole in the window shows the scene.
 */
fun composeScreen(scene: Bitmap?, overlay: Bitmap?): Bitmap? {
    if (scene == null) return overlay
    if (overlay != null) {
        Canvas(scene).drawBitmap(overlay, 0f, 0f, null)
        overlay.recycle()
    }
    return scene
}

/**
 * Shares the finished archive through the system sheet. WhatsApp accepts a zip
 * as a document; the ClipData entry carries the read grant for it.
 */
fun shareBundle(context: Context, zip: File) {
    try {
        val uri = FileProvider.getUriForFile(context, "${context.packageName}.fileprovider", zip)
        val intent = Intent(Intent.ACTION_SEND).apply {
            type = "application/zip"
            putExtra(Intent.EXTRA_STREAM, uri)
            putExtra(Intent.EXTRA_SUBJECT, "3D tarama testi")
            putExtra(Intent.EXTRA_TEXT, "3D tarama test paketi: ${zip.name}")
            clipData = ClipData.newRawUri("test", uri)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
        context.startActivity(Intent.createChooser(intent, "Test paketini paylaş"))
    } catch (_: Exception) {
    }
}
