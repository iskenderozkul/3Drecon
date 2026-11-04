package com.example.recon

import android.content.Context
import android.content.Intent
import android.os.Build
import androidx.core.content.FileProvider
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Continuous session log, written from the moment the activity starts until it
 * is destroyed. A crashed or force-killed process leaves the file without the
 * final `session_end=clean` line, which is how the next launch knows something
 * went wrong and offers the log for sharing.
 */
class ScanLog(private val context: Context) {

    private var writer: BufferedWriter? = null
    private var file: File? = null

    val isRecording: Boolean
        get() = writer != null

    fun currentFile(): File? = file

    fun start(): File? {
        stop()
        val dir = logDirectory(context)
        if (!dir.exists() && !dir.mkdirs()) return null
        prune(dir, keep = 12)
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val target = File(dir, "scan_$stamp.txt")
        return try {
            writer = BufferedWriter(FileWriter(target))
            file = target
            log("# Mobile 3D Scanner session")
            log("started=${SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date())}")
            log("app_version=${appVersion()}")
            log("device=${Build.MANUFACTURER} ${Build.MODEL}")
            log("android=${Build.VERSION.RELEASE} sdk=${Build.VERSION.SDK_INT}")
            log("abi=${Build.SUPPORTED_ABIS.firstOrNull() ?: "?"}")
            log("heap_max_mb=${Runtime.getRuntime().maxMemory() / (1024 * 1024)}")
            target
        } catch (e: Exception) {
            file = null
            writer = null
            null
        }
    }

    fun header(key: String, value: Any?) {
        if (!isRecording) return
        log("$key=$value")
    }

    fun log(line: String) {
        val out = writer ?: return
        try {
            out.write(line)
            out.newLine()
            out.flush()
        } catch (_: Exception) {
            writer = null
        }
    }

    fun logBlock(text: String) {
        if (!isRecording || text.isBlank()) return
        text.lineSequence().forEach { if (it.isNotBlank()) log(it) }
    }

    fun stop(): File? {
        val out = writer
        val current = file
        if (out != null) {
            try {
                out.write("session_end=clean")
                out.newLine()
                out.flush()
                out.close()
            } catch (_: Exception) {
            }
        }
        writer = null
        return current
    }

    /**
     * Finalises the log when the activity leaves the foreground. The file is
     * closed with a clean marker so it can be shared even if the process is
     * killed afterwards; [resume] reopens the same file on return instead of
     * starting a new one.
     */
    fun suspend(reason: String) {
        val out = writer ?: return
        try {
            out.write("session_end=clean reason=$reason")
            out.newLine()
            out.flush()
            out.close()
        } catch (_: Exception) {
        }
        writer = null
    }

    fun resume(): Boolean {
        if (writer != null) return true
        val target = file
        if (target == null || !target.exists()) return start() != null
        return try {
            writer = BufferedWriter(FileWriter(target, true))
            log("session_resume=${SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date())}")
            true
        } catch (_: Exception) {
            writer = null
            false
        }
    }

    private fun appVersion(): String = runCatching {
        val info = context.packageManager.getPackageInfo(context.packageName, 0)
        @Suppress("DEPRECATION")
        val code = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            info.longVersionCode
        } else {
            info.versionCode.toLong()
        }
        "${info.versionName ?: "?"} ($code)"
    }.getOrDefault("?")

    private fun prune(dir: File, keep: Int) {
        val logs = dir.listFiles { f ->
            f.isFile && f.name.startsWith("scan_") && f.name.endsWith(".txt")
        } ?: return
        if (logs.size < keep) return
        logs.sortedBy { it.lastModified() }
            .take(logs.size - keep + 1)
            .forEach { runCatching { it.delete() } }
    }

    companion object {

        private const val STALE_LOG_MS = 24L * 60L * 60L * 1000L

        fun logDirectory(context: Context): File =
            File(context.getExternalFilesDir(null) ?: context.filesDir, "logs")

        private fun crashDirectory(context: Context): File =
            File(context.getExternalFilesDir(null) ?: context.filesDir, "crash")

        /**
         * Returns the newest session log that never reached its clean end, i.e.
         * the previous run died without shutting down. Null when the last run
         * ended normally or there is no history yet. Logs older than a day are
         * ignored so an old build's leftovers do not force safe mode.
         */
        fun findUncleanSession(context: Context): File? {
            val newest = logDirectory(context)
                .listFiles { f -> f.isFile && f.name.startsWith("scan_") && f.name.endsWith(".txt") }
                ?.maxByOrNull { it.lastModified() } ?: return null
            val ageMs = System.currentTimeMillis() - newest.lastModified()
            if (ageMs > STALE_LOG_MS) return null
            val tail = runCatching {
                newest.bufferedReader().use { reader ->
                    val lines = reader.readLines()
                    lines.takeLast(60)
                }
            }.getOrNull() ?: return null
            if (tail.any { it.startsWith("session_end=") }) return null
            if (tail.any { it.startsWith("recovered=yes") }) return null
            return newest
        }

        fun markRecovered(file: File) {
            runCatching { file.appendText("recovered=yes\n") }
        }

        /**
         * Packs the crashed session log together with any Java crash report the
         * process managed to write, then hands the single file to the chooser
         * (WhatsApp, mail, …) through the FileProvider.
         */
        fun shareSessionLog(context: Context, session: File) {
            val payload = buildString {
                append(runCatching { session.readText() }.getOrDefault(""))
                val crash = crashDirectory(context)
                    .listFiles { f -> f.isFile && f.name.endsWith(".txt") }
                    ?.maxByOrNull { it.lastModified() }
                if (crash != null && crash.lastModified() >= session.lastModified()) {
                    appendLine()
                    appendLine("--- crash report: ${crash.name} ---")
                    append(runCatching { crash.readText() }.getOrDefault(""))
                }
            }
            val shareDir = File(context.cacheDir, "share")
            if (!shareDir.exists()) shareDir.mkdirs()
            val out = try {
                File(shareDir, "oturum_${session.name}").also { it.writeText(payload) }
            } catch (_: Exception) {
                return
            }
            val uri = try {
                FileProvider.getUriForFile(context, "${context.packageName}.fileprovider", out)
            } catch (_: Exception) {
                return
            }
            val send = Intent(Intent.ACTION_SEND).apply {
                type = "text/plain"
                putExtra(Intent.EXTRA_STREAM, uri)
                putExtra(Intent.EXTRA_TEXT, "3D Scanner oturum logu ekte.")
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            }
            val chooser = Intent.createChooser(send, "Logu paylaş").apply {
                addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            runCatching { context.startActivity(chooser) }
        }
    }
}
