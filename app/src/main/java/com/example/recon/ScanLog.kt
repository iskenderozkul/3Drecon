package com.example.recon

import android.content.Context
import android.os.Build
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Test-run log. Plain text, written line by line from measured engine state,
 * then packed into the run's zip.
 */
class ScanLog(private val context: Context) {

    private var writer: BufferedWriter? = null
    private var file: File? = null

    val isRecording: Boolean
        get() = writer != null

    fun currentFile(): File? = file

    fun start(): File? {
        stop()
        return try {
            val dir = File(context.getExternalFilesDir(null) ?: context.filesDir, "logs")
            if (!dir.exists()) dir.mkdirs()
            val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            val target = File(dir, "scan_$stamp.txt")
            writer = BufferedWriter(FileWriter(target))
            file = target
            log("# Mobile 3D Scanner diagnostics")
            log("started=${SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date())}")
            log("device=${Build.MANUFACTURER} ${Build.MODEL}")
            log("android=${Build.VERSION.RELEASE} sdk=${Build.VERSION.SDK_INT}")
            log("abi=${Build.SUPPORTED_ABIS.firstOrNull() ?: "?"}")
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
        out.write(line)
        out.newLine()
        out.flush()
    }

    fun logBlock(text: String) {
        if (!isRecording || text.isBlank()) return
        text.lineSequence().forEach { if (it.isNotBlank()) log(it) }
    }

    fun stop(): File? {
        try {
            writer?.flush()
            writer?.close()
        } catch (_: Exception) {
        }
        writer = null
        return file
    }
}

