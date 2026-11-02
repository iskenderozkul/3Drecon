package com.example.recon

import java.nio.ByteBuffer

/**
 * JNI surface of the native reconstruction core (librecon.so).
 *
 * Every function is a thin, allocation-free bridge into the C++ engine; the
 * handle is created once per GL context and owned by the GL renderer.
 */
object NativeRecon {

    init {
        System.loadLibrary("recon")
    }

    external fun create(): Long
    external fun destroy(handle: Long)

    /** Routes fatal native signals to a crash report inside [directory]. */
    external fun installCrashHandler(directory: String)

    external fun glInit(handle: Long): Boolean
    external fun glShutdown(handle: Long)

    external fun submitFrame(
        handle: Long,
        y: ByteBuffer, yRowStride: Int, yPixelStride: Int,
        u: ByteBuffer, v: ByteBuffer, uvRowStride: Int, uvPixelStride: Int,
        width: Int, height: Int, timestampNs: Long,
        qx: Float, qy: Float, qz: Float, qw: Float,
        tx: Float, ty: Float, tz: Float,
        fx: Float, fy: Float, cx: Float, cy: Float,
        tracking: Boolean,
        depth: ByteBuffer?, depthRowStride: Int, depthWidth: Int, depthHeight: Int,
        depthTimestampNs: Long, depthApiActive: Boolean,
        depthInfo: FloatArray
    )

    external fun render(
        handle: Long,
        cameraTexture: Int,
        viewMatrix: FloatArray,
        projectionMatrix: FloatArray,
        backgroundUv: FloatArray,
        width: Int,
        height: Int,
        mode: Int,
        anchors: FloatArray
    ): Boolean

    external fun getStats(handle: Long, floats: FloatArray, ints: IntArray)
    external fun setScanning(handle: Long, scanning: Boolean)
    external fun reset(handle: Long)
    external fun exportMesh(handle: Long, path: String, format: String): Boolean
    external fun depthPreview(handle: Long, depthOut: IntArray, confidenceOut: IntArray)
    external fun hasMesh(handle: Long): Boolean
    external fun getDiagnostics(handle: Long): String
}
