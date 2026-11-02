package com.example.recon

import android.graphics.Bitmap

/**
 * Render modes handed to the native overlay shader.
 * [nativeId] must stay in sync with GLRenderer::render in gl/renderer.cpp.
 */
enum class RenderMode(val title: String, val nativeId: Int) {
    AR_OVERLAY("AR", 0),
    WIREFRAME("Wire", 1),
    NORMALS("Normal", 2),
    DEPTH_HEATMAP("Depth", 3);

    fun next(): RenderMode = values()[(ordinal + 1) % values().size]
}

/** World-locked AR anchor as exposed to the UI. */
data class ReconAnchor(
    val id: Long,
    val label: String,
    val x: Float,
    val y: Float,
    val z: Float
)

/**
 * Live pipeline numbers pulled from the native engine. Everything here is
 * measured: frame rate and processing time from the worker thread, geometry
 * counts from the incremental Marching Cubes snapshot, previews from the actual
 * stereo depth and confidence maps.
 */
data class ReconMetrics(
    val fps: Float = 0f,
    val processMs: Float = 0f,
    val triangles: Int = 0,
    val vertices: Int = 0,
    val keyframes: Int = 0,
    val blocks: Int = 0,
    // 0 = none yet, 1 = ARCore Depth API, 2 = native MVS fallback.
    val depthSource: Int = 0,
    val tracking: Boolean = false,
    val scanning: Boolean = false,
    val depthPreview: Bitmap? = null,
    val confidencePreview: Bitmap? = null
)
