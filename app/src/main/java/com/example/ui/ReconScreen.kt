package com.example.ui

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.rememberModalBottomSheetState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import com.example.ar.ReconSurfaceView
import com.example.recon.ReconAnchor
import com.example.recon.ReconMetrics
import com.example.recon.RenderMode
import com.example.recon.ScanLog
import com.example.ui.components.AnchorListSheet
import com.example.ui.components.ExportMeshSheet
import com.example.ui.components.ReconControlBar
import com.example.ui.components.ReconHud
import com.example.ui.theme.DarkSurface
import com.example.ui.theme.RoseError
import kotlinx.coroutines.delay

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ReconScreen(scanLog: ScanLog) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current

    val surfaceView = remember(context) { ReconSurfaceView(context) }

    var metrics by remember { mutableStateOf(ReconMetrics(scanning = true)) }
    var anchors by remember { mutableStateOf<List<ReconAnchor>>(emptyList()) }
    var renderMode by remember { mutableStateOf(RenderMode.AR_OVERLAY) }
    var showExportSheet by remember { mutableStateOf(false) }
    var showAnchorSheet by remember { mutableStateOf(false) }
    var errorMessage by remember { mutableStateOf<String?>(null) }

    val exportSheetState = rememberModalBottomSheetState(skipPartiallyExpanded = true)
    val anchorSheetState = rememberModalBottomSheetState(skipPartiallyExpanded = true)

    DisposableEffect(lifecycleOwner, surfaceView) {
        val observer = LifecycleEventObserver { _, event ->
            when (event) {
                Lifecycle.Event.ON_RESUME -> surfaceView.onResumeSession()
                Lifecycle.Event.ON_PAUSE -> surfaceView.onPauseSession()
                else -> Unit
            }
        }
        lifecycleOwner.lifecycle.addObserver(observer)
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            surfaceView.release()
        }
    }

    LaunchedEffect(surfaceView) {
        while (true) {
            metrics = surfaceView.snapshotMetrics()
            anchors = surfaceView.anchors()
            scanLog.log(
                "t fps=${"%.1f".format(metrics.fps)} ms=${"%.0f".format(metrics.processMs)} " +
                    "kf=${metrics.keyframes} tris=${metrics.triangles} " +
                    "blocks=${metrics.blocks} src=${metrics.depthSource} " +
                    "tracking=${metrics.tracking} scanning=${metrics.scanning}"
            )
            scanLog.logBlock(surfaceView.drainDiagnostics())
            delay(500)
        }
    }

    Box(modifier = Modifier.fillMaxSize()) {
        AndroidView(
            factory = {
                surfaceView.apply {
                    onError = { message ->
                        errorMessage = message
                        scanLog.log("error=$message")
                    }
                }
            },
            modifier = Modifier.fillMaxSize()
        )

        ReconHud(
            metrics = metrics,
            anchorCount = anchors.size,
            recording = metrics.scanning,
            modifier = Modifier.align(Alignment.TopCenter)
        )

        errorMessage?.let { message ->
            Surface(
                modifier = Modifier
                    .align(Alignment.Center)
                    .padding(24.dp),
                shape = RoundedCornerShape(16.dp),
                color = DarkSurface.copy(alpha = 0.95f)
            ) {
                Text(
                    text = message,
                    color = RoseError,
                    fontSize = 13.sp,
                    modifier = Modifier.padding(16.dp)
                )
            }
        }

        ReconControlBar(
            isScanning = metrics.scanning,
            currentRenderMode = renderMode,
            onToggleScan = {
                val next = !surfaceView.isScanning()
                surfaceView.setScanning(next)
                metrics = metrics.copy(scanning = next)
            },
            onSelectRenderMode = { mode ->
                renderMode = mode
                surfaceView.setRenderMode(mode)
            },
            onCycleRenderMode = {
                val next = renderMode.next()
                renderMode = next
                surfaceView.setRenderMode(next)
            },
            onAddAnchor = { surfaceView.addAnchor() },
            onOpenAnchors = { showAnchorSheet = true },
            onOpenExport = { showExportSheet = true },
            onResetScan = {
                surfaceView.resetReconstruction()
            },
            modifier = Modifier.align(Alignment.BottomCenter)
        )

        if (showExportSheet) {
            ExportMeshSheet(
                sheetState = exportSheetState,
                triangleCount = metrics.triangles,
                vertexCount = metrics.vertices,
                onDismiss = { showExportSheet = false },
                onExport = { format -> surfaceView.exportMesh(format) }
            )
        }

        if (showAnchorSheet) {
            AnchorListSheet(
                sheetState = anchorSheetState,
                anchors = anchors,
                onDismiss = { showAnchorSheet = false },
                onRemoveAnchor = { id ->
                    surfaceView.removeAnchor(id)
                    anchors = anchors.filterNot { it.id == id }
                }
            )
        }
    }
}
