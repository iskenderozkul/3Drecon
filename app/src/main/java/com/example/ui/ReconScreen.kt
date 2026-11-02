package com.example.ui

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
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
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
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
import com.example.recon.TestSession
import com.example.recon.captureLayer
import com.example.recon.captureWindow
import com.example.recon.composeScreen
import com.example.recon.findActivity
import com.example.recon.shareBundle
import com.example.ui.components.AnchorListSheet
import com.example.ui.components.ExportMeshSheet
import com.example.ui.components.ReconControlBar
import com.example.ui.components.ReconHud
import com.example.ui.components.TestBar
import com.example.ui.theme.DarkSurface
import com.example.ui.theme.RoseError
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ReconScreen() {
    val context = LocalContext.current
    val activity = remember(context) { context.findActivity() }
    val scope = rememberCoroutineScope()
    val lifecycleOwner = LocalLifecycleOwner.current

    val surfaceView = remember(context) { ReconSurfaceView(context) }

    var metrics by remember { mutableStateOf(ReconMetrics(scanning = true)) }
    var anchors by remember { mutableStateOf<List<ReconAnchor>>(emptyList()) }
    var renderMode by remember { mutableStateOf(RenderMode.AR_OVERLAY) }
    var showExportSheet by remember { mutableStateOf(false) }
    var showAnchorSheet by remember { mutableStateOf(false) }
    var errorMessage by remember { mutableStateOf<String?>(null) }

    val scanLog = remember(context) { ScanLog(context) }
    val testSession = remember(context) { TestSession(context, scanLog) }
    var testing by remember { mutableStateOf(false) }
    var packing by remember { mutableStateOf(false) }
    var elapsedSeconds by remember { mutableLongStateOf(0L) }
    var testMessage by remember { mutableStateOf<String?>(null) }

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
            scanLog.stop()
            surfaceView.release()
        }
    }

    LaunchedEffect(surfaceView) {
        while (true) {
            metrics = surfaceView.snapshotMetrics()
            anchors = surfaceView.anchors()

            if (testSession.running) {
                testSession.observe(metrics, surfaceView.currentRenderMode().name)
                elapsedSeconds = testSession.elapsedSeconds()
            }
            if (scanLog.isRecording) {
                scanLog.log(
                    "t fps=${"%.1f".format(metrics.fps)} ms=${"%.0f".format(metrics.processMs)} " +
                        "kf=${metrics.keyframes} tris=${metrics.triangles} " +
                        "blocks=${metrics.blocks} src=${metrics.depthSource} " +
                        "tracking=${metrics.tracking} scanning=${metrics.scanning}"
                )
                scanLog.logBlock(surfaceView.drainDiagnostics())
            }
            delay(500)
        }
    }

    fun startTest() {
        if (packing) return
        surfaceView.resetReconstruction()
        surfaceView.setScanning(true)
        metrics = metrics.copy(scanning = true)
        // The bundle's screenshot is the artefact that gets reviewed, so the run
        // starts in the mode that shows the fused surface, not wireframe.
        renderMode = RenderMode.AR_OVERLAY
        surfaceView.setRenderMode(renderMode)
        testMessage = null
        if (testSession.start()) {
            testSession.header("depth_api_supported", surfaceView.depthApiSupported())
            testSession.header("tracking", surfaceView.isTracking())
            testSession.header(
                "columns",
                "kf ts src cam dep valid dmin dmed dmax tris blocks ms pos yaw dh ddt"
            )
            testing = true
            elapsedSeconds = 0L
        } else {
            testMessage = "Kayıt dosyası açılamadı"
        }
    }

    fun finishTest() {
        if (packing || !testSession.running) return
        scope.launch {
            val duration = testSession.elapsedSeconds()
            val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            // Both layers first, while the live scene and the HUD are untouched.
            val scene = captureLayer(surfaceView)
            val overlay = captureWindow(activity)
            val screen = composeScreen(scene, overlay)
            packing = true
            surfaceView.setScanning(false)
            metrics = metrics.copy(scanning = false)
            testSession.observe(metrics, surfaceView.currentRenderMode().name)

            val mesh = withContext(Dispatchers.IO) {
                runCatching { surfaceView.exportMesh("OBJ") }.getOrNull()
            }
            // The full re-mesh runs inside the export and logs its stats, so the
            // log is drained once more before it is closed.
            scanLog.logBlock(surfaceView.drainDiagnostics())
            scanLog.stop()
            val zip = withContext(Dispatchers.IO) {
                testSession.pack(
                    stamp = stamp,
                    durationSeconds = duration,
                    meshFile = mesh,
                    screen = screen,
                    notes = listOf(
                        "depth_api_supported=${surfaceView.depthApiSupported()}",
                        "screen_scene=${scene != null}"
                    )
                )
            }
            packing = false
            testing = false
            if (zip != null) {
                testMessage = "Paket hazır: ${zip.name}"
                shareBundle(context, zip)
            } else {
                testMessage = "Paket oluşturulamadı"
            }
        }
    }

    LaunchedEffect(testMessage) {
        if (testMessage != null) {
            delay(4000)
            testMessage = null
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
            recording = testing,
            modifier = Modifier.align(Alignment.TopCenter)
        )

        Column(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .padding(bottom = 178.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            if (metrics.tracking) {
                Spacer(modifier = Modifier.height(8.dp))
            }
            TestBar(
                running = testing,
                busy = packing,
                elapsedSeconds = elapsedSeconds,
                onStart = { startTest() },
                onFinish = { finishTest() }
            )
        }

        testMessage?.let { message ->
            Surface(
                modifier = Modifier
                    .align(Alignment.Center)
                    .padding(24.dp),
                shape = RoundedCornerShape(16.dp),
                color = DarkSurface.copy(alpha = 0.95f)
            ) {
                Text(
                    text = message,
                    color = if (message.startsWith("Paket hazır")) {
                        com.example.ui.theme.EmeraldSuccess
                    } else {
                        RoseError
                    },
                    fontSize = 13.sp,
                    modifier = Modifier.padding(16.dp)
                )
            }
        }

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
