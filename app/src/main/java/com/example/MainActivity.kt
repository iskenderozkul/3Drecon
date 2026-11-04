package com.example

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.recon.ScanLog
import com.example.ui.ReconScreen
import com.example.ui.theme.CyanAccent
import com.example.ui.theme.DarkBackground
import com.example.ui.theme.TextPrimary
import com.example.ui.theme.TextSecondary
import com.example.ui.theme.Mobile3DReconTheme
import com.google.ar.core.ArCoreApk
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MainActivity : ComponentActivity() {

    private var cameraGranted by mutableStateOf(false)
    private var arReady by mutableStateOf(false)
    private var statusMessage by mutableStateOf("Preparing the AR scanner…")

    /** Set when the previous run died without writing its clean end marker. */
    private var crashedSession by mutableStateOf<File?>(null)

    private lateinit var sessionLog: ScanLog

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        cameraGranted = granted
        sessionLog.log("stage=camera_permission granted=$granted")
        if (granted) {
            ensureArCore()
        } else {
            statusMessage = "Camera access is required to scan your surroundings."
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        installCrashHandlers()

        sessionLog = ScanLog(this)
        // Checked before this run's log is created, so the newest file is the
        // previous session. A missing clean-end marker means that run crashed.
        crashedSession = ScanLog.findUncleanSession(this)
        sessionLog.start()
        sessionLog.log("stage=activity_created")
        sessionLog.log(
            "previous_session=" + (crashedSession?.name ?: "clean")
        )

        // A scan is continuous work with no touch input; without this flag the
        // system display timeout puts the screen (and the AR session) to sleep
        // mid-scan, which users read as the app doing "power saving".
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        enableEdgeToEdge()

        setContent {
            Mobile3DReconTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = DarkBackground
                ) {
                    val crashed = crashedSession
                    when {
                        crashed != null -> RecoveryScreen(
                            logFile = crashed,
                            onShare = {
                                ScanLog.markRecovered(crashed)
                                sessionLog.log("stage=recovery_share")
                                ScanLog.shareSessionLog(this, crashed)
                            },
                            onContinue = {
                                ScanLog.markRecovered(crashed)
                                sessionLog.log("stage=recovery_continue")
                                crashedSession = null
                                requestCameraPermission()
                            }
                        )
                        arReady -> ReconScreen(sessionLog)
                        else -> StartupScreen(
                            message = statusMessage,
                            onRequestPermission = { requestCameraPermission() }
                        )
                    }
                }
            }
        }

        if (crashedSession == null) {
            requestCameraPermission()
        }
    }

    override fun onStart() {
        super.onStart()
        if (this::sessionLog.isInitialized) sessionLog.resume()
    }

    override fun onResume() {
        super.onResume()
        sessionLog.log("stage=activity_resumed")
        if (crashedSession == null && cameraGranted && !arReady) ensureArCore()
    }

    /**
     * Closing the log here means a normal exit (including swiping the app away
     * from recents) always leaves a finished, shareable file. Without this the
     * user had to wait for a crash to get a log at all.
     */
    override fun onStop() {
        if (this::sessionLog.isInitialized) {
            sessionLog.log("stage=activity_stopped")
            sessionLog.suspend("background")
        }
        super.onStop()
    }

    override fun onDestroy() {
        if (this::sessionLog.isInitialized) {
            sessionLog.log("stage=activity_destroyed")
            sessionLog.stop()
        }
        super.onDestroy()
    }

    /**
     * A fatal crash previously left no trace. Uncaught Java exceptions are
     * written into the app's crash directory; the native signal handler was
     * removed because replacing ART's SIGSEGV handler crashed at startup.
     * The continuous [ScanLog] covers hangs and native crashes instead.
     */
    private fun installCrashHandlers() {
        if (crashHandlersInstalled) return
        crashHandlersInstalled = true

        val directory = File(getExternalFilesDir(null) ?: filesDir, "crash")
        if (!directory.exists()) directory.mkdirs()

        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
            runCatching {
                val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
                File(directory, "java_crash_$stamp.txt").writeText(
                    buildString {
                        appendLine("thread=${thread.name}")
                        appendLine("device=${android.os.Build.MANUFACTURER} ${android.os.Build.MODEL}")
                        appendLine(
                            "android=${android.os.Build.VERSION.RELEASE} " +
                                "sdk=${android.os.Build.VERSION.SDK_INT}"
                        )
                        appendLine(Log.getStackTraceString(throwable))
                    }
                )
            }
            if (previous != null) {
                previous.uncaughtException(thread, throwable)
            } else {
                android.os.Process.killProcess(android.os.Process.myPid())
                kotlin.system.exitProcess(10)
            }
        }
    }

    private fun requestCameraPermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED
        ) {
            cameraGranted = true
            sessionLog.log("stage=camera_permission granted=true (already)")
            ensureArCore()
        } else {
            sessionLog.log("stage=camera_permission request")
            permissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    private fun ensureArCore() {
        sessionLog.log("stage=arcore_request")
        try {
            when (val status = ArCoreApk.getInstance().requestInstall(this, true)) {
                ArCoreApk.InstallStatus.INSTALLED -> {
                    sessionLog.log("stage=arcore_installed")
                    arReady = true
                }
                ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                    sessionLog.log("stage=arcore_install_requested status=$status")
                    statusMessage = "Installing Google Play Services for AR…"
                }
            }
        } catch (e: UnavailableDeviceNotCompatibleException) {
            sessionLog.log("stage=arcore_error incompatible")
            statusMessage = "This device does not support ARCore, which the 3D scanner requires."
        } catch (e: UnavailableUserDeclinedInstallationException) {
            sessionLog.log("stage=arcore_error declined")
            statusMessage = "Google Play Services for AR is required. Install it and relaunch the app."
        } catch (e: Exception) {
            sessionLog.log("stage=arcore_error ${e.javaClass.simpleName}: ${e.message}")
            statusMessage = "ARCore could not be started: ${e.message}"
        }
    }

    companion object {
        @Volatile private var crashHandlersInstalled = false
    }
}

@Composable
private fun StartupScreen(message: String, onRequestPermission: () -> Unit) {
    Box(modifier = Modifier.fillMaxSize()) {
        Column(
            modifier = Modifier
                .align(Alignment.Center)
                .padding(32.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            Text(
                text = "3D SCANNER",
                color = TextPrimary,
                fontSize = 22.sp
            )
            Text(
                text = message,
                color = TextSecondary,
                fontSize = 13.sp,
                textAlign = TextAlign.Center
            )
            Button(
                onClick = onRequestPermission,
                colors = ButtonDefaults.buttonColors(containerColor = CyanAccent)
            ) {
                Text(text = "Grant camera access", fontSize = 13.sp)
            }
        }
    }
}

/**
 * Safe mode: the previous run crashed, so the app starts here instead of
 * going straight into the AR session. The log can be shared before anything
 * that might crash again is started.
 */
@Composable
private fun RecoveryScreen(logFile: File, onShare: () -> Unit, onContinue: () -> Unit) {
    Box(modifier = Modifier.fillMaxSize()) {
        Column(
            modifier = Modifier
                .align(Alignment.Center)
                .padding(28.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(14.dp)
        ) {
            Text(
                text = "SAFE MODE",
                color = TextPrimary,
                fontSize = 22.sp
            )
            Text(
                text = "The previous session ended unexpectedly. Its log is saved and " +
                    "can be shared before the scanner starts again.",
                color = TextSecondary,
                fontSize = 13.sp,
                textAlign = TextAlign.Center
            )
            Text(
                text = logFile.name,
                color = CyanAccent,
                fontSize = 11.sp
            )
            Spacer(modifier = Modifier.height(4.dp))
            Button(
                onClick = onShare,
                colors = ButtonDefaults.buttonColors(containerColor = CyanAccent)
            ) {
                Text(text = "Share previous log", fontSize = 13.sp)
            }
            OutlinedButton(onClick = onContinue) {
                Text(text = "Continue to scanner", color = TextPrimary, fontSize = 13.sp)
            }
        }
    }
}
