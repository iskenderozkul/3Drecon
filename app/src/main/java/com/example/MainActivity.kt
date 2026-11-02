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
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
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
import com.example.recon.NativeRecon
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

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        cameraGranted = granted
        if (granted) {
            ensureArCore()
        } else {
            statusMessage = "Camera access is required to scan your surroundings."
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        installCrashHandlers()
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
                    if (arReady) {
                        ReconScreen()
                    } else {
                        StartupScreen(
                            message = statusMessage,
                            onRequestPermission = { requestCameraPermission() }
                        )
                    }
                }
            }
        }

        requestCameraPermission()
    }

    override fun onResume() {
        super.onResume()
        if (cameraGranted && !arReady) ensureArCore()
    }

    /**
     * A fatal crash previously left no trace: the process died and the next
     * test bundle contained nothing about it. Both fatal native signals and
     * uncaught Java exceptions now write a report into the app's files
     * directory, which the next bundle carries.
     */
    private fun installCrashHandlers() {
        if (crashHandlersInstalled) return
        crashHandlersInstalled = true

        val directory = File(getExternalFilesDir(null) ?: filesDir, "crash")
        if (!directory.exists()) directory.mkdirs()
        runCatching { NativeRecon.installCrashHandler(directory.absolutePath) }

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
            ensureArCore()
        } else {
            permissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    private fun ensureArCore() {
        try {
            when (ArCoreApk.getInstance().requestInstall(this, true)) {
                ArCoreApk.InstallStatus.INSTALLED -> {
                    arReady = true
                }
                ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                    statusMessage = "Installing Google Play Services for AR…"
                }
            }
        } catch (e: UnavailableDeviceNotCompatibleException) {
            statusMessage = "This device does not support ARCore, which the 3D scanner requires."
        } catch (e: UnavailableUserDeclinedInstallationException) {
            statusMessage = "Google Play Services for AR is required. Install it and relaunch the app."
        } catch (e: Exception) {
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
