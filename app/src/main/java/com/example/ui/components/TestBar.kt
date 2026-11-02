package com.example.ui.components

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.ui.theme.CyanAccent
import com.example.ui.theme.DarkSurface
import com.example.ui.theme.EmeraldSuccess
import com.example.ui.theme.RoseError
import com.example.ui.theme.TextSecondary

/**
 * The whole test flow in one control: it is on screen as soon as the scanner
 * opens, so starting a run never needs a menu, and finishing packs the log,
 * the mesh and the screenshots into one zip and opens the share sheet.
 */
@Composable
fun TestBar(
    running: Boolean,
    busy: Boolean,
    elapsedSeconds: Long,
    onStart: () -> Unit,
    onFinish: () -> Unit,
    modifier: Modifier = Modifier
) {
    val accent = if (running) RoseError else CyanAccent
    Surface(
        modifier = modifier.fillMaxWidth().padding(horizontal = 16.dp),
        shape = RoundedCornerShape(16.dp),
        color = DarkSurface.copy(alpha = 0.92f),
        border = BorderStroke(1.dp, accent.copy(alpha = 0.55f)),
        tonalElevation = 6.dp
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(start = 14.dp, end = 6.dp, top = 6.dp, bottom = 6.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(8.dp)
                    .clip(CircleShape)
                    .background(if (running) RoseError else EmeraldSuccess)
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text(
                text = "TEST",
                color = accent,
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text(
                text = when {
                    busy -> "Paket hazırlanıyor…"
                    running -> formatElapsed(elapsedSeconds)
                    else -> "Hepsi tek zip'te"
                },
                color = TextSecondary,
                fontSize = 11.sp,
                maxLines = 1,
                modifier = Modifier.weight(1f)
            )
            Spacer(modifier = Modifier.width(8.dp))
            when {
                busy -> CircularProgressIndicator(
                    modifier = Modifier.size(20.dp).padding(end = 4.dp),
                    color = CyanAccent,
                    strokeWidth = 2.dp
                )
                else -> Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(10.dp))
                        .background(if (running) RoseError else CyanAccent)
                        .clickable(enabled = !busy) { if (running) onFinish() else onStart() }
                        .padding(horizontal = 16.dp, vertical = 7.dp)
                        .testTag("test_toggle_button"),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = if (running) "Bitir" else "Başlat",
                        color = Color.Black,
                        fontSize = 12.sp,
                        fontWeight = FontWeight.Bold
                    )
                }
            }
        }
    }
}

private fun formatElapsed(seconds: Long): String {
    val m = seconds / 60
    val s = seconds % 60
    return "%02d:%02d".format(m, s)
}
