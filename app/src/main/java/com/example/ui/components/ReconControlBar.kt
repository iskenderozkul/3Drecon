package com.example.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.BookmarkAdd
import androidx.compose.material.icons.filled.DeleteSweep
import androidx.compose.material.icons.filled.FileDownload
import androidx.compose.material.icons.filled.Layers
import androidx.compose.material.icons.filled.Pause
import androidx.compose.material.icons.filled.Place
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.recon.RenderMode
import com.example.ui.theme.AmberAccent
import com.example.ui.theme.CyanAccent
import com.example.ui.theme.DarkSurface
import com.example.ui.theme.DarkSurfaceElevated
import com.example.ui.theme.EmeraldSuccess
import com.example.ui.theme.RoseError

@Composable
fun ReconControlBar(
    isScanning: Boolean,
    currentRenderMode: RenderMode,
    onToggleScan: () -> Unit,
    onSelectRenderMode: (RenderMode) -> Unit,
    onCycleRenderMode: () -> Unit,
    onAddAnchor: () -> Unit,
    onOpenAnchors: () -> Unit,
    onOpenExport: () -> Unit,
    onResetScan: () -> Unit,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier
            .fillMaxWidth()
            .navigationBarsPadding()
            .padding(horizontal = 16.dp, vertical = 12.dp),
        shape = RoundedCornerShape(28.dp),
        color = DarkSurface.copy(alpha = 0.92f),
        tonalElevation = 8.dp,
        shadowElevation = 12.dp
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 12.dp, vertical = 10.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            // Top mini render mode chips
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 8.dp),
                horizontalArrangement = Arrangement.SpaceEvenly,
                verticalAlignment = Alignment.CenterVertically
            ) {
                RenderMode.values().forEach { mode ->
                    val isSelected = currentRenderMode == mode
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(12.dp))
                            .background(if (isSelected) CyanAccent.copy(alpha = 0.25f) else DarkSurfaceElevated)
                            .clickable { if (!isSelected) onSelectRenderMode(mode) }
                            .padding(horizontal = 10.dp, vertical = 5.dp)
                    ) {
                        Text(
                            text = mode.title,
                            color = if (isSelected) CyanAccent else Color.LightGray,
                            fontSize = 11.sp,
                            style = MaterialTheme.typography.labelSmall
                        )
                    }
                }
            }

            // Bottom Action buttons row
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                // Clear / Reset
                ActionButton(
                    icon = Icons.Default.DeleteSweep,
                    label = "Reset",
                    tint = RoseError,
                    onClick = onResetScan,
                    tag = "reset_scan_button"
                )

                // Add AR Anchor
                ActionButton(
                    icon = Icons.Default.BookmarkAdd,
                    label = "Anchor",
                    tint = AmberAccent,
                    onClick = onAddAnchor,
                    tag = "add_anchor_button"
                )

                // World-locked anchor list
                ActionButton(
                    icon = Icons.Default.Place,
                    label = "Anchors",
                    tint = AmberAccent,
                    onClick = onOpenAnchors,
                    tag = "open_anchors_button"
                )

                // Main Scan / Pause FAB
                Box(
                    modifier = Modifier
                        .size(58.dp)
                        .clip(CircleShape)
                        .background(if (isScanning) EmeraldSuccess else RoseError)
                        .clickable(onClick = onToggleScan)
                        .testTag("toggle_scan_button"),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        imageVector = if (isScanning) Icons.Default.Pause else Icons.Default.PlayArrow,
                        contentDescription = if (isScanning) "Pause Scan" else "Start Scan",
                        tint = Color.Black,
                        modifier = Modifier.size(32.dp)
                    )
                }

                // Render Mode Switcher
                ActionButton(
                    icon = Icons.Default.Layers,
                    label = "Mode",
                    tint = CyanAccent,
                    onClick = onCycleRenderMode,
                    tag = "cycle_render_mode_button"
                )

                // Export 3D Mesh
                ActionButton(
                    icon = Icons.Default.FileDownload,
                    label = "Export",
                    tint = CyanAccent,
                    onClick = onOpenExport,
                    tag = "export_mesh_button"
                )
            }
        }
    }
}

@Composable
private fun ActionButton(
    icon: ImageVector,
    label: String,
    tint: Color,
    onClick: () -> Unit,
    tag: String
) {
    Column(
        horizontalAlignment = Alignment.CenterHorizontally,
        modifier = Modifier
            .clip(RoundedCornerShape(12.dp))
            .clickable(onClick = onClick)
            .padding(horizontal = 8.dp, vertical = 4.dp)
            .testTag(tag)
    ) {
        Icon(
            imageVector = icon,
            contentDescription = label,
            tint = tint,
            modifier = Modifier.size(24.dp)
        )
        Text(
            text = label,
            color = Color.LightGray,
            fontSize = 10.sp,
            style = MaterialTheme.typography.labelSmall
        )
    }
}
