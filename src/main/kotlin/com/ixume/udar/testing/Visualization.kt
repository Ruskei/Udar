package com.ixume.udar.testing

import com.ixume.proverka.Proverka
import com.ixume.proverka.feature.Feature
import com.ixume.proverka.feature.impl.PointTextDisplay
import com.ixume.proverka.feature.impl.PointTextDisplay.Companion.pointText
import net.kyori.adventure.text.format.TextColor
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d

fun World.debugConnect(start: Vector3d, end: Vector3d, options: Particle.DustOptions, interval: Double = 0.1) {
    val dir = Vector3d(end).sub(start).normalize()!!
    if (!dir.isFinite) return
    var t = 0.0
    while (t < end.distance(start)) {
        spawnParticle(
            Particle.REDSTONE,
            Location(
                this,
                start.x + dir.x * t,
                start.y + dir.y * t,
                start.z + dir.z * t,
            ),
            1, options,
        )

        t += interval
    }
}

fun World.debugConnectProverka(start: Vector3d, end: Vector3d, color: Color, interval: Double = 0.1, post: (Feature) -> Unit) {
    val wm = Proverka.INSTANCE.getWorldManager(this)
    val dir = Vector3d(end).sub(start).normalize()!!
    if (!dir.isFinite) return
    var t = 0.0
    while (t < end.distance(start)) {
        val f = PointTextDisplay(
            loc =
                Location(
                    this,
                    start.x + dir.x * t,
                    start.y + dir.y * t,
                    start.z + dir.z * t,
                ),
            text = pointText(TextColor.color(color.asRGB()))
        )
        wm.register(f)
        post(f)

        t += interval
    }
}