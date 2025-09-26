package com.ixume.udar.testing

import org.bukkit.Location
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d

fun World.debugConnect(start: Vector3d, end: Vector3d, options: Particle.DustOptions, interval: Double = 0.2) {
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