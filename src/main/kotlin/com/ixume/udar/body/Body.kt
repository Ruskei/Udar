package com.ixume.udar.body

import org.bukkit.World
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.UUID

interface Body {
    val id: UUID
    val world: World

    val pos: Vector3d
    val q: Quaterniond

    val velocity: Vector3d
    val omega: Vector3d

    val inverseMass: Double
    val inverseInertia: Vector3d

    val isConvex: Boolean
}