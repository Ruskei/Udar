package com.ixume.udar.body

import org.bukkit.World
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.*

class EnvironmentBody(
    override val world: World
) : Body {
    override val id: UUID = UUID.randomUUID()
    override val pos: Vector3d = Vector3d()
    override val q: Quaterniond = Quaterniond()
    override val velocity: Vector3d = Vector3d()
    override val omega: Vector3d = Vector3d()
    override val inverseMass: Double = 0.0
    override val inverseInertia: Vector3d = Vector3d()

    override val isConvex: Boolean = false
}