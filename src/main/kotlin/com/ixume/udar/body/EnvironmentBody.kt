package com.ixume.udar.body

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.physicsWorld
import org.bukkit.World
import org.joml.Matrix3d
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.*

class EnvironmentBody(
    override val world: World
) : Body {
    override val id: UUID = UUID.randomUUID()
    override val physicsWorld: PhysicsWorld = world.physicsWorld!!
    override val pos: Vector3d = Vector3d()
    override val q: Quaterniond = Quaterniond()
    override val velocity: Vector3d = Vector3d()
    override val omega: Vector3d = Vector3d()
    override val torque: Vector3d = Vector3d()
    override val inverseMass: Double = 0.0
    override val inverseInertia: Matrix3d = Matrix3d(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    )

    override val isConvex: Boolean = false
}