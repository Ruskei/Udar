package com.ixume.udar.model.element.instance

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.model.element.Axis
import org.joml.Quaterniond
import org.joml.Vector3d

class CubeElement(
    val from: Vector3d,
    val to: Vector3d,
    val rotation: Rotation?
) : ModelElement {
    override fun realize(pw: PhysicsWorld, origin: Vector3d): ActiveBody {
        val dims = Vector3d(to).sub(from).div(16.0)
        val offset = Vector3d(from).div(16.0)

        val q = Quaterniond()
        val p = Vector3d(origin).add(offset).add(Vector3d(dims).mul(0.5))

        if (rotation != null) {
            val pivot = Vector3d(rotation.pivot).div(16.0)
            val o = Vector3d(offset).add(Vector3d(dims).mul(0.5)).sub(pivot)

            when (rotation.axis) {
                Axis.X -> {
                    o.rotateX(Math.toRadians(rotation.angle))
                    q.rotationX(Math.toRadians(rotation.angle))
                }
                Axis.Y -> {
                    o.rotateY(Math.toRadians(rotation.angle))
                    q.rotationY(Math.toRadians(rotation.angle))
                }
                Axis.Z -> {
                    o.rotateZ(Math.toRadians(rotation.angle))
                    q.rotationZ(Math.toRadians(rotation.angle))
                }
            }

            p.set(pivot).add(o).add(origin)
        }

        val c = Cuboid(
            world = pw.world,
            pos = p,
            velocity = Vector3d(0.0),
            width = dims.x,
            height = dims.y,
            length = dims.z,
            q = q,
            omega = Vector3d(0.0, 0.0, 0.0),
            density = 1.0,
            hasGravity = false,
        )

        return c
    }

    data class Rotation(
        val angle: Double,
        val axis: Axis,
        val pivot: Vector3d,
    )
}
