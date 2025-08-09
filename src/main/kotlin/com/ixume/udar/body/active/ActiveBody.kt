package com.ixume.udar.body.active

import com.ixume.udar.body.CollidableBody
import com.ixume.udar.collisiondetection.broadphase.aabb.AABB
import com.ixume.udar.collisiondetection.capability.Projectable
import org.joml.Quaterniond
import org.joml.Vector2d
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlin.math.min

interface ActiveBody : CollidableBody, Projectable {
    var age: Int

    val vertices: Array<Vector3d>
    val radius: Double
    val fatBB: AABB
    val tightBB: AABB

    var awake: AtomicBoolean
    var startled: AtomicBoolean
    val linearDelta: Vector3d
    val angularDelta: Double

    val mass: Double
    val localInertia: Vector3d
    val prevQ: Quaterniond

    val hasGravity: Boolean

    fun globalToLocal(vec: Vector3d): Vector3d

    fun step() {}
    fun update() {}
    fun ensureNonAligned() {}

    /**
     * @return List of intersection positions and normals
     */
    fun intersect(origin: Vector3d, end: Vector3d): List<Pair<Vector3d, Vector3d>>
    fun visualize() {}

    fun onKill()

    override fun project(axis: Vector3d): Vector2d {
        var min = Double.MAX_VALUE
        var max = -Double.MAX_VALUE

        val vs = vertices

        if (vs.isEmpty()) return Vector2d(0.0)

        for (v in vs) {
            val s = v.dot(axis)
            min = min(min, s)
            max = max(max, s)
        }

        return Vector2d(min, max)
    }

    fun applyImpulse(
        point: Vector3d,
        normal: Vector3d,
        impulse: Vector3d,
    )

    companion object {
        const val TIME_STEP = 0.005
    }
}