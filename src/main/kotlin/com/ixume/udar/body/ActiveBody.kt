package com.ixume.udar.body

import com.ixume.udar.collisiondetection.capability.Projectable
import org.bukkit.util.BoundingBox
import org.joml.Quaterniond
import org.joml.Vector3d
import kotlin.math.max
import kotlin.math.min

interface ActiveBody : CollidableBody, Projectable {
    val vertices: List<Vector3d>
    val edges: List<Pair<Vector3d, Vector3d>>
    val boundingBox: BoundingBox

    val prevQ: Quaterniond

    val hasGravity: Boolean

    fun globalToLocal(vec: Vector3d): Vector3d

    fun step() {}

    /**
     * @return List of intersection positions and normals
     */
    fun intersect(origin: Vector3d, end: Vector3d): List<Pair<Vector3d, Vector3d>>

    fun ensureNonAligned() {}

    fun visualize() {}

    fun kill()

    override fun project(axis: Vector3d): Pair<Double, Double> {
        var min = Double.MAX_VALUE
        var max = -Double.MAX_VALUE

        val vs = vertices

        if (vs.isEmpty()) return 0.0 to 0.0

        for (v in vs) {
            val s = v.dot(axis)
            min = min(min, s)
            max = max(max, s)
        }

        return min to max
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