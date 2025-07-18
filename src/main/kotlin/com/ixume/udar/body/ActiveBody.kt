package com.ixume.udar.body

import org.bukkit.util.BoundingBox
import org.joml.Quaterniond
import org.joml.Vector3d

interface ActiveBody : CollidableBody {
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

    fun applyImpulse(
        point: Vector3d,
        normal: Vector3d,
        impulse: Vector3d,
    )

    companion object {
        const val TIME_STEP = 0.005
    }
}