package com.ixume.udar.body

import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import org.bukkit.World
import org.bukkit.util.BoundingBox
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.UUID

interface ActiveBody  {
    val id: UUID
    val type: BodyType
    val velocity: Vector3d

    val world: World

    val vertices: List<Vector3d>
    val edges: List<Pair<Vector3d, Vector3d>>
    val boundingBox: BoundingBox

    val inverseMass: Double
    val inverseInertia: Vector3d
    val q: Quaterniond
    val omega: Vector3d

    val hasGravity: Boolean

    val pos: Vector3d
    fun support(dir: Vector3d): Vector3d

    fun globalToLocal(vec: Vector3d): Vector3d

    fun step() {}

    /**
     * @return List of intersection positions and normals
     */
    fun intersect(origin: Vector3d, end: Vector3d): List<Pair<Vector3d, Vector3d>>

    fun ensureNonAligned() {}

    fun collidesBody(other: ActiveBody): CollisionResult?
    fun collidesEnvironment(): List<CollisionResult>

    fun visualize() {}

    val contacts: MutableList<Contact>
    val previousContacts: List<Contact>

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

enum class BodyType {
    ACTIVE, PASSIVE
}