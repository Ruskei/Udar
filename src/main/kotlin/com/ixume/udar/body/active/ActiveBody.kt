package com.ixume.udar.body.active

import com.ixume.udar.body.A2ACollidable
import com.ixume.udar.body.A2SCollidable
import com.ixume.udar.body.Body
import com.ixume.udar.collisiondetection.capability.Projectable
import com.ixume.udar.dynamicaabb.AABB
import org.joml.Quaterniond
import org.joml.Vector2d
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlin.math.min

interface ActiveBody : A2ACollidable, A2SCollidable, Body, Projectable {
    /**
     * index in ActiveBodiesCollection#allBodies()
     */
    var idx: Int

    var age: Int
    var isChild: Boolean

    val vertices: Array<Vector3d>
    val faces: Array<Face>
    val edges: Array<Edge>
    var fatBB: Int
    val tightBB: AABB

    val awake: AtomicBoolean
    val startled: AtomicBoolean

    val mass: Double
    val localInertia: Vector3d
    val prevQ: Quaterniond

    var hasGravity: Boolean
    var idleTime: Int

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
}