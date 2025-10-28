package com.ixume.udar.physics.sphericaljoint

import com.ixume.udar.AtomicList
import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import it.unimi.dsi.fastutil.ints.IntArrayList
import org.joml.Vector3d

data class SphericalJointRequest(
    val a: ActiveBody,
    val ra: Vector3d,
    val b: ActiveBody,
    val rb: Vector3d,
)

class SphericalJointConstraintManager(val physicsWorld: PhysicsWorld) {
    val constraints = SphericalJointConstraintList()
    private val toAdd = AtomicList<SphericalJointRequest>()

    fun addConstraint(a: ActiveBody, ra: Vector3d, b: ActiveBody, rb: Vector3d) {
        toAdd.add(SphericalJointRequest(a, ra, b, rb))
    }

    private var toRemoveBuffer = IntArrayList()

    fun tick() {
        val constraintsToAdd = toAdd.getAndClear()

        toRemoveBuffer.clear()
        constraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, _, _, _, _, _, _ ->
            val a = physicsWorld.activeBodies.fastGet(bodyAIdx)
            if (a == null || a.dead.get()) {
                toRemoveBuffer.add(constraintIdx)
                return@forEach
            }

            val b = physicsWorld.activeBodies.fastGet(bodyBIdx)
            if (b == null || b.dead.get()) {
                toRemoveBuffer.add(constraintIdx)
                return@forEach
            }
        }

        for (i in 0..<toRemoveBuffer.size) {
            constraints.remove(toRemoveBuffer.getInt(i))
        }

        for (request in constraintsToAdd) {
            constraints.add(
                request.a,
                request.b,
                request.ra.x.toFloat(),
                request.ra.y.toFloat(),
                request.ra.z.toFloat(),
                request.rb.x.toFloat(),
                request.rb.y.toFloat(),
                request.rb.z.toFloat()
            )
        }
    }
}
