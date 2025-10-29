package com.ixume.udar.physics.angular

import com.ixume.udar.AtomicList
import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import it.unimi.dsi.fastutil.ints.IntArrayList
import org.joml.Vector3f

data class AngularConstraintRequest(
    val a: ActiveBody,
    val b: ActiveBody,
    val bodyAAxis: Vector3f,
    val bodyBAxis: Vector3f,
)

class AngularConstraintManager(val physicsWorld: PhysicsWorld) {
    val constraints = AngularConstraintList()
    private val toAdd = AtomicList<AngularConstraintRequest>()

    fun addConstraint(a: ActiveBody, b: ActiveBody, bodyAAxis: Vector3f, bodyBAxis: Vector3f) {
        toAdd.add(AngularConstraintRequest(a, b, bodyAAxis, bodyBAxis))
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
                request.bodyAAxis.x,
                request.bodyAAxis.y,
                request.bodyAAxis.z,
                request.bodyBAxis.x,
                request.bodyBAxis.y,
                request.bodyBAxis.z
            )
        }
    }
}
