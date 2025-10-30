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

data class LimitedAngularConstraintRequest(
    val a: ActiveBody,
    val b: ActiveBody,
    val jA: Vector3f,
    val jB: Vector3f,
    val gA: Vector3f,
    val gB: Vector3f,
    val minAngle: Float,
    val maxAngle: Float,
)

data class BallJointConstraintRequest(
    val a: ActiveBody,
    val b: ActiveBody,
    val jA: Vector3f,
    val jB: Vector3f,
    val gA: Vector3f,
    val gB: Vector3f,
    val swingAngle: Float,
    val minTwistAngle: Float,
    val maxTwistAngle: Float,
)

data class AngularFrictionConstraintRequest(
    val a: ActiveBody,
    val b: ActiveBody,
    val friction: Float,
)

class AngularConstraintManager(val physicsWorld: PhysicsWorld) {
    val constraints = AngularConstraintList()
    val limitedConstraints = LimitedAngularConstraintList()
    val ballConstraints = BallJointConstraintList()
    private val toAdd = AtomicList<AngularConstraintRequest>()
    private val limitedToAdd = AtomicList<LimitedAngularConstraintRequest>()
    private val ballToAdd = AtomicList<BallJointConstraintRequest>()

    fun addConstraint(a: ActiveBody, b: ActiveBody, bodyAAxis: Vector3f, bodyBAxis: Vector3f) {
        toAdd.add(AngularConstraintRequest(a, b, bodyAAxis, bodyBAxis))
    }

    fun addConstraint(a: ActiveBody, b: ActiveBody, jA: Vector3f, jB: Vector3f, gA: Vector3f, gB: Vector3f, minAngle: Float, maxAngle: Float) {
        limitedToAdd.add(LimitedAngularConstraintRequest(a, b, jA, jB, gA, gB, minAngle, maxAngle))
    }

    fun addConstraint(a: ActiveBody, b: ActiveBody, jA: Vector3f, jB: Vector3f, gA: Vector3f, gB: Vector3f, swingAngle: Float, minTwistAngle: Float, maxTwistAngle: Float) {
        ballToAdd.add(BallJointConstraintRequest(a, b, jA, jB, gA, gB, swingAngle, minTwistAngle, maxTwistAngle))
    }

    private var toRemoveBuffer = IntArrayList()
    private var limitedToRemoveBuffer = IntArrayList()
    private var ballToRemoveBuffer = IntArrayList()

    fun tick() {
        val constraintsToAdd = toAdd.getAndClear()
        val limitedConstraintsToAdd = limitedToAdd.getAndClear()
        val ballConstraintsToAdd = ballToAdd.getAndClear()

        // Process regular constraints
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

        // Process limited constraints
        limitedToRemoveBuffer.clear()
        limitedConstraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, _, _, _, _, _, _, _, _, _, _, _, _, _, _ ->
            val a = physicsWorld.activeBodies.fastGet(bodyAIdx)
            if (a == null || a.dead.get()) {
                limitedToRemoveBuffer.add(constraintIdx)
                return@forEach
            }

            val b = physicsWorld.activeBodies.fastGet(bodyBIdx)
            if (b == null || b.dead.get()) {
                limitedToRemoveBuffer.add(constraintIdx)
                return@forEach
            }
        }

        for (i in 0..<limitedToRemoveBuffer.size) {
            limitedConstraints.remove(limitedToRemoveBuffer.getInt(i))
        }

        for (request in limitedConstraintsToAdd) {
            limitedConstraints.add(
                request.a,
                request.b,
                request.jA.x,
                request.jA.y,
                request.jA.z,
                request.jB.x,
                request.jB.y,
                request.jB.z,
                request.gA.x,
                request.gA.y,
                request.gA.z,
                request.gB.x,
                request.gB.y,
                request.gB.z,
                request.minAngle,
                request.maxAngle
            )
        }

        // Process ball joint constraints
        ballToRemoveBuffer.clear()
        ballConstraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _ ->
            val a = physicsWorld.activeBodies.fastGet(bodyAIdx)
            if (a == null || a.dead.get()) {
                ballToRemoveBuffer.add(constraintIdx)
                return@forEach
            }

            val b = physicsWorld.activeBodies.fastGet(bodyBIdx)
            if (b == null || b.dead.get()) {
                ballToRemoveBuffer.add(constraintIdx)
                return@forEach
            }
        }

        for (i in 0..<ballToRemoveBuffer.size) {
            ballConstraints.remove(ballToRemoveBuffer.getInt(i))
        }

        for (request in ballConstraintsToAdd) {
            ballConstraints.add(
                request.a,
                request.b,
                request.jA.x,
                request.jA.y,
                request.jA.z,
                request.jB.x,
                request.jB.y,
                request.jB.z,
                request.gA.x,
                request.gA.y,
                request.gA.z,
                request.gB.x,
                request.gB.y,
                request.gB.z,
                request.swingAngle,
                request.minTwistAngle,
                request.maxTwistAngle
            )
        }
    }
}
