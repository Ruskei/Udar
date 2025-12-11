package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.cone.ConeConstraint
import com.ixume.udar.physics.hinge.HingeConstraint
import com.ixume.udar.physics.position.PointConstraint
import com.ixume.udar.util.AtomicList

class ConstraintManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = ConstraintSolver(physicsWorld)

    private val pointConstraints = AtomicList<PointConstraint>()
    private val hingeConstraints = AtomicList<HingeConstraint>()
    private val coneConstraints = AtomicList<ConeConstraint>()

    fun constrain(constraint: PointConstraint) = pointConstraints.add(constraint)
    fun constrain(constraint: HingeConstraint) = hingeConstraints.add(constraint)
    fun constrain(constraint: ConeConstraint) = coneConstraints.add(constraint)
    fun unconstrain(constraint: PointConstraint) = pointConstraints.remove(constraint)
    fun unconstrain(constraint: HingeConstraint) = hingeConstraints.remove(constraint)
    fun unconstrain(constraint: ConeConstraint) = coneConstraints.remove(constraint)

    fun onKill(b: ActiveBody) {
        pointConstraints.removeAll { it.b1.uuid == b.uuid || it.b2.uuid == b.uuid }
        hingeConstraints.removeAll { it.b1.uuid == b.uuid || it.b2.uuid == b.uuid }
        coneConstraints.removeAll { it.b1.uuid == b.uuid || it.b2.uuid == b.uuid }
    }

    fun solve() {
        if (physicsWorld.activeBodies.size() == 0) return
        constraintSolver.setup(
            pointConstraints = pointConstraints.get(),
            hingeConstraints = hingeConstraints.get(),
            coneConstraints = coneConstraints.get(),
        )

        for (iteration in 1..Udar.CONFIG.collision.normalIterations) {
            constraintSolver.debugData.iteration = iteration
            constraintSolver.solve()
        }

        if (Udar.CONFIG.debug.reportLambdas) {
            constraintSolver.reportLambdas()
        }

        for (iteration in 1..Udar.CONFIG.collision.frictionIterations) {
            constraintSolver.solvePost()
        }

        constraintSolver.write()
    }

    fun solvePositions() {
        if (physicsWorld.activeBodies.size() == 0) return

        for (iteration in 1..Udar.CONFIG.collision.posIterations) {
            constraintSolver.solvePositions()
        }
    }
}