package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.position.PointConstraint
import com.ixume.udar.util.AtomicList

class ConstraintManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = ConstraintSolver(physicsWorld)

    private val pointConstraints = AtomicList<PointConstraint>()

    fun constrain(constraint: PointConstraint) = pointConstraints.add(constraint)
    fun unconstrain(constraint: PointConstraint) = pointConstraints.remove(constraint)

    fun onKill(b: ActiveBody) {
        pointConstraints.removeAll { it.b1 == b || it.b2 == b }
    }

    fun solve() {
        if (physicsWorld.activeBodies.size() == 0) return
        constraintSolver.setup(
            pointConstraints = pointConstraints.get(),
        )

        for (iteration in 1..Udar.CONFIG.collision.normalIterations) {
            constraintSolver.solve(iteration)
        }

        if (Udar.CONFIG.debug.reportLambdas) {
            constraintSolver.contactSolver.reportLambdas()
        }

        for (iteration in 1..Udar.CONFIG.collision.frictionIterations) {
            constraintSolver.solvePost()
        }

        constraintSolver.write()
    }
}