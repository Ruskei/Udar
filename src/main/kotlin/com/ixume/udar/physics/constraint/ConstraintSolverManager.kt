package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar

class ConstraintSolverManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = LocalConstraintSolver(physicsWorld)

    fun solve() {
        if (physicsWorld.activeBodies.size() == 0) return
        constraintSolver.setup()

        var jointItrs = 1
        while (jointItrs < Udar.CONFIG.sphericalJoint.iterations) {
            constraintSolver.solveJoints()

            jointItrs++
        }

        var normalItrs = 1
        while (normalItrs <= Udar.CONFIG.collision.normalIterations) {
            constraintSolver.solveNormal()

            normalItrs++
        }

        var frictionItrs = 1
        while (frictionItrs <= Udar.CONFIG.collision.frictionIterations) {
            constraintSolver.solveFriction()

            frictionItrs++
        }

        constraintSolver.write()
    }
}