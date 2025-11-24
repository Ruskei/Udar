package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar

class ConstraintSolverManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = LocalConstraintSolver(physicsWorld)

    fun solve() {
        if (physicsWorld.activeBodies.size() == 0) return
        constraintSolver.setup()

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