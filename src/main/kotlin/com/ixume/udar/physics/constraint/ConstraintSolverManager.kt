package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar

class ConstraintSolverManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = LocalConstraintData(physicsWorld)

    fun solve() {
        if (physicsWorld.activeBodies.size() == 0) return
        constraintSolver.setup()
        constraintSolver.massSplittingConstraintSolver.warm()

        for (i in 1..Udar.CONFIG.collision.normalIterations) {
            constraintSolver.solve(i)
        }

        if (Udar.CONFIG.debug.reportLambdas) {
            constraintSolver.massSplittingConstraintSolver.reportLambdas()
        }

        for (i in 1..Udar.CONFIG.collision.frictionIterations) {
            constraintSolver.solvePost()
        }

        constraintSolver.write()

    }

    fun solvePositions() {
        if (physicsWorld.activeBodies.size() == 0) return
        for (i in 1..Udar.CONFIG.massSplittingConfig.posIterations) {
            constraintSolver.massSplittingConstraintSolver.solvePositions()
        }
    }
}