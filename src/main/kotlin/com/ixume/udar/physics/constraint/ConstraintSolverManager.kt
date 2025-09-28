package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar

class ConstraintSolverManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = LocalConstraintSolver(physicsWorld)

    fun prepare() {
        constraintSolver.prepare()
    }

    fun solve() {
        constraintSolver.setup()

        constraintSolver.warmStart()

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

const val BODY_DATA_FLOATS = 6
const val V_OFFSET = 0
const val O_OFFSET = 3

const val A2A_N_CONTACT_DATA_FLOATS = 26

const val A2A_N_NORMAL_OFFSET = 0
const val A2A_N_J1_OFFSET = 3
const val A2A_N_J3_OFFSET = 6
const val A2A_N_BIAS_OFFSET = 9
const val A2A_N_DEN_OFFSET = 10
const val A2A_N_LAMBDA_OFFSET = 11
const val A2A_N_DELTA_OFFSET = 12
const val A2A_N_MY_IDX_OFFSET = 24
const val A2A_N_OTHER_IDX_OFFSET = 25

const val A2S_N_CONTACT_DATA_FLOATS = 18

const val A2S_N_NORMAL_OFFSET = 0
const val A2S_N_J1_OFFSET = 3
const val A2S_N_BIAS_OFFSET = 6
const val A2S_N_DEN_OFFSET = 7
const val A2S_N_LAMBDA_OFFSET = 8
const val A2S_N_DELTA_OFFSET = 9
const val A2S_N_MY_IDX_OFFSET = 15
const val A2S_MANIFOLD_ID_OFFSET = 16