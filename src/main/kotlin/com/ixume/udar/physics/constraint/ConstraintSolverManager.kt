package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.Contact

class ConstraintSolverManager(val physicsWorld: PhysicsWorld) {
    private val constraintSolver = LocalConstraintSolver(physicsWorld)


    /**
     * for A in bodies:
     *  solve all body.contacts
     *
     *  this would mean we can keep the data of A close
     *      still have to jump around to B though
     *
     *  proposed unified structure (FLATTENED) (US) :
     *  contactdata[]
     *
     *  contact data:
     *      - NORMAL ( vec3 ) 3 0
     *      - J1 ( vec 3 ) 3 3
     *      - J3 ( vec 3 ) 3 6
     *      - bias ( float ) 1 9
     *      - den ( float ) 1 10
     *      - lambda ( float ) 1 11
     *      - delta ( 4 vec3s ) 12 12
     *      - myID ( int ) 1 24
     *      - otherID ( int ) 1 25
     *
     * bodydata:
     *  - v ( vec3 ) 3 0
     *  - o ( vec3 ) 3 3
     *
     *  total: 25 floats
     *  narrowphase generates things in this way already, so take advantage!
     *  use atomic array ?
     *
     *  1. create associations (id -> Contact) and (Contact -> id)
     *  2. generate flattened list
     *  3. iterate
     *  4. write
     *
     *  for static contacts, we need to modify original body data but all of them are guaranteed to be independent
     *  so just iterate through above US body only care about the body
     *  dense takes up 1 bit per potential contact
     *  sparse takes up ~32 (could vary?) bits per contact
     *  let's just use bitset for now, it's simple and likely better since many objects tend to be colliding with the ground
     */
    fun solve() {
        constraintSolver.setup()

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

const val A2S_N_CONTACT_DATA_FLOATS = 16

const val A2S_N_NORMAL_OFFSET = 0
const val A2S_N_J1_OFFSET = 3
const val A2S_N_BIAS_OFFSET = 6
const val A2S_N_DEN_OFFSET = 7
const val A2S_N_LAMBDA_OFFSET = 8
const val A2S_N_DELTA_OFFSET = 9
const val A2S_N_MY_IDX_OFFSET = 15
