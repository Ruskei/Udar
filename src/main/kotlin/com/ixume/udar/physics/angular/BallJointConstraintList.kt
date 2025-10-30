package com.ixume.udar.physics.angular

import com.ixume.udar.body.active.ActiveBody
import kotlin.math.max

class BallJointConstraintList {
    var arr = FloatArray(0)
    var cursor = 0

    fun size(): Int {
        return cursor
    }

    fun add(
        bodyA: ActiveBody,
        bodyB: ActiveBody,
        jAX: Float,
        jAY: Float,
        jAZ: Float,
        jBX: Float,
        jBY: Float,
        jBZ: Float,
        gAX: Float,
        gAY: Float,
        gAZ: Float,
        gBX: Float,
        gBY: Float,
        gBZ: Float,
        swingAngle: Float,
        minTwistAngle: Float,
        maxTwistAngle: Float,
    ): Int {
        val idx = cursor * BALL_JOINT_DATA_SIZE
        val constraintIdx = cursor
        cursor++

        grow(idx + BALL_JOINT_DATA_SIZE)

        arr[idx + BALL_JOINT_BODY_A_IDX_OFFSET] = Float.fromBits(bodyA.idx)
        arr[idx + BALL_JOINT_BODY_B_IDX_OFFSET] = Float.fromBits(bodyB.idx)
        arr[idx + BALL_JOINT_JA_X_OFFSET] = jAX
        arr[idx + BALL_JOINT_JA_Y_OFFSET] = jAY
        arr[idx + BALL_JOINT_JA_Z_OFFSET] = jAZ
        arr[idx + BALL_JOINT_JB_X_OFFSET] = jBX
        arr[idx + BALL_JOINT_JB_Y_OFFSET] = jBY
        arr[idx + BALL_JOINT_JB_Z_OFFSET] = jBZ
        arr[idx + BALL_JOINT_GA_X_OFFSET] = gAX
        arr[idx + BALL_JOINT_GA_Y_OFFSET] = gAY
        arr[idx + BALL_JOINT_GA_Z_OFFSET] = gAZ
        arr[idx + BALL_JOINT_GB_X_OFFSET] = gBX
        arr[idx + BALL_JOINT_GB_Y_OFFSET] = gBY
        arr[idx + BALL_JOINT_GB_Z_OFFSET] = gBZ
        arr[idx + BALL_JOINT_SWING_ANGLE_OFFSET] = swingAngle
        arr[idx + BALL_JOINT_MIN_TWIST_ANGLE_OFFSET] = minTwistAngle
        arr[idx + BALL_JOINT_MAX_TWIST_ANGLE_OFFSET] = maxTwistAngle

        return constraintIdx
    }

    fun remove(constraintIdx: Int) {
        if (constraintIdx < 0 || constraintIdx >= cursor) return

        val lastIdx = (cursor - 1) * BALL_JOINT_DATA_SIZE
        val removeIdx = constraintIdx * BALL_JOINT_DATA_SIZE

        if (constraintIdx != cursor - 1) {
            System.arraycopy(arr, lastIdx, arr, removeIdx, BALL_JOINT_DATA_SIZE)
        }

        cursor--
    }

    fun bodyAIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_BODY_A_IDX_OFFSET].toRawBits()
    }

    fun bodyBIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_BODY_B_IDX_OFFSET].toRawBits()
    }

    fun jAX(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_JA_X_OFFSET]
    }

    fun jAY(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_JA_Y_OFFSET]
    }

    fun jAZ(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_JA_Z_OFFSET]
    }

    fun jBX(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_JB_X_OFFSET]
    }

    fun jBY(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_JB_Y_OFFSET]
    }

    fun jBZ(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_JB_Z_OFFSET]
    }

    fun gAX(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_GA_X_OFFSET]
    }

    fun gAY(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_GA_Y_OFFSET]
    }

    fun gAZ(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_GA_Z_OFFSET]
    }

    fun gBX(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_GB_X_OFFSET]
    }

    fun gBY(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_GB_Y_OFFSET]
    }

    fun gBZ(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_GB_Z_OFFSET]
    }

    fun swingAngle(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_SWING_ANGLE_OFFSET]
    }

    fun minTwistAngle(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_MIN_TWIST_ANGLE_OFFSET]
    }

    fun maxTwistAngle(constraintIdx: Int): Float {
        return arr[constraintIdx * BALL_JOINT_DATA_SIZE + BALL_JOINT_MAX_TWIST_ANGLE_OFFSET]
    }

    fun clear() {
        cursor = 0
    }

    inline fun forEach(block: (constraintIdx: Int, bodyAIdx: Int, bodyBIdx: Int, jAX: Float, jAY: Float, jAZ: Float, jBX: Float, jBY: Float, jBZ: Float, gAX: Float, gAY: Float, gAZ: Float, gBX: Float, gBY: Float, gBZ: Float, swingAngle: Float, minTwistAngle: Float, maxTwistAngle: Float) -> Unit) {
        var i = 0
        while (i < cursor) {
            val idx = i * BALL_JOINT_DATA_SIZE

            val bodyAIdx = arr[idx + BALL_JOINT_BODY_A_IDX_OFFSET].toRawBits()
            val bodyBIdx = arr[idx + BALL_JOINT_BODY_B_IDX_OFFSET].toRawBits()
            val jAX = arr[idx + BALL_JOINT_JA_X_OFFSET]
            val jAY = arr[idx + BALL_JOINT_JA_Y_OFFSET]
            val jAZ = arr[idx + BALL_JOINT_JA_Z_OFFSET]
            val jBX = arr[idx + BALL_JOINT_JB_X_OFFSET]
            val jBY = arr[idx + BALL_JOINT_JB_Y_OFFSET]
            val jBZ = arr[idx + BALL_JOINT_JB_Z_OFFSET]
            val gAX = arr[idx + BALL_JOINT_GA_X_OFFSET]
            val gAY = arr[idx + BALL_JOINT_GA_Y_OFFSET]
            val gAZ = arr[idx + BALL_JOINT_GA_Z_OFFSET]
            val gBX = arr[idx + BALL_JOINT_GB_X_OFFSET]
            val gBY = arr[idx + BALL_JOINT_GB_Y_OFFSET]
            val gBZ = arr[idx + BALL_JOINT_GB_Z_OFFSET]
            val swingAngle = arr[idx + BALL_JOINT_SWING_ANGLE_OFFSET]
            val minTwistAngle = arr[idx + BALL_JOINT_MIN_TWIST_ANGLE_OFFSET]
            val maxTwistAngle = arr[idx + BALL_JOINT_MAX_TWIST_ANGLE_OFFSET]

            block(i, bodyAIdx, bodyBIdx, jAX, jAY, jAZ, jBX, jBY, jBZ, gAX, gAY, gAZ, gBX, gBY, gBZ, swingAngle, minTwistAngle, maxTwistAngle)

            i++
        }
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

const val BALL_JOINT_DATA_SIZE = 17

const val BALL_JOINT_BODY_A_IDX_OFFSET = 0
const val BALL_JOINT_BODY_B_IDX_OFFSET = 1
const val BALL_JOINT_JA_X_OFFSET = 2
const val BALL_JOINT_JA_Y_OFFSET = 3
const val BALL_JOINT_JA_Z_OFFSET = 4
const val BALL_JOINT_JB_X_OFFSET = 5
const val BALL_JOINT_JB_Y_OFFSET = 6
const val BALL_JOINT_JB_Z_OFFSET = 7
const val BALL_JOINT_GA_X_OFFSET = 8
const val BALL_JOINT_GA_Y_OFFSET = 9
const val BALL_JOINT_GA_Z_OFFSET = 10
const val BALL_JOINT_GB_X_OFFSET = 11
const val BALL_JOINT_GB_Y_OFFSET = 12
const val BALL_JOINT_GB_Z_OFFSET = 13
const val BALL_JOINT_SWING_ANGLE_OFFSET = 14
const val BALL_JOINT_MIN_TWIST_ANGLE_OFFSET = 15
const val BALL_JOINT_MAX_TWIST_ANGLE_OFFSET = 16

/*
Data layout (17 floats per constraint):

Indices (2):
    bodyAIdx: Int (as Float)    0
    bodyBIdx: Int (as Float)    1

Axis on A (jA) (3):
    jAX:  Float,  2
    jAY:  Float,  3
    jAZ:  Float,  4

Axis on B (jB) (3):
    jBX:  Float,  5
    jBY:  Float,  6
    jBZ:  Float,  7

Reference axis on A (gA) (3):
    gAX:  Float,  8
    gAY:  Float,  9
    gAZ:  Float,  10

Reference axis on B (gB) (3):
    gBX:  Float,  11
    gBY:  Float,  12
    gBZ:  Float,  13

Angles (3):
    swingAngle:      Float,  14
    minTwistAngle:   Float,  15
    maxTwistAngle:   Float,  16
 */
