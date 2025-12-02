package com.ixume.udar.physics.angular

import com.ixume.udar.body.active.ActiveBody
import kotlin.math.max

class LimitedAngularConstraintList {
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
        minAngle: Float,
        maxAngle: Float,
    ): Int {
        val idx = cursor * LIMITED_ANGULAR_DATA_SIZE
        val constraintIdx = cursor
        cursor++

        grow(idx + LIMITED_ANGULAR_DATA_SIZE)

        arr[idx + LIMITED_ANGULAR_BODY_A_IDX_OFFSET] = Float.fromBits(bodyA.idx)
        arr[idx + LIMITED_ANGULAR_BODY_B_IDX_OFFSET] = Float.fromBits(bodyB.idx)
        arr[idx + LIMITED_ANGULAR_JA_X_OFFSET] = jAX
        arr[idx + LIMITED_ANGULAR_JA_Y_OFFSET] = jAY
        arr[idx + LIMITED_ANGULAR_JA_Z_OFFSET] = jAZ
        arr[idx + LIMITED_ANGULAR_JB_X_OFFSET] = jBX
        arr[idx + LIMITED_ANGULAR_JB_Y_OFFSET] = jBY
        arr[idx + LIMITED_ANGULAR_JB_Z_OFFSET] = jBZ
        arr[idx + LIMITED_ANGULAR_GA_X_OFFSET] = gAX
        arr[idx + LIMITED_ANGULAR_GA_Y_OFFSET] = gAY
        arr[idx + LIMITED_ANGULAR_GA_Z_OFFSET] = gAZ
        arr[idx + LIMITED_ANGULAR_GB_X_OFFSET] = gBX
        arr[idx + LIMITED_ANGULAR_GB_Y_OFFSET] = gBY
        arr[idx + LIMITED_ANGULAR_GB_Z_OFFSET] = gBZ
        arr[idx + LIMITED_ANGULAR_MIN_ANGLE_OFFSET] = minAngle
        arr[idx + LIMITED_ANGULAR_MAX_ANGLE_OFFSET] = maxAngle

        return constraintIdx
    }

    fun remove(constraintIdx: Int) {
        if (constraintIdx < 0 || constraintIdx >= cursor) return

        val lastIdx = (cursor - 1) * LIMITED_ANGULAR_DATA_SIZE
        val removeIdx = constraintIdx * LIMITED_ANGULAR_DATA_SIZE

        if (constraintIdx != cursor - 1) {
            System.arraycopy(arr, lastIdx, arr, removeIdx, LIMITED_ANGULAR_DATA_SIZE)
        }

        cursor--
    }

    fun bodyAIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * LIMITED_ANGULAR_DATA_SIZE + LIMITED_ANGULAR_BODY_A_IDX_OFFSET].toRawBits()
    }

    fun bodyBIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * LIMITED_ANGULAR_DATA_SIZE + LIMITED_ANGULAR_BODY_B_IDX_OFFSET].toRawBits()
    }
    
    fun clear() {
        cursor = 0
    }

    inline fun forEach(block: (constraintIdx: Int, bodyAIdx: Int, bodyBIdx: Int, jAX: Float, jAY: Float, jAZ: Float, jBX: Float, jBY: Float, jBZ: Float, gAX: Float, gAY: Float, gAZ: Float, gBX: Float, gBY: Float, gBZ: Float, minAngle: Float, maxAngle: Float) -> Unit) {
        var i = 0
        while (i < cursor) {
            val idx = i * LIMITED_ANGULAR_DATA_SIZE

            val bodyAIdx = arr[idx + LIMITED_ANGULAR_BODY_A_IDX_OFFSET].toRawBits()
            val bodyBIdx = arr[idx + LIMITED_ANGULAR_BODY_B_IDX_OFFSET].toRawBits()
            val jAX = arr[idx + LIMITED_ANGULAR_JA_X_OFFSET]
            val jAY = arr[idx + LIMITED_ANGULAR_JA_Y_OFFSET]
            val jAZ = arr[idx + LIMITED_ANGULAR_JA_Z_OFFSET]
            val jBX = arr[idx + LIMITED_ANGULAR_JB_X_OFFSET]
            val jBY = arr[idx + LIMITED_ANGULAR_JB_Y_OFFSET]
            val jBZ = arr[idx + LIMITED_ANGULAR_JB_Z_OFFSET]
            val gAX = arr[idx + LIMITED_ANGULAR_GA_X_OFFSET]
            val gAY = arr[idx + LIMITED_ANGULAR_GA_Y_OFFSET]
            val gAZ = arr[idx + LIMITED_ANGULAR_GA_Z_OFFSET]
            val gBX = arr[idx + LIMITED_ANGULAR_GB_X_OFFSET]
            val gBY = arr[idx + LIMITED_ANGULAR_GB_Y_OFFSET]
            val gBZ = arr[idx + LIMITED_ANGULAR_GB_Z_OFFSET]
            val minAngle = arr[idx + LIMITED_ANGULAR_MIN_ANGLE_OFFSET]
            val maxAngle = arr[idx + LIMITED_ANGULAR_MAX_ANGLE_OFFSET]

            block(i, bodyAIdx, bodyBIdx, jAX, jAY, jAZ, jBX, jBY, jBZ, gAX, gAY, gAZ, gBX, gBY, gBZ, minAngle, maxAngle)

            i++
        }
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

const val LIMITED_ANGULAR_DATA_SIZE = 16
const val LIMITED_ANGULAR_BODY_A_IDX_OFFSET = 0
const val LIMITED_ANGULAR_BODY_B_IDX_OFFSET = 1
const val LIMITED_ANGULAR_JA_X_OFFSET = 2
const val LIMITED_ANGULAR_JA_Y_OFFSET = 3
const val LIMITED_ANGULAR_JA_Z_OFFSET = 4
const val LIMITED_ANGULAR_JB_X_OFFSET = 5
const val LIMITED_ANGULAR_JB_Y_OFFSET = 6
const val LIMITED_ANGULAR_JB_Z_OFFSET = 7
const val LIMITED_ANGULAR_GA_X_OFFSET = 8
const val LIMITED_ANGULAR_GA_Y_OFFSET = 9
const val LIMITED_ANGULAR_GA_Z_OFFSET = 10
const val LIMITED_ANGULAR_GB_X_OFFSET = 11
const val LIMITED_ANGULAR_GB_Y_OFFSET = 12
const val LIMITED_ANGULAR_GB_Z_OFFSET = 13
const val LIMITED_ANGULAR_MIN_ANGLE_OFFSET = 14
const val LIMITED_ANGULAR_MAX_ANGLE_OFFSET = 15

/*
Data layout (16 floats per constraint):

Indices (2):
    bodyAIdx: Int (as Float)    0
    bodyBIdx: Int (as Float)    1

Axis of rotation on A (jA) (3):
    jAX:  Float,  2
    jAY:  Float,  3
    jAZ:  Float,  4

Axis of rotation on B (jB) (3):
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

Angle limits (2):
    minAngle (phi<):  Float,  14
    maxAngle (phi>):  Float,  15
 */
