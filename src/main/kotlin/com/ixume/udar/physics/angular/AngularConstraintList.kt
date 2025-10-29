package com.ixume.udar.physics.angular

import com.ixume.udar.body.active.ActiveBody
import kotlin.math.max

class AngularConstraintList {
    var arr = FloatArray(0)
    var cursor = 0

    fun size(): Int {
        return cursor
    }

    fun add(
        bodyA: ActiveBody,
        bodyB: ActiveBody,
        bodyAAxisX: Float,
        bodyAAxisY: Float,
        bodyAAxisZ: Float,
        bodyBAxisX: Float,
        bodyBAxisY: Float,
        bodyBAxisZ: Float,
    ): Int {
        val idx = cursor * ANGULAR_DATA_SIZE
        val constraintIdx = cursor
        cursor++

        grow(idx + ANGULAR_DATA_SIZE)

        arr[idx + ANGULAR_BODY_A_IDX_OFFSET] = Float.fromBits(bodyA.idx)
        arr[idx + ANGULAR_BODY_B_IDX_OFFSET] = Float.fromBits(bodyB.idx)
        arr[idx + ANGULAR_BODY_A_AXIS_X_OFFSET] = bodyAAxisX
        arr[idx + ANGULAR_BODY_A_AXIS_Y_OFFSET] = bodyAAxisY
        arr[idx + ANGULAR_BODY_A_AXIS_Z_OFFSET] = bodyAAxisZ
        arr[idx + ANGULAR_BODY_B_AXIS_X_OFFSET] = bodyBAxisX
        arr[idx + ANGULAR_BODY_B_AXIS_Y_OFFSET] = bodyBAxisY
        arr[idx + ANGULAR_BODY_B_AXIS_Z_OFFSET] = bodyBAxisZ

        return constraintIdx
    }

    fun remove(constraintIdx: Int) {
        if (constraintIdx < 0 || constraintIdx >= cursor) return

        val lastIdx = (cursor - 1) * ANGULAR_DATA_SIZE
        val removeIdx = constraintIdx * ANGULAR_DATA_SIZE

        if (constraintIdx != cursor - 1) {
            System.arraycopy(arr, lastIdx, arr, removeIdx, ANGULAR_DATA_SIZE)
        }

        cursor--
    }

    fun bodyAIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * ANGULAR_DATA_SIZE + ANGULAR_BODY_A_IDX_OFFSET].toRawBits()
    }

    fun bodyBIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * ANGULAR_DATA_SIZE + ANGULAR_BODY_B_IDX_OFFSET].toRawBits()
    }
    
    fun clear() {
        cursor = 0
    }

    inline fun forEach(block: (constraintIdx: Int, bodyAIdx: Int, bodyBIdx: Int, bodyAAxisX: Float, bodyAAxisY: Float, bodyAAxisZ: Float, bodyBAxisX: Float, bodyBAxisY: Float, bodyBAxisZ: Float) -> Unit) {
        var i = 0
        while (i < cursor) {
            val idx = i * ANGULAR_DATA_SIZE

            val bodyAIdx = arr[idx + ANGULAR_BODY_A_IDX_OFFSET].toRawBits()
            val bodyBIdx = arr[idx + ANGULAR_BODY_B_IDX_OFFSET].toRawBits()
            val bodyAAxisX = arr[idx + ANGULAR_BODY_A_AXIS_X_OFFSET]
            val bodyAAxisY = arr[idx + ANGULAR_BODY_A_AXIS_Y_OFFSET]
            val bodyAAxisZ = arr[idx + ANGULAR_BODY_A_AXIS_Z_OFFSET]
            val bodyBAxisX = arr[idx + ANGULAR_BODY_B_AXIS_X_OFFSET]
            val bodyBAxisY = arr[idx + ANGULAR_BODY_B_AXIS_Y_OFFSET]
            val bodyBAxisZ = arr[idx + ANGULAR_BODY_B_AXIS_Z_OFFSET]

            block(i, bodyAIdx, bodyBIdx, bodyAAxisX, bodyAAxisY, bodyAAxisZ, bodyBAxisX, bodyBAxisY, bodyBAxisZ)

            i++
        }
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

const val ANGULAR_DATA_SIZE = 8
const val ANGULAR_BODY_A_IDX_OFFSET = 0
const val ANGULAR_BODY_B_IDX_OFFSET = 1
const val ANGULAR_BODY_A_AXIS_X_OFFSET = 2
const val ANGULAR_BODY_A_AXIS_Y_OFFSET = 3
const val ANGULAR_BODY_A_AXIS_Z_OFFSET = 4
const val ANGULAR_BODY_B_AXIS_X_OFFSET = 5
const val ANGULAR_BODY_B_AXIS_Y_OFFSET = 6
const val ANGULAR_BODY_B_AXIS_Z_OFFSET = 7

/*
Data layout (8 floats per constraint):

Indices (2):
    bodyAIdx: Int (as Float)    0
    bodyBIdx: Int (as Float)    1

Body A Axis (3):
    bodyAAxisX:  Float,  2
    bodyAAxisY:  Float,  3
    bodyAAxisZ:  Float,  4

Body B Axis (3):
    bodyBAxisX:  Float,  5
    bodyBAxisY:  Float,  6
    bodyBAxisZ:  Float,  7
 */
