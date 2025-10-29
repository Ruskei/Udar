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
        val idx = cursor * DATA_SIZE
        val constraintIdx = cursor
        cursor++

        grow(idx + DATA_SIZE)

        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(bodyA.idx)
        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(bodyB.idx)
        arr[idx + BODY_A_AXIS_X_OFFSET] = bodyAAxisX
        arr[idx + BODY_A_AXIS_Y_OFFSET] = bodyAAxisY
        arr[idx + BODY_A_AXIS_Z_OFFSET] = bodyAAxisZ
        arr[idx + BODY_B_AXIS_X_OFFSET] = bodyBAxisX
        arr[idx + BODY_B_AXIS_Y_OFFSET] = bodyBAxisY
        arr[idx + BODY_B_AXIS_Z_OFFSET] = bodyBAxisZ

        return constraintIdx
    }

    fun remove(constraintIdx: Int) {
        if (constraintIdx < 0 || constraintIdx >= cursor) return

        val lastIdx = (cursor - 1) * DATA_SIZE
        val removeIdx = constraintIdx * DATA_SIZE

        if (constraintIdx != cursor - 1) {
            System.arraycopy(arr, lastIdx, arr, removeIdx, DATA_SIZE)
        }

        cursor--
    }

    fun bodyAIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * DATA_SIZE + BODY_A_IDX_OFFSET].toRawBits()
    }

    fun bodyBIdx(constraintIdx: Int): Int {
        return arr[constraintIdx * DATA_SIZE + BODY_B_IDX_OFFSET].toRawBits()
    }

    fun bodyAAxisX(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_A_AXIS_X_OFFSET]
    }

    fun bodyAAxisY(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_A_AXIS_Y_OFFSET]
    }

    fun bodyAAxisZ(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_A_AXIS_Z_OFFSET]
    }

    fun bodyBAxisX(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_B_AXIS_X_OFFSET]
    }

    fun bodyBAxisY(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_B_AXIS_Y_OFFSET]
    }

    fun bodyBAxisZ(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_B_AXIS_Z_OFFSET]
    }

    fun clear() {
        cursor = 0
    }

    inline fun forEach(block: (constraintIdx: Int, bodyAIdx: Int, bodyBIdx: Int, bodyAAxisX: Float, bodyAAxisY: Float, bodyAAxisZ: Float, bodyBAxisX: Float, bodyBAxisY: Float, bodyBAxisZ: Float) -> Unit) {
        var i = 0
        while (i < cursor) {
            val idx = i * DATA_SIZE

            val bodyAIdx = arr[idx + BODY_A_IDX_OFFSET].toRawBits()
            val bodyBIdx = arr[idx + BODY_B_IDX_OFFSET].toRawBits()
            val bodyAAxisX = arr[idx + BODY_A_AXIS_X_OFFSET]
            val bodyAAxisY = arr[idx + BODY_A_AXIS_Y_OFFSET]
            val bodyAAxisZ = arr[idx + BODY_A_AXIS_Z_OFFSET]
            val bodyBAxisX = arr[idx + BODY_B_AXIS_X_OFFSET]
            val bodyBAxisY = arr[idx + BODY_B_AXIS_Y_OFFSET]
            val bodyBAxisZ = arr[idx + BODY_B_AXIS_Z_OFFSET]

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

const val DATA_SIZE = 8

const val BODY_A_IDX_OFFSET = 0
const val BODY_B_IDX_OFFSET = 1
const val BODY_A_AXIS_X_OFFSET = 2
const val BODY_A_AXIS_Y_OFFSET = 3
const val BODY_A_AXIS_Z_OFFSET = 4
const val BODY_B_AXIS_X_OFFSET = 5
const val BODY_B_AXIS_Y_OFFSET = 6
const val BODY_B_AXIS_Z_OFFSET = 7

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
