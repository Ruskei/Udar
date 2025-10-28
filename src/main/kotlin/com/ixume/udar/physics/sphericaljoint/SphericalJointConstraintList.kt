package com.ixume.udar.physics.sphericaljoint

import com.ixume.udar.body.active.ActiveBody
import kotlin.math.max

class SphericalJointConstraintList {
    var arr = FloatArray(0)
    var cursor = 0

    fun size(): Int {
        return cursor
    }

    fun add(
        bodyA: ActiveBody,
        bodyB: ActiveBody,
        rax: Float,
        ray: Float,
        raz: Float,
        rbx: Float,
        rby: Float,
        rbz: Float,
    ): Int {
        val idx = cursor * DATA_SIZE
        val constraintIdx = cursor
        cursor++

        grow(idx + DATA_SIZE)

        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(bodyA.idx)
        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(bodyB.idx)
        arr[idx + RAX_OFFSET] = rax
        arr[idx + RAY_OFFSET] = ray
        arr[idx + RAZ_OFFSET] = raz
        arr[idx + RBX_OFFSET] = rbx
        arr[idx + RBY_OFFSET] = rby
        arr[idx + RBZ_OFFSET] = rbz
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = bodyA.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = bodyB.inverseMass.toFloat()

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

    fun bodyAIM(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_A_INVERSE_MASS_OFFSET]
    }

    fun bodyBIM(constraintIdx: Int): Float {
        return arr[constraintIdx * DATA_SIZE + BODY_B_INVERSE_MASS_OFFSET]
    }

    fun clear() {
        cursor = 0
    }

    inline fun forEach(block: (constraintIdx: Int, bodyAIdx: Int, bodyBIdx: Int, rax: Float, ray: Float, raz: Float, rbx: Float, rby: Float, rbz: Float) -> Unit) {
        var i = 0
        while (i < cursor) {
            val idx = i * DATA_SIZE

            val bodyAIdx = arr[idx + BODY_A_IDX_OFFSET].toRawBits()
            val bodyBIdx = arr[idx + BODY_B_IDX_OFFSET].toRawBits()
            val rax = arr[idx + RAX_OFFSET]
            val ray = arr[idx + RAY_OFFSET]
            val raz = arr[idx + RAZ_OFFSET]
            val rbx = arr[idx + RBX_OFFSET]
            val rby = arr[idx + RBY_OFFSET]
            val rbz = arr[idx + RBZ_OFFSET]

            block(i, bodyAIdx, bodyBIdx, rax, ray, raz, rbx, rby, rbz)

            i++
        }
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

const val DATA_SIZE = 10

const val BODY_A_IDX_OFFSET = 0
const val BODY_B_IDX_OFFSET = 1
const val RAX_OFFSET = 2
const val RAY_OFFSET = 3
const val RAZ_OFFSET = 4
const val RBX_OFFSET = 5
const val RBY_OFFSET = 6
const val RBZ_OFFSET = 7
const val BODY_A_INVERSE_MASS_OFFSET = 8
const val BODY_B_INVERSE_MASS_OFFSET = 9

/*
Data layout (10 floats per constraint):

Indices (2):
    bodyAIdx: Int (as Float)    0
    bodyBIdx: Int (as Float)    1

Anchor points (6):
    rax:   Float,  2
    ray:   Float,  3
    raz:   Float,  4
    rbx:   Float,  5
    rby:   Float,  6
    rbz:   Float,  7

Inverse masses (2):
    bodyA.inverseMass:          8
    bodyB.inverseMass:          9
 */