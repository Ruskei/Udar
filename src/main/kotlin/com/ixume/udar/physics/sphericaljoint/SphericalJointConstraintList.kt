package com.ixume.udar.physics.sphericaljoint

import com.ixume.udar.body.active.ActiveBody
import org.joml.Matrix3f
import org.joml.Quaternionf
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

        arr[idx + BODY_A_ROT_X_OFFSET] = bodyA.q.x.toFloat()
        arr[idx + BODY_A_ROT_Y_OFFSET] = bodyA.q.y.toFloat()
        arr[idx + BODY_A_ROT_Z_OFFSET] = bodyA.q.z.toFloat()
        arr[idx + BODY_A_ROT_W_OFFSET] = bodyA.q.w.toFloat()
        arr[idx + BODY_A_POS_X_OFFSET] = bodyA.pos.x.toFloat()
        arr[idx + BODY_A_POS_Y_OFFSET] = bodyA.pos.y.toFloat()
        arr[idx + BODY_A_POS_Z_OFFSET] = bodyA.pos.z.toFloat()
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = bodyA.inverseMass.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = bodyA.inverseInertia.m00.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = bodyA.inverseInertia.m01.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = bodyA.inverseInertia.m02.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = bodyA.inverseInertia.m10.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = bodyA.inverseInertia.m11.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = bodyA.inverseInertia.m12.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = bodyA.inverseInertia.m20.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = bodyA.inverseInertia.m21.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = bodyA.inverseInertia.m22.toFloat()

        arr[idx + BODY_B_ROT_X_OFFSET] = bodyB.q.x.toFloat()
        arr[idx + BODY_B_ROT_Y_OFFSET] = bodyB.q.y.toFloat()
        arr[idx + BODY_B_ROT_Z_OFFSET] = bodyB.q.z.toFloat()
        arr[idx + BODY_B_ROT_W_OFFSET] = bodyB.q.w.toFloat()
        arr[idx + BODY_B_POS_X_OFFSET] = bodyB.pos.x.toFloat()
        arr[idx + BODY_B_POS_Y_OFFSET] = bodyB.pos.y.toFloat()
        arr[idx + BODY_B_POS_Z_OFFSET] = bodyB.pos.z.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = bodyB.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XX_OFFSET] = bodyB.inverseInertia.m00.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XY_OFFSET] = bodyB.inverseInertia.m01.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XZ_OFFSET] = bodyB.inverseInertia.m02.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YX_OFFSET] = bodyB.inverseInertia.m10.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YY_OFFSET] = bodyB.inverseInertia.m11.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YZ_OFFSET] = bodyB.inverseInertia.m12.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZX_OFFSET] = bodyB.inverseInertia.m20.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZY_OFFSET] = bodyB.inverseInertia.m21.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZZ_OFFSET] = bodyB.inverseInertia.m22.toFloat()

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

    fun bodyAII(constraintIdx: Int, out: Matrix3f): Matrix3f {
        val baseIdx = constraintIdx * DATA_SIZE
        return out.set(
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XX_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XY_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XZ_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YX_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YY_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YZ_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZX_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZY_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET]
        )
    }

    fun bodyBII(constraintIdx: Int, out: Matrix3f): Matrix3f {
        val baseIdx = constraintIdx * DATA_SIZE
        return out.set(
            arr[baseIdx + BODY_B_INVERSE_INERTIA_XX_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_XY_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_XZ_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_YX_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_YY_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_YZ_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_ZX_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_ZY_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_ZZ_OFFSET]
        )
    }

    fun setBodyData(
        constraintIdx: Int,
        bodyA: ActiveBody,
        bodyB: ActiveBody,
    ) {
        val idx = constraintIdx * DATA_SIZE

        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(bodyA.idx)
        arr[idx + BODY_A_ROT_X_OFFSET] = bodyA.q.x.toFloat()
        arr[idx + BODY_A_ROT_Y_OFFSET] = bodyA.q.y.toFloat()
        arr[idx + BODY_A_ROT_Z_OFFSET] = bodyA.q.z.toFloat()
        arr[idx + BODY_A_ROT_W_OFFSET] = bodyA.q.w.toFloat()
        arr[idx + BODY_A_POS_X_OFFSET] = bodyA.pos.x.toFloat()
        arr[idx + BODY_A_POS_Y_OFFSET] = bodyA.pos.y.toFloat()
        arr[idx + BODY_A_POS_Z_OFFSET] = bodyA.pos.z.toFloat()
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = bodyA.inverseMass.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = bodyA.inverseInertia.m00.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = bodyA.inverseInertia.m01.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = bodyA.inverseInertia.m02.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = bodyA.inverseInertia.m10.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = bodyA.inverseInertia.m11.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = bodyA.inverseInertia.m12.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = bodyA.inverseInertia.m20.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = bodyA.inverseInertia.m21.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = bodyA.inverseInertia.m22.toFloat()

        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(bodyB.idx)
        arr[idx + BODY_B_ROT_X_OFFSET] = bodyB.q.x.toFloat()
        arr[idx + BODY_B_ROT_Y_OFFSET] = bodyB.q.y.toFloat()
        arr[idx + BODY_B_ROT_Z_OFFSET] = bodyB.q.z.toFloat()
        arr[idx + BODY_B_ROT_W_OFFSET] = bodyB.q.w.toFloat()
        arr[idx + BODY_B_POS_X_OFFSET] = bodyB.pos.x.toFloat()
        arr[idx + BODY_B_POS_Y_OFFSET] = bodyB.pos.y.toFloat()
        arr[idx + BODY_B_POS_Z_OFFSET] = bodyB.pos.z.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = bodyB.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XX_OFFSET] = bodyB.inverseInertia.m00.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XY_OFFSET] = bodyB.inverseInertia.m01.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XZ_OFFSET] = bodyB.inverseInertia.m02.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YX_OFFSET] = bodyB.inverseInertia.m10.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YY_OFFSET] = bodyB.inverseInertia.m11.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YZ_OFFSET] = bodyB.inverseInertia.m12.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZX_OFFSET] = bodyB.inverseInertia.m20.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZY_OFFSET] = bodyB.inverseInertia.m21.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZZ_OFFSET] = bodyB.inverseInertia.m22.toFloat()
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

const val DATA_SIZE = 42

const val BODY_A_IDX_OFFSET = 0
const val BODY_B_IDX_OFFSET = 1
const val RAX_OFFSET = 2
const val RAY_OFFSET = 3
const val RAZ_OFFSET = 4
const val RBX_OFFSET = 5
const val RBY_OFFSET = 6
const val RBZ_OFFSET = 7

const val BODY_A_ROT_X_OFFSET = 8
const val BODY_A_ROT_Y_OFFSET = 9
const val BODY_A_ROT_Z_OFFSET = 10
const val BODY_A_ROT_W_OFFSET = 11
const val BODY_A_POS_X_OFFSET = 12
const val BODY_A_POS_Y_OFFSET = 13
const val BODY_A_POS_Z_OFFSET = 14
const val BODY_A_INVERSE_MASS_OFFSET = 15
const val BODY_A_INVERSE_INERTIA_XX_OFFSET = 16
const val BODY_A_INVERSE_INERTIA_XY_OFFSET = 17
const val BODY_A_INVERSE_INERTIA_XZ_OFFSET = 18
const val BODY_A_INVERSE_INERTIA_YX_OFFSET = 19
const val BODY_A_INVERSE_INERTIA_YY_OFFSET = 20
const val BODY_A_INVERSE_INERTIA_YZ_OFFSET = 21
const val BODY_A_INVERSE_INERTIA_ZX_OFFSET = 22
const val BODY_A_INVERSE_INERTIA_ZY_OFFSET = 23
const val BODY_A_INVERSE_INERTIA_ZZ_OFFSET = 24

const val BODY_B_ROT_X_OFFSET = 25
const val BODY_B_ROT_Y_OFFSET = 26
const val BODY_B_ROT_Z_OFFSET = 27
const val BODY_B_ROT_W_OFFSET = 28
const val BODY_B_POS_X_OFFSET = 29
const val BODY_B_POS_Y_OFFSET = 30
const val BODY_B_POS_Z_OFFSET = 31
const val BODY_B_INVERSE_MASS_OFFSET = 32
const val BODY_B_INVERSE_INERTIA_XX_OFFSET = 33
const val BODY_B_INVERSE_INERTIA_XY_OFFSET = 34
const val BODY_B_INVERSE_INERTIA_XZ_OFFSET = 35
const val BODY_B_INVERSE_INERTIA_YX_OFFSET = 36
const val BODY_B_INVERSE_INERTIA_YY_OFFSET = 37
const val BODY_B_INVERSE_INERTIA_YZ_OFFSET = 38
const val BODY_B_INVERSE_INERTIA_ZX_OFFSET = 39
const val BODY_B_INVERSE_INERTIA_ZY_OFFSET = 40
const val BODY_B_INVERSE_INERTIA_ZZ_OFFSET = 41

/*
Data layout (42 floats per constraint):

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

Body A data (17):
    bodyA.rot.x:                8
    bodyA.rot.y:                9
    bodyA.rot.z:                10
    bodyA.rot.w:                11
    bodyA.pos.x:                12
    bodyA.pos.y:                13
    bodyA.pos.z:                14
    bodyA.inverseMass:          15
    bodyA.inverseInertia.m00:   16
    bodyA.inverseInertia.m01:   17
    bodyA.inverseInertia.m02:   18
    bodyA.inverseInertia.m10:   19
    bodyA.inverseInertia.m11:   20
    bodyA.inverseInertia.m12:   21
    bodyA.inverseInertia.m20:   22
    bodyA.inverseInertia.m21:   23
    bodyA.inverseInertia.m22:   24

Body B data (17):
    bodyB.rot.x:                25
    bodyB.rot.y:                26
    bodyB.rot.z:                27
    bodyB.rot.w:                28
    bodyB.pos.x:                29
    bodyB.pos.y:                30
    bodyB.pos.z:                31
    bodyB.inverseMass:          32
    bodyB.inverseInertia.m00:   33
    bodyB.inverseInertia.m01:   34
    bodyB.inverseInertia.m02:   35
    bodyB.inverseInertia.m10:   36
    bodyB.inverseInertia.m11:   37
    bodyB.inverseInertia.m12:   38
    bodyB.inverseInertia.m20:   39
    bodyB.inverseInertia.m21:   40
    bodyB.inverseInertia.m22:   41
 */