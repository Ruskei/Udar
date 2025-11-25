package com.ixume.udar.physics.contact.v2

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.constraint.ConstraintMath
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import kotlin.math.max

@JvmInline
internal value class A2AManifoldData(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(manifolds: Int, constraints: Int): Int {
        return max(manifolds * A2A_MANIFOLD_DATA_SIZE + constraints * A2A_CONSTRAINT_DATA_SIZE, value.size)
    }

    internal inline fun add(
        cursor: Int,

        body1Idx: Int,
        body2Idx: Int,

        im1: Float,
        im2: Float,

        numConstraints: Int,
        constraintDataSupplier: (baseIdx: Int, constraintIdx: Int) -> Unit,
    ): Int {
        value[cursor + 0] = Float.fromBits(body1Idx)
        value[cursor + 1] = Float.fromBits(body2Idx)
        value[cursor + 2] = im1
        value[cursor + 3] = im2
        value[cursor + 4] = Float.fromBits(numConstraints)

        for (i in 0..<numConstraints) {
            constraintDataSupplier(cursor + A2A_MANIFOLD_DATA_SIZE + i * A2A_CONSTRAINT_DATA_SIZE, i)
        }

        return cursor + A2A_MANIFOLD_DATA_SIZE + numConstraints * A2A_CONSTRAINT_DATA_SIZE
    }

    internal fun addData(
        baseIdx: Int,
        manifoldIdx: Int,
        constraintIdx: Int,

        b1: ActiveBody,
        b2: ActiveBody,

        nx: Float,
        ny: Float,
        nz: Float,

        bias: Float,
        lambda: Float,

        manifolds: A2AManifoldArray,
    ) {
        val rax = manifolds.pointAX(manifoldIdx, constraintIdx) - manifolds.bodyAX(manifoldIdx)
        val ray = manifolds.pointAY(manifoldIdx, constraintIdx) - manifolds.bodyAY(manifoldIdx)
        val raz = manifolds.pointAZ(manifoldIdx, constraintIdx) - manifolds.bodyAZ(manifoldIdx)

        val rbx = manifolds.pointBX(manifoldIdx, constraintIdx) - manifolds.bodyBX(manifoldIdx)
        val rby = manifolds.pointBY(manifoldIdx, constraintIdx) - manifolds.bodyBY(manifoldIdx)
        val rbz = manifolds.pointBZ(manifoldIdx, constraintIdx) - manifolds.bodyBZ(manifoldIdx)

        ConstraintMath.addData(
            b1 = b1,
            b2 = b2,
            r1x = rax,
            r1y = ray,
            r1z = raz,
            r2x = rbx,
            r2y = rby,
            r2z = rbz,
            nx = nx,
            ny = ny,
            nz = nz,
            bias = bias,
            lambda = lambda,
        ) { j10, j11, j12, j13, j14, j15, j23, j24, j25, ej13, ej14, ej15, ej23, ej24, ej25, lambda, iden, bias ->
            set(
                baseIdx = baseIdx,

                j10 = j10,
                j11 = j11,
                j12 = j12,
                j13 = j13,
                j14 = j14,
                j15 = j15,

                j23 = j23,
                j24 = j24,
                j25 = j25,

                ej13 = ej13,
                ej14 = ej14,
                ej15 = ej15,

                ej23 = ej23,
                ej24 = ej24,
                ej25 = ej25,

                lambda = lambda,
                iden = iden,
                bias = bias,
            )
        }
    }

    internal fun set(
        baseIdx: Int,

        j10: Float,
        j11: Float,
        j12: Float,
        j13: Float,
        j14: Float,
        j15: Float,

        j23: Float,
        j24: Float,
        j25: Float,

        ej13: Float,
        ej14: Float,
        ej15: Float,

        ej23: Float,
        ej24: Float,
        ej25: Float,
        lambda: Float,
        iden: Float,
        bias: Float,
    ) {
        value[baseIdx + 0] = j10
        value[baseIdx + 1] = j11
        value[baseIdx + 2] = j12
        value[baseIdx + 3] = j13
        value[baseIdx + 4] = j14
        value[baseIdx + 5] = j15
        value[baseIdx + 6] = j23
        value[baseIdx + 7] = j24
        value[baseIdx + 8] = j25
        value[baseIdx + 9] = ej13
        value[baseIdx + 10] = ej14
        value[baseIdx + 11] = ej15
        value[baseIdx + 12] = ej23
        value[baseIdx + 13] = ej24
        value[baseIdx + 14] = ej25
        value[baseIdx + 15] = lambda
        value[baseIdx + 16] = iden
        value[baseIdx + 17] = bias
    }

    internal inline fun forEachConstraint(
        numManifolds: Int,
        block: (
            b1Idx: Int,
            b2Idx: Int,

            im1: Float,
            im2: Float,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var p = 0
        var i = 0
        while (i < numManifolds) {
            val b1Idx = this[p++].toRawBits()
            val b2Idx = this[p++].toRawBits()
            val im1 = this[p++]
            val im2 = this[p++]

            val numConstraints = this[p++].toRawBits()
            repeat(numConstraints) {
                block(b1Idx, b2Idx, im1, im2, p)
                p += A2A_CONSTRAINT_DATA_SIZE
            }

            i++
        }
    }

    internal inline fun forEachManifold(
        numManifolds: Int,
        block: (
            body1Idx: Int,
            body2Idx: Int,

            im1: Float,
            im2: Float,

            numConstraints: Int,
            idx: Int,
            baseIdx: Int,
        ) -> Unit,
    ) {
        var p = 0
        var i = 0
        while (i < numManifolds) {
            val c = p
            val b1Idx = this[p++].toRawBits()
            val b2Idx = this[p++].toRawBits()
            val im1 = this[p++]
            val im2 = this[p++]
            val numConstraints = this[p++].toRawBits()

            block(b1Idx, b2Idx, im1, im2, numConstraints, i, c)

            p += numConstraints * A2A_CONSTRAINT_DATA_SIZE
            i++
        }
    }
}

/*
[]Manifold

Manifold = {
  body1Idx: Int
  body2Idx: Int
  numConstraints: Int
  im1: Float, im2: Float
  [numConstraints]Constraint
}

Constraint = {
  j10: Float
  j11: Float
  j12: Float
  j13: Float
  j14: Float
  j15: Float
  //omit j20..2 since those are just j10..3 negated
  j23: Float
  j24: Float
  j25: Float
  
  ej13: Float
  ej14: Float
  ej15: Float
  
  ej23: Float
  ej24: Float
  ej25: Float
  
  lambda: Float
  iden: Float
  bias: Float
}
 */
internal const val A2A_MANIFOLD_DATA_SIZE = 5
internal const val A2A_CONSTRAINT_DATA_SIZE = 18
internal const val A2A_LAMBDA_OFFSET = 15
internal const val A2A_IDEN_OFFSET = 16
internal const val A2A_BIAS_OFFSET = 17
