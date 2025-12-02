package com.ixume.udar.physics.contact.v2

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldArray
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldBuffer
import java.lang.Math.fma
import kotlin.math.max

@JvmInline
internal value class A2SManifoldData(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    operator fun get(idx: Int) = value[idx]
    operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(manifolds: Int, constraints: Int): Int {
        return max(manifolds * A2S_MANIFOLD_DATA_SIZE + constraints * A2S_CONSTRAINT_DATA_SIZE, value.size)
    }

    internal inline fun add(
        cursor: Int,

        body1Idx: Int,

        im1: Float,

        numConstraints: Int,
        constraintDataSupplier: (baseIdx: Int, constraintIdx: Int) -> Unit,
    ): Int {
        value[cursor + 0] = Float.fromBits(body1Idx)
        value[cursor + 1] = im1
        value[cursor + 2] = Float.fromBits(numConstraints)

        for (i in 0..<numConstraints) {
            constraintDataSupplier(cursor + A2S_MANIFOLD_DATA_SIZE + i * A2S_CONSTRAINT_DATA_SIZE, i)
        }

        return cursor + A2S_MANIFOLD_DATA_SIZE + numConstraints * A2S_CONSTRAINT_DATA_SIZE
    }

    internal fun addData(
        baseIdx: Int,
        manifoldIdx: Int,
        constraintIdx: Int,

        b1: ActiveBody,

        nx: Float,
        ny: Float,
        nz: Float,

        bias: Float,
        lambda: Float,

        manifolds: A2SManifoldBuffer,
    ) {
        val im1 = b1.inverseMass.toFloat()

        val ii1 = b1.inverseInertia

        val rax = manifolds.pointAX(manifoldIdx, constraintIdx) - manifolds.bodyX(manifoldIdx)
        val ray = manifolds.pointAY(manifoldIdx, constraintIdx) - manifolds.bodyY(manifoldIdx)
        val raz = manifolds.pointAZ(manifoldIdx, constraintIdx) - manifolds.bodyZ(manifoldIdx)

        val j10 = nx
        val j11 = ny
        val j12 = nz
        val j13 = fma(ray, nz, -raz * ny)
        val j14 = fma(raz, nx, -rax * nz)
        val j15 = fma(rax, ny, -ray * nx)

        val ej10 = nx * im1
        val ej11 = ny * im1
        val ej12 = nz * im1
        val ej13 = fma(ii1.m00.toFloat(), j13, fma(ii1.m10.toFloat(), j14, ii1.m20.toFloat() * j15))
        val ej14 = fma(ii1.m01.toFloat(), j13, fma(ii1.m11.toFloat(), j14, ii1.m21.toFloat() * j15))
        val ej15 = fma(ii1.m02.toFloat(), j13, fma(ii1.m12.toFloat(), j14, ii1.m22.toFloat() * j15))

        val iden =
            1f / (
                    ej10 * j10 + ej11 * j11 + ej12 * j12 +
                    j13 * ej13 + j14 * ej14 + j15 * ej15
                 )

        set(
            baseIdx = baseIdx,

            j10 = j10,
            j11 = j11,
            j12 = j12,
            j13 = j13,
            j14 = j14,
            j15 = j15,

            ej13 = ej13,
            ej14 = ej14,
            ej15 = ej15,

            lambda = lambda,
            iden = iden,
            bias = bias,
        )
    }

    internal fun set(
        baseIdx: Int,

        j10: Float,
        j11: Float,
        j12: Float,
        j13: Float,
        j14: Float,
        j15: Float,

        ej13: Float,
        ej14: Float,
        ej15: Float,
        
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
        value[baseIdx + 6] = ej13
        value[baseIdx + 7] = ej14
        value[baseIdx + 8] = ej15
        value[baseIdx + 9] = lambda
        value[baseIdx + 10] = iden
        value[baseIdx + 11] = bias
    }

    internal inline fun forEachConstraint(
        numManifolds: Int,
        block: (
            b1Idx: Int,

            im1: Float,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var p = 0
        var i = 0
        while (i < numManifolds) {
            val b1Idx = this[p++].toRawBits()
            val im1 = this[p++]

            val numConstraints = this[p++].toRawBits()
            repeat(numConstraints) {
                block(b1Idx, im1, p)
                p += A2S_CONSTRAINT_DATA_SIZE
            }

            i++
        }
    }

    internal inline fun forEachManifold(
        numManifolds: Int,
        block: (
            body1Idx: Int,

            im1: Float,

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
            val im1 = this[p++]
            val numConstraints = this[p++].toRawBits()

            block(b1Idx, im1, numConstraints, i, c)

            p += numConstraints * A2S_CONSTRAINT_DATA_SIZE
            i++
        }
    }
}

/*
[]Manifold

Manifold = {
  body1Idx: Int
  numConstraints: Int
  im1: Float
  [numConstraints]Constraint
}

Constraint = {
  j10: Float
  j11: Float
  j12: Float
  j13: Float
  j14: Float
  j15: Float
  
  ej13: Float
  ej14: Float
  ej15: Float
  
  lambda: Float
  iden: Float
  bias: Float
}
 */
internal const val A2S_MANIFOLD_DATA_SIZE = 3
internal const val A2S_CONSTRAINT_DATA_SIZE = 12
internal const val A2S_LAMBDA_OFFSET = 9
internal const val A2S_IDEN_OFFSET = 10
internal const val A2S_BIAS_OFFSET = 11
