package com.ixume.udar.physics.splitting

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.constraint.ox
import com.ixume.udar.physics.constraint.oy
import com.ixume.udar.physics.constraint.oz
import com.ixume.udar.physics.constraint.vx
import com.ixume.udar.physics.constraint.vy
import com.ixume.udar.physics.constraint.vz
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import java.lang.Math.fma

@JvmInline
internal value class A2AConstraintBlocks(val value: FloatArray) {
    constructor (size: Int) : this(FloatArray(size))
    
    operator fun get(idx: Int): Float = value[idx]
    operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }
}

internal inline fun A2AConstraintBlocks.forEachBlock(
    numBlocks: Int,
    block: (
        b1Idx: Int,
        b2Idx: Int,

        v10: Float,
        v11: Float,
        v12: Float,
        v13: Float,
        v14: Float,
        v15: Float,

        v20: Float,
        v21: Float,
        v22: Float,
        v23: Float,
        v24: Float,
        v25: Float,

        manifoldIdx: Int,
        numConstraints: Int,
        cursor: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()
        val manifoldIdx = this[p++].toRawBits()

        val v10 = this[p++]
        val v11 = this[p++]
        val v12 = this[p++]
        val v13 = this[p++]
        val v14 = this[p++]
        val v15 = this[p++]

        val v20 = this[p++]
        val v21 = this[p++]
        val v22 = this[p++]
        val v23 = this[p++]
        val v24 = this[p++]
        val v25 = this[p++]

        val numConstraints = this[p++].toRawBits()

        block(
            b1Idx, b2Idx,

            v10,
            v11,
            v12,
            v13,
            v14,
            v15,

            v20,
            v21,
            v22,
            v23,
            v24,
            v25,

            manifoldIdx,
            numConstraints,
            p
        )

        p += numConstraints * A2A_CONSTRAINT_DATA_SIZE
        i++
    }
}

internal inline fun A2AConstraintBlocks.updateEachConstraint(
    numBlocks: Int,
    flatBodyData: FloatArray,
    temp: A2AConstraintData,
    block: (
        b1Idx: Int,
        b2Idx: Int,

        v1Idx: Int,
        v2Idx: Int,

        data: A2AConstraintData,
        lIdx: Int,

        numConstraints: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()
        p++ // manifold idx

        val v1Idx = p
        if (b1Idx == -1) {
            value.fill(0f, p, p + 6)
            p += 6
        } else {
            this[p++] = flatBodyData.vx(b1Idx)
            this[p++] = flatBodyData.vy(b1Idx)
            this[p++] = flatBodyData.vz(b1Idx)
            this[p++] = flatBodyData.ox(b1Idx)
            this[p++] = flatBodyData.oy(b1Idx)
            this[p++] = flatBodyData.oz(b1Idx)
        }

        val v2Idx = p
        if (b2Idx == -1) {
            value.fill(0f, p, p + 6)
            p += 6
        } else {
            this[p++] = flatBodyData.vx(b2Idx)
            this[p++] = flatBodyData.vy(b2Idx)
            this[p++] = flatBodyData.vz(b2Idx)
            this[p++] = flatBodyData.ox(b2Idx)
            this[p++] = flatBodyData.oy(b2Idx)
            this[p++] = flatBodyData.oz(b2Idx)
        }

        val numConstraints = this[p++].toRawBits()
        repeat(numConstraints) {
            temp.j10 = this[p++]
            temp.j11 = this[p++]
            temp.j12 = this[p++]
            temp.j13 = this[p++]
            temp.j14 = this[p++]
            temp.j15 = this[p++]

            temp.j20 = this[p++]
            temp.j21 = this[p++]
            temp.j22 = this[p++]
            temp.j23 = this[p++]
            temp.j24 = this[p++]
            temp.j25 = this[p++]

            temp.ej10 = this[p++]
            temp.ej11 = this[p++]
            temp.ej12 = this[p++]
            temp.ej13 = this[p++]
            temp.ej14 = this[p++]
            temp.ej15 = this[p++]

            temp.ej20 = this[p++]
            temp.ej21 = this[p++]
            temp.ej22 = this[p++]
            temp.ej23 = this[p++]
            temp.ej24 = this[p++]
            temp.ej25 = this[p++]

            val lIdx = p++
            temp.lambda = this[lIdx]
            temp.den = this[p++]
            temp.error = this[p++]

            temp.r1x = this[p++]
            temp.r1y = this[p++]
            temp.r1z = this[p++]

            temp.r2x = this[p++]
            temp.r2y = this[p++]
            temp.r2z = this[p++]

            temp.nx = this[p++]
            temp.ny = this[p++]
            temp.nz = this[p++]

            block(
                b1Idx,
                b2Idx,
                v1Idx,
                v2Idx,

                temp,
                lIdx,

                numConstraints,
            )
        }

        i++
    }
}

internal inline fun A2AConstraintBlocks.forEachConstraint(
    numBlocks: Int,
    temp: A2AConstraintData,
    block: (
        b1Idx: Int,
        b2Idx: Int,

        data: A2AConstraintData,

        manifoldIdx: Int,
        constraintIdx: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()
        val manifoldIdx = this[p++].toRawBits()

        p += 12

        val numConstraints = this[p++].toRawBits()
        for (j in 0..<numConstraints) {
            temp.j10 = this[p++]
            temp.j11 = this[p++]
            temp.j12 = this[p++]
            temp.j13 = this[p++]
            temp.j14 = this[p++]
            temp.j15 = this[p++]

            temp.j20 = this[p++]
            temp.j21 = this[p++]
            temp.j22 = this[p++]
            temp.j23 = this[p++]
            temp.j24 = this[p++]
            temp.j25 = this[p++]

            temp.ej10 = this[p++]
            temp.ej11 = this[p++]
            temp.ej12 = this[p++]
            temp.ej13 = this[p++]
            temp.ej14 = this[p++]
            temp.ej15 = this[p++]

            temp.ej20 = this[p++]
            temp.ej21 = this[p++]
            temp.ej22 = this[p++]
            temp.ej23 = this[p++]
            temp.ej24 = this[p++]
            temp.ej25 = this[p++]

            val lIdx = p++
            temp.lambda = this[lIdx]
            temp.den = this[p++]
            temp.error = this[p++]

            temp.r1x = this[p++]
            temp.r1y = this[p++]
            temp.r1z = this[p++]

            temp.r2x = this[p++]
            temp.r2y = this[p++]
            temp.r2z = this[p++]

            temp.nx = this[p++]
            temp.ny = this[p++]
            temp.nz = this[p++]

            block(
                b1Idx,
                b2Idx,

                temp,

                manifoldIdx, j
            )
        }

        i++
    }
}

/**
 * Returns new cursor position
 */
internal fun A2AConstraintBlocks.add(
    cursor: Int,
    temp: A2AConstraintData,

    body1Idx: Int,
    body2Idx: Int,

    numContacts: Int,
    manifoldIdx: Int,
    constraintDataSupplier: (data: A2AConstraintData, constraintIdx: Int) -> Unit,
): Int {
    var c = cursor

    this[c++] = Float.fromBits(body1Idx)
    this[c++] = Float.fromBits(body2Idx)
    this[c++] = Float.fromBits(manifoldIdx)

    //space for velocities
    value.fill(0f, c, c + 12)
    c += 12

    this[c++] = Float.fromBits(numContacts)

    for (i in 0..<numContacts) {
        constraintDataSupplier(temp, i)

        this[c++] = temp.j10
        this[c++] = temp.j11
        this[c++] = temp.j12
        this[c++] = temp.j13
        this[c++] = temp.j14
        this[c++] = temp.j15

        this[c++] = temp.j20
        this[c++] = temp.j21
        this[c++] = temp.j22
        this[c++] = temp.j23
        this[c++] = temp.j24
        this[c++] = temp.j25

        this[c++] = temp.ej10
        this[c++] = temp.ej11
        this[c++] = temp.ej12
        this[c++] = temp.ej13
        this[c++] = temp.ej14
        this[c++] = temp.ej15

        this[c++] = temp.ej20
        this[c++] = temp.ej21
        this[c++] = temp.ej22
        this[c++] = temp.ej23
        this[c++] = temp.ej24
        this[c++] = temp.ej25

        this[c++] = temp.lambda
        this[c++] = temp.den
        this[c++] = temp.error

        this[c++] = temp.r1x
        this[c++] = temp.r1y
        this[c++] = temp.r1z

        this[c++] = temp.r2x
        this[c++] = temp.r2y
        this[c++] = temp.r2z

        this[c++] = temp.nx
        this[c++] = temp.ny
        this[c++] = temp.nz
    }

    return c
}

internal data class A2AConstraintData(
    var j10: Float = 0f,
    var j11: Float = 0f,
    var j12: Float = 0f,
    var j13: Float = 0f,
    var j14: Float = 0f,
    var j15: Float = 0f,

    var j20: Float = 0f,
    var j21: Float = 0f,
    var j22: Float = 0f,
    var j23: Float = 0f,
    var j24: Float = 0f,
    var j25: Float = 0f,

    var ej10: Float = 0f,
    var ej11: Float = 0f,
    var ej12: Float = 0f,
    var ej13: Float = 0f,
    var ej14: Float = 0f,
    var ej15: Float = 0f,

    var ej20: Float = 0f,
    var ej21: Float = 0f,
    var ej22: Float = 0f,
    var ej23: Float = 0f,
    var ej24: Float = 0f,
    var ej25: Float = 0f,

    var lambda: Float = 0f,
    var den: Float = 0f,
    var error: Float = 0f,

    var r1x: Float = 0f,
    var r1y: Float = 0f,
    var r1z: Float = 0f,

    var r2x: Float = 0f,
    var r2y: Float = 0f,
    var r2z: Float = 0f,

    var nx: Float = 0f,
    var ny: Float = 0f,
    var nz: Float = 0f,
)

internal fun addA2AContact(
    b1Idx: Int,
    b1: ActiveBody,
    b2Idx: Int,
    b2: ActiveBody,

    contactIdx: Int,

    nx: Float,
    ny: Float,
    nz: Float,

    error: Float,
    lambda: Float,

    data: A2AConstraintData,
    idx: Int,

    manifolds: A2AManifoldArray,
    numPairsPerBody: IntArray,
) {
    val n1 = numPairsPerBody[b1Idx]
    val n2 = numPairsPerBody[b2Idx]

    val im1 = b1.inverseMass.toFloat()
    val im2 = b2.inverseMass.toFloat()

    val ii1 = b1.inverseInertia
    val ii2 = b2.inverseInertia

    val rax = manifolds.pointAX(contactIdx, idx) - manifolds.bodyAX(contactIdx)
    val ray = manifolds.pointAY(contactIdx, idx) - manifolds.bodyAY(contactIdx)
    val raz = manifolds.pointAZ(contactIdx, idx) - manifolds.bodyAZ(contactIdx)

    val rbx = manifolds.pointBX(contactIdx, idx) - manifolds.bodyBX(contactIdx)
    val rby = manifolds.pointBY(contactIdx, idx) - manifolds.bodyBY(contactIdx)
    val rbz = manifolds.pointBZ(contactIdx, idx) - manifolds.bodyBZ(contactIdx)

    data.j10 = nx
    data.j11 = ny
    data.j12 = nz
    data.j13 = fma(ray, nz, -raz * ny)
    data.j14 = fma(raz, nx, -rax * nz)
    data.j15 = fma(rax, ny, -ray * nx)

    data.j20 = -nx
    data.j21 = -ny
    data.j22 = -nz
    data.j23 = -fma(rby, nz, -rbz * ny)
    data.j24 = -fma(rbz, nx, -rbx * nz)
    data.j25 = -fma(rbx, ny, -rby * nx)

    data.ej10 = data.j10 * im1
    data.ej11 = data.j11 * im1
    data.ej12 = data.j12 * im1
    data.ej13 =
        fma(ii1.m00.toFloat(), data.j13, fma(ii1.m10.toFloat(), data.j14, ii1.m20.toFloat() * data.j15))
    data.ej14 =
        fma(ii1.m01.toFloat(), data.j13, fma(ii1.m11.toFloat(), data.j14, ii1.m21.toFloat() * data.j15))
    data.ej15 =
        fma(ii1.m02.toFloat(), data.j13, fma(ii1.m12.toFloat(), data.j14, ii1.m22.toFloat() * data.j15))

    data.ej20 = data.j20 * im2
    data.ej21 = data.j21 * im2
    data.ej22 = data.j22 * im2
    data.ej23 =
        fma(ii2.m00.toFloat(), data.j23, fma(ii2.m10.toFloat(), data.j24, ii2.m20.toFloat() * data.j25))
    data.ej24 =
        fma(ii2.m01.toFloat(), data.j23, fma(ii2.m11.toFloat(), data.j24, ii2.m21.toFloat() * data.j25))
    data.ej25 =
        fma(ii2.m02.toFloat(), data.j23, fma(ii2.m12.toFloat(), data.j24, ii2.m22.toFloat() * data.j25))

    data.lambda = lambda
    data.den =
        1f / (
                (data.ej10 * data.j10 + data.ej11 * data.j11 + data.ej12 * data.j12 +
                 data.j13 * data.ej13 + data.j14 * data.ej14 + data.j15 * data.ej15) * n1 +
                (data.ej20 * data.j20 + data.ej21 * data.j21 + data.ej22 * data.j22 +
                 data.j23 * data.ej23 + data.j24 * data.ej24 + data.j25 * data.ej25) * n2
             )

    data.error = error

    data.r1x = rax
    data.r1y = ray
    data.r1z = raz

    data.r2x = rbx
    data.r2y = rby
    data.r2z = rbz

    data.nx = nx
    data.ny = ny
    data.nz = nz
}

internal const val A2A_BASE_CONSTRAINT_BLOCK_SIZE = 16
internal const val A2A_CONSTRAINT_DATA_SIZE = 36
internal const val A2A_LAMBDA_OFFSET = 24
