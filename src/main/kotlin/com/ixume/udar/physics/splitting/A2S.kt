package com.ixume.udar.physics.splitting

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.constraint.*
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldBuffer
import java.lang.Math.fma

@JvmInline
internal value class A2SConstraintBlocks(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    operator fun get(idx: Int): Float = value[idx]
    operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }
}

internal inline fun A2SConstraintBlocks.forEachBlock(
    numBlocks: Int,
    block: (
        b1Idx: Int,

        v10: Float,
        v11: Float,
        v12: Float,
        v13: Float,
        v14: Float,
        v15: Float,

        manifoldIdx: Int,
        numConstraints: Int,
        cursor: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val manifoldIdx = this[p++].toRawBits()

        val v10 = this[p++]
        val v11 = this[p++]
        val v12 = this[p++]
        val v13 = this[p++]
        val v14 = this[p++]
        val v15 = this[p++]

        val numConstraints = this[p++].toRawBits()

        block(
            b1Idx,

            v10,
            v11,
            v12,
            v13,
            v14,
            v15,

            manifoldIdx,
            numConstraints,
            p
        )

        p += numConstraints * A2S_CONSTRAINT_DATA_SIZE
        i++
    }
}

internal inline fun A2SConstraintBlocks.updateEachConstraint(
    numBlocks: Int,
    flatBodyData: FloatArray,
    temp: A2SConstraintData,
    block: (
        b1Idx: Int,

        v1Idx: Int,

        data: A2SConstraintData,
        lIdx: Int,

        numConstraints: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
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

        val numConstraints = this[p++].toRawBits()
        repeat(numConstraints) {
            temp.j10 = this[p++]
            temp.j11 = this[p++]
            temp.j12 = this[p++]
            temp.j13 = this[p++]
            temp.j14 = this[p++]
            temp.j15 = this[p++]

            temp.ej10 = this[p++]
            temp.ej11 = this[p++]
            temp.ej12 = this[p++]
            temp.ej13 = this[p++]
            temp.ej14 = this[p++]
            temp.ej15 = this[p++]

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
                v1Idx,

                temp,
                lIdx,

                numConstraints,
            )
        }

        i++
    }
}

internal inline fun A2SConstraintBlocks.forEachConstraint(
    numBlocks: Int,
    temp: A2SConstraintData,
    block: (
        b1Idx: Int,

        data: A2SConstraintData,

        manifoldIdx: Int,
        constraintIdx: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val manifoldIdx = this[p++].toRawBits()

        p += 6

        val numConstraints = this[p++].toRawBits()
        for (j in 0..<numConstraints) {
            temp.j10 = this[p++]
            temp.j11 = this[p++]
            temp.j12 = this[p++]
            temp.j13 = this[p++]
            temp.j14 = this[p++]
            temp.j15 = this[p++]

            temp.ej10 = this[p++]
            temp.ej11 = this[p++]
            temp.ej12 = this[p++]
            temp.ej13 = this[p++]
            temp.ej14 = this[p++]
            temp.ej15 = this[p++]

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
internal fun A2SConstraintBlocks.add(
    cursor: Int,
    temp: A2SConstraintData,

    body1Idx: Int,

    numContacts: Int,
    manifoldIdx: Int,
    constraintDataSupplier: (data: A2SConstraintData, constraintIdx: Int) -> Unit,
): Int {
    var c = cursor

    this[c++] = Float.fromBits(body1Idx)
    this[c++] = Float.fromBits(manifoldIdx)

    //space for velocities
    value.fill(0f, c, c + 6)
    c += 6

    this[c++] = Float.fromBits(numContacts)

    for (i in 0..<numContacts) {
        constraintDataSupplier(temp, i)

        this[c++] = temp.j10
        this[c++] = temp.j11
        this[c++] = temp.j12
        this[c++] = temp.j13
        this[c++] = temp.j14
        this[c++] = temp.j15

        this[c++] = temp.ej10
        this[c++] = temp.ej11
        this[c++] = temp.ej12
        this[c++] = temp.ej13
        this[c++] = temp.ej14
        this[c++] = temp.ej15

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

internal data class A2SConstraintData(
    var j10: Float = 0f,
    var j11: Float = 0f,
    var j12: Float = 0f,
    var j13: Float = 0f,
    var j14: Float = 0f,
    var j15: Float = 0f,

    var ej10: Float = 0f,
    var ej11: Float = 0f,
    var ej12: Float = 0f,
    var ej13: Float = 0f,
    var ej14: Float = 0f,
    var ej15: Float = 0f,

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

internal fun addA2SContact(
    bIdx: Int,
    b: ActiveBody,
    contactIdx: Int,

    nx: Float,
    ny: Float,
    nz: Float,

    error: Float,
    lambda: Float,

    data: A2SConstraintData,
    idx: Int,

    envManifolds: A2SManifoldBuffer,
    numPairsPerBody: IntArray,
) {
    val n = numPairsPerBody[bIdx]
    val im = b.inverseMass.toFloat()
    val ii = b.inverseInertia

    val pax = envManifolds.pointAX(contactIdx, idx)
    val pay = envManifolds.pointAY(contactIdx, idx)
    val paz = envManifolds.pointAZ(contactIdx, idx)

    val rax = pax - envManifolds.bodyX(contactIdx)
    val ray = pay - envManifolds.bodyY(contactIdx)
    val raz = paz - envManifolds.bodyZ(contactIdx)

    data.j10 = nx
    data.j11 = ny
    data.j12 = nz
    data.j13 = fma(ray, nz, -raz * ny)
    data.j14 = fma(raz, nx, -rax * nz)
    data.j15 = fma(rax, ny, -ray * nx)

    data.ej10 = data.j10 * im
    data.ej11 = data.j11 * im
    data.ej12 = data.j12 * im
    data.ej13 =
        fma(ii.m00.toFloat(), data.j13, fma(ii.m10.toFloat(), data.j14, ii.m20.toFloat() * data.j15))
    data.ej14 =
        fma(ii.m01.toFloat(), data.j13, fma(ii.m11.toFloat(), data.j14, ii.m21.toFloat() * data.j15))
    data.ej15 =
        fma(ii.m02.toFloat(), data.j13, fma(ii.m12.toFloat(), data.j14, ii.m22.toFloat() * data.j15))

    data.lambda = lambda
    data.den =
        1f / (
                (data.ej10 * data.j10 + data.ej11 * data.j11 + data.ej12 * data.j12 +
                 data.j13 * data.ej13 + data.j14 * data.ej14 + data.j15 * data.ej15) * n
             )

    data.error = error

    data.r1x = rax
    data.r1y = ray
    data.r1z = raz

    val depth = envManifolds.depth(contactIdx, idx)

    data.r2x = pax + depth * nx
    data.r2y = pay + depth * ny
    data.r2z = paz + depth * nz

    data.nx = nx
    data.ny = ny
    data.nz = nz
}

internal const val A2S_BASE_CONSTRAINT_BLOCK_SIZE = 9
internal const val A2S_CONSTRAINT_DATA_SIZE = 24
internal const val A2S_LAMBDA_OFFSET = 12
