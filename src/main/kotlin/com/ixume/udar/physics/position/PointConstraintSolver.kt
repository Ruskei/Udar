package com.ixume.udar.physics.position

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.ConstraintMath
import com.ixume.udar.physics.constraint.ConstraintSolver
import org.joml.Quaterniond
import org.joml.Vector3d
import java.lang.Math.fma
import kotlin.math.max
import kotlin.math.pow

class PointConstraintSolver(val parent: ConstraintSolver) {
    private var constraintData = ConstraintData(0)
    private var numConstraints = 0

    private var dt = Udar.CONFIG.timeStep.toFloat()
    private var bias = Udar.CONFIG.positionConstraint.bias
    private var slop = Udar.CONFIG.positionConstraint.slop
    private var carryover = Udar.CONFIG.positionConstraint.carryover
    private var relaxation = Udar.CONFIG.positionConstraint.relaxation
    private fun effectiveERP() =
        1f - (1.0 - Udar.CONFIG.positionConstraint.erp.toDouble()).pow(1.0 / Udar.CONFIG.collision.posIterations.toDouble())
            .toFloat()

    private var erp = effectiveERP()

    private val _tempV = Vector3d()
    private val _tempQ = Quaterniond()
    private lateinit var rawConstraints: List<PointConstraint>

    fun setup(constraints: List<PointConstraint>) {
        rawConstraints = constraints

        dt = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.positionConstraint.bias
        slop = Udar.CONFIG.positionConstraint.slop
        carryover = Udar.CONFIG.positionConstraint.carryover
        relaxation = Udar.CONFIG.positionConstraint.relaxation
        erp = effectiveERP()

        numConstraints = constraints.size

        if (constraintData.sizeFor(numConstraints) != constraintData.value.size) {
            val s = constraintData.sizeFor(numConstraints)
            constraintData = ConstraintData(s)
        }

        var i = 0
        while (i < numConstraints) {
            val constraint = constraints[i]

            val b1 = constraint.b1
            val b2 = constraint.b2

            val q1 = b1.q
            val q2 = b2.q

            val im1 = b1.inverseMass.toFloat()
            val im2 = b2.inverseMass.toFloat()

            val ii1 = b1.inverseInertia
            val ii2 = b2.inverseInertia

            q1.transform(constraint.r1x.toDouble(), constraint.r1y.toDouble(), constraint.r1z.toDouble(), _tempV)

            val r1x = _tempV.x.toFloat()
            val r1y = _tempV.y.toFloat()
            val r1z = _tempV.z.toFloat()

            val p1x = r1x + b1.pos.x.toFloat()
            val p1y = r1y + b1.pos.y.toFloat()
            val p1z = r1z + b1.pos.z.toFloat()

            q2.transform(constraint.r2x.toDouble(), constraint.r2y.toDouble(), constraint.r2z.toDouble(), _tempV)

            val r2x = _tempV.x.toFloat()
            val r2y = _tempV.y.toFloat()
            val r2z = _tempV.z.toFloat()

            val p2x = r2x + b2.pos.x.toFloat()
            val p2y = r2y + b2.pos.y.toFloat()
            val p2z = r2z + b2.pos.z.toFloat()

            val nx = p2x - p1x
            val ny = p2y - p1y
            val nz = p2z - p1z

            val bias1 = bias * nx / dt
            val bias2 = bias * ny / dt
            val bias3 = bias * nz / dt

            val j12y = -r1z
            val j12z = r1y

            val j14y = r2z
            val j14z = -r2y

            val j22x = r1z
            val j22z = -r1x

            val j24x = -r2z
            val j24z = r2x

            val j32x = -r1y
            val j32y = r1x

            val j34x = r2y
            val j34y = -r2x

            val ej12x = fma(ii1.m10.toFloat(), j12y, ii1.m20.toFloat() * j12z)
            val ej12y = fma(ii1.m11.toFloat(), j12y, ii1.m21.toFloat() * j12z)
            val ej12z = fma(ii1.m12.toFloat(), j12y, ii1.m22.toFloat() * j12z)

            val ej14x = fma(ii2.m10.toFloat(), j14y, ii2.m20.toFloat() * j14z)
            val ej14y = fma(ii2.m11.toFloat(), j14y, ii2.m21.toFloat() * j14z)
            val ej14z = fma(ii2.m12.toFloat(), j14y, ii2.m22.toFloat() * j14z)

            val ej22x = fma(ii1.m00.toFloat(), j22x, ii1.m20.toFloat() * j22z)
            val ej22y = fma(ii1.m01.toFloat(), j22x, ii1.m21.toFloat() * j22z)
            val ej22z = fma(ii1.m02.toFloat(), j22x, ii1.m22.toFloat() * j22z)

            val ej24x = fma(ii2.m00.toFloat(), j24x, ii2.m20.toFloat() * j24z)
            val ej24y = fma(ii2.m01.toFloat(), j24x, ii2.m21.toFloat() * j24z)
            val ej24z = fma(ii2.m02.toFloat(), j24x, ii2.m22.toFloat() * j24z)

            val ej32x = fma(ii1.m00.toFloat(), j32x, ii1.m10.toFloat() * j32y)
            val ej32y = fma(ii1.m01.toFloat(), j32x, ii1.m11.toFloat() * j32y)
            val ej32z = fma(ii1.m02.toFloat(), j32x, ii1.m12.toFloat() * j32y)

            val ej34x = fma(ii2.m00.toFloat(), j34x, ii2.m10.toFloat() * j34y)
            val ej34y = fma(ii2.m01.toFloat(), j34x, ii2.m11.toFloat() * j34y)
            val ej34z = fma(ii2.m02.toFloat(), j34x, ii2.m12.toFloat() * j34y)

            val k11 =
                im1 +
                fma(ej12y, j12y, ej12z * j12z) +
                im2 +
                fma(ej14y, j14y, ej14z * j14z)
            val k12 =
                fma(ej12x, j22x, ej12z * j22z) +
                fma(ej14x, j24x, ej14z * j24z)
            val k13 =
                fma(ej12x, j32x, ej12y * j32y) +
                fma(ej14x, j34x, ej14y * j34y)

            val k22 =
                im1 +
                fma(ej22x, j22x, ej22z * j22z) +
                im2 +
                fma(ej24x, j24x, ej24z * j24z)
            val k23 =
                fma(ej22x, j32x, ej22y * j32y) +
                fma(ej24x, j34x, ej24y * j34y)

            val k33 =
                im1 +
                fma(ej32x, j32x, ej32y * j32y) +
                im2 +
                fma(ej34x, j34x, ej34y * j34y)

            constraintData.set(
                cursor = i * CONSTRAINT_DATA_SIZE,
                body1Idx = b1.idx,
                body2Idx = b2.idx,

                r1x = r1x,
                r1y = r1y,
                r1z = r1z,

                r2x = r2x,
                r2y = r2y,
                r2z = r2z,

                im1 = im1,
                im2 = im2,

                e12x = ej12x,
                e12y = ej12y,
                e12z = ej12z,

                e22x = ej22x,
                e22y = ej22y,
                e22z = ej22z,

                e32x = ej32x,
                e32y = ej32y,
                e32z = ej32z,

                e14x = ej14x,
                e14y = ej14y,
                e14z = ej14z,

                e24x = ej24x,
                e24y = ej24y,
                e24z = ej24z,

                e34x = ej34x,
                e34y = ej34y,
                e34z = ej34z,

                k11 = k11,
                k12 = k12,
                k13 = k13,

                k22 = k22,
                k23 = k23,

                k33 = k33,

                b1 = bias1,
                b2 = bias2,
                b3 = bias3,

                l1 = 0f,
                l2 = 0f,
                l3 = 0f,
            )

            i++
        }
    }

    fun solveVelocity() {
        val bodyData = parent.flatBodyData
        constraintData.forEach(
            numConstraints = numConstraints,
        ) { b1Idx, b2Idx, rawIdx ->
            val v1 = bodyData[b1Idx * 6 + 0]
            val v2 = bodyData[b1Idx * 6 + 1]
            val v3 = bodyData[b1Idx * 6 + 2]
            val v4 = bodyData[b1Idx * 6 + 3]
            val v5 = bodyData[b1Idx * 6 + 4]
            val v6 = bodyData[b1Idx * 6 + 5]

            val v7 = bodyData[b2Idx * 6 + 0]
            val v8 = bodyData[b2Idx * 6 + 1]
            val v9 = bodyData[b2Idx * 6 + 2]
            val v10 = bodyData[b2Idx * 6 + 3]
            val v11 = bodyData[b2Idx * 6 + 4]
            val v12 = bodyData[b2Idx * 6 + 5]

            val b1 = constraintData[rawIdx + B_OFFSET + 0]
            val b2 = constraintData[rawIdx + B_OFFSET + 1]
            val b3 = constraintData[rawIdx + B_OFFSET + 2]

            val r1x = constraintData[rawIdx + J_OFFSET + 0]
            val r1y = constraintData[rawIdx + J_OFFSET + 1]
            val r1z = constraintData[rawIdx + J_OFFSET + 2]

            val r2x = constraintData[rawIdx + J_OFFSET + 3]
            val r2y = constraintData[rawIdx + J_OFFSET + 4]
            val r2z = constraintData[rawIdx + J_OFFSET + 5]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)

            val im1 = constraintData[rawIdx + IM_OFFSET + 0]
            val im2 = constraintData[rawIdx + IM_OFFSET + 1]

            val e12x = constraintData[rawIdx + E_OFFSET + 0]
            val e12y = constraintData[rawIdx + E_OFFSET + 1]
            val e12z = constraintData[rawIdx + E_OFFSET + 2]

            val e22x = constraintData[rawIdx + E_OFFSET + 3]
            val e22y = constraintData[rawIdx + E_OFFSET + 4]
            val e22z = constraintData[rawIdx + E_OFFSET + 5]

            val e32x = constraintData[rawIdx + E_OFFSET + 6]
            val e32y = constraintData[rawIdx + E_OFFSET + 7]
            val e32z = constraintData[rawIdx + E_OFFSET + 8]

            val e14x = constraintData[rawIdx + E_OFFSET + 9]
            val e14y = constraintData[rawIdx + E_OFFSET + 10]
            val e14z = constraintData[rawIdx + E_OFFSET + 11]

            val e24x = constraintData[rawIdx + E_OFFSET + 12]
            val e24y = constraintData[rawIdx + E_OFFSET + 13]
            val e24z = constraintData[rawIdx + E_OFFSET + 14]

            val e34x = constraintData[rawIdx + E_OFFSET + 15]
            val e34y = constraintData[rawIdx + E_OFFSET + 16]
            val e34z = constraintData[rawIdx + E_OFFSET + 17]

            val k11 = constraintData[rawIdx + K_OFFSET + 0]
            val k12 = constraintData[rawIdx + K_OFFSET + 1]
            val k13 = constraintData[rawIdx + K_OFFSET + 2]

            val k22 = constraintData[rawIdx + K_OFFSET + 3]
            val k23 = constraintData[rawIdx + K_OFFSET + 4]

            val k33 = constraintData[rawIdx + K_OFFSET + 5]

            val t1 = constraintData[rawIdx + L_OFFSET + 0]
            val t2 = constraintData[rawIdx + L_OFFSET + 1]
            val t3 = constraintData[rawIdx + L_OFFSET + 2]

            val l1: Float
            val l2: Float
            val l3: Float

            ConstraintMath.solveSymmetric3x3(
                k11, k12, k13,
                k22, k23,
                k33,

                jv1, jv2, jv3,
            ) { s1, s2, s3 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
            }

            constraintData[rawIdx + L_OFFSET + 0] = l1
            constraintData[rawIdx + L_OFFSET + 1] = l2
            constraintData[rawIdx + L_OFFSET + 2] = l3

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3

            bodyData[b1Idx * 6 + 0] += -im1 * d1
            bodyData[b1Idx * 6 + 1] += -im1 * d2
            bodyData[b1Idx * 6 + 2] += -im1 * d3
            bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, e32x * d3))
            bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, e32y * d3))
            bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, e32z * d3))

            bodyData[b2Idx * 6 + 0] += im2 * d1
            bodyData[b2Idx * 6 + 1] += im2 * d2
            bodyData[b2Idx * 6 + 2] += im2 * d3
            bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, e34x * d3))
            bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, e34y * d3))
            bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, e34z * d3))
        }
    }

    fun solvePosition() {
        var i = 0
        constraintData.forEach(
            numConstraints = numConstraints,
        ) { _, _, rawIdx ->
            val constraint = rawConstraints[i]

            val b1 = constraint.b1
            val b2 = constraint.b2

            val q1 = b1.q
            val q2 = b2.q

            val im1 = b1.inverseMass.toFloat()
            val im2 = b2.inverseMass.toFloat()

            val ii1 = b1.inverseInertia
            val ii2 = b2.inverseInertia

            q1.transform(constraint.r1x.toDouble(), constraint.r1y.toDouble(), constraint.r1z.toDouble(), _tempV)

            val r1x = _tempV.x.toFloat()
            val r1y = _tempV.y.toFloat()
            val r1z = _tempV.z.toFloat()

            val p1x = r1x + b1.pos.x.toFloat()
            val p1y = r1y + b1.pos.y.toFloat()
            val p1z = r1z + b1.pos.z.toFloat()

            q2.transform(constraint.r2x.toDouble(), constraint.r2y.toDouble(), constraint.r2z.toDouble(), _tempV)

            val r2x = _tempV.x.toFloat()
            val r2y = _tempV.y.toFloat()
            val r2z = _tempV.z.toFloat()

            val p2x = r2x + b2.pos.x.toFloat()
            val p2y = r2y + b2.pos.y.toFloat()
            val p2z = r2z + b2.pos.z.toFloat()

            val nx = p2x - p1x
            val ny = p2y - p1y
            val nz = p2z - p1z

            val e1 = nx
            val e2 = ny
            val e3 = nz

            val j12y = -r1z
            val j12z = r1y

            val j14y = r2z
            val j14z = -r2y

            val j22x = r1z
            val j22z = -r1x

            val j24x = -r2z
            val j24z = r2x

            val j32x = -r1y
            val j32y = r1x

            val j34x = r2y
            val j34y = -r2x

            val ej12x = fma(ii1.m10.toFloat(), j12y, ii1.m20.toFloat() * j12z)
            val ej12y = fma(ii1.m11.toFloat(), j12y, ii1.m21.toFloat() * j12z)
            val ej12z = fma(ii1.m12.toFloat(), j12y, ii1.m22.toFloat() * j12z)

            val ej14x = fma(ii2.m10.toFloat(), j14y, ii2.m20.toFloat() * j14z)
            val ej14y = fma(ii2.m11.toFloat(), j14y, ii2.m21.toFloat() * j14z)
            val ej14z = fma(ii2.m12.toFloat(), j14y, ii2.m22.toFloat() * j14z)

            val ej22x = fma(ii1.m00.toFloat(), j22x, ii1.m20.toFloat() * j22z)
            val ej22y = fma(ii1.m01.toFloat(), j22x, ii1.m21.toFloat() * j22z)
            val ej22z = fma(ii1.m02.toFloat(), j22x, ii1.m22.toFloat() * j22z)

            val ej24x = fma(ii2.m00.toFloat(), j24x, ii2.m20.toFloat() * j24z)
            val ej24y = fma(ii2.m01.toFloat(), j24x, ii2.m21.toFloat() * j24z)
            val ej24z = fma(ii2.m02.toFloat(), j24x, ii2.m22.toFloat() * j24z)

            val ej32x = fma(ii1.m00.toFloat(), j32x, ii1.m10.toFloat() * j32y)
            val ej32y = fma(ii1.m01.toFloat(), j32x, ii1.m11.toFloat() * j32y)
            val ej32z = fma(ii1.m02.toFloat(), j32x, ii1.m12.toFloat() * j32y)

            val ej34x = fma(ii2.m00.toFloat(), j34x, ii2.m10.toFloat() * j34y)
            val ej34y = fma(ii2.m01.toFloat(), j34x, ii2.m11.toFloat() * j34y)
            val ej34z = fma(ii2.m02.toFloat(), j34x, ii2.m12.toFloat() * j34y)

            val k11 =
                im1 +
                fma(ej12y, j12y, ej12z * j12z) +
                im2 +
                fma(ej14y, j14y, ej14z * j14z)
            val k12 =
                fma(ej12x, j22x, ej12z * j22z) +
                fma(ej14x, j24x, ej14z * j24z)
            val k13 =
                fma(ej12x, j32x, ej12y * j32y) +
                fma(ej14x, j34x, ej14y * j34y)

            val k22 =
                im1 +
                fma(ej22x, j22x, ej22z * j22z) +
                im2 +
                fma(ej24x, j24x, ej24z * j24z)
            val k23 =
                fma(ej22x, j32x, ej22y * j32y) +
                fma(ej24x, j34x, ej24y * j34y)

            val k33 =
                im1 +
                fma(ej32x, j32x, ej32y * j32y) +
                im2 +
                fma(ej34x, j34x, ej34y * j34y)

            val l1: Float
            val l2: Float
            val l3: Float

            ConstraintMath.solveSymmetric3x3(
                k11, k12, k13,
                k22, k23,
                k33,

                -e1, -e2, -e3,
            ) { s1, s2, s3 ->
                l1 = s1
                l2 = s2
                l3 = s3
            }

            val d1 = -im1 * l1
            val d2 = -im1 * l2
            val d3 = -im1 * l3
            val d4 = fma(ej12x, l1, fma(ej22x, l2, ej32x * l3))
            val d5 = fma(ej12y, l1, fma(ej22y, l2, ej32y * l3))
            val d6 = fma(ej12z, l1, fma(ej22z, l2, ej32z * l3))

            val d7 = im2 * l1
            val d8 = im2 * l2
            val d9 = im2 * l3
            val d10 = fma(ej14x, l1, fma(ej24x, l2, ej34x * l3))
            val d11 = fma(ej14y, l1, fma(ej24y, l2, ej34y * l3))
            val d12 = fma(ej14z, l1, fma(ej24z, l2, ej34z * l3))

            b1.pos.add(
                erp * d1.toDouble(),
                erp * d2.toDouble(),
                erp * d3.toDouble(),
            )

            _tempQ.set(q1).conjugate().transform(
                erp * d4.toDouble(),
                erp * d5.toDouble(),
                erp * d6.toDouble(),
                _tempV
            )

            _tempQ.set(q1).mul(_tempV.x, _tempV.y, _tempV.z, 0.0).mul(0.5)

            q1.add(_tempQ).normalize()

            b2.pos.add(
                erp * d7.toDouble(),
                erp * d8.toDouble(),
                erp * d9.toDouble(),
            )

            _tempQ.set(q2).conjugate().transform(
                erp * d10.toDouble(),
                erp * d11.toDouble(),
                erp * d12.toDouble(),
                _tempV
            )

            _tempQ.set(q2).mul(_tempV.x, _tempV.y, _tempV.z, 0.0).mul(0.5)

            q2.add(_tempQ).normalize()

            i++
        }
    }
}

@JvmInline
private value class ConstraintData(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * CONSTRAINT_DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        im1: Float,
        im2: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e34x: Float,
        e34y: Float,
        e34z: Float,

        k11: Float,
        k12: Float,
        k13: Float,

        k22: Float,
        k23: Float,

        k33: Float,

        b1: Float,
        b2: Float,
        b3: Float,

        l1: Float,
        l2: Float,
        l3: Float,
    ) {
        this[cursor + 0] = Float.fromBits(body1Idx)
        this[cursor + 1] = Float.fromBits(body2Idx)

        this[cursor + 2] = r1x
        this[cursor + 3] = r1y
        this[cursor + 4] = r1z

        this[cursor + 5] = r2x
        this[cursor + 6] = r2y
        this[cursor + 7] = r2z

        this[cursor + 8] = im1
        this[cursor + 9] = im2

        this[cursor + 10] = e12x
        this[cursor + 11] = e12y
        this[cursor + 12] = e12z

        this[cursor + 13] = e22x
        this[cursor + 14] = e22y
        this[cursor + 15] = e22z

        this[cursor + 16] = e32x
        this[cursor + 17] = e32y
        this[cursor + 18] = e32z

        this[cursor + 19] = e14x
        this[cursor + 20] = e14y
        this[cursor + 21] = e14z

        this[cursor + 22] = e24x
        this[cursor + 23] = e24y
        this[cursor + 24] = e24z

        this[cursor + 25] = e34x
        this[cursor + 26] = e34y
        this[cursor + 27] = e34z

        this[cursor + 28] = k11
        this[cursor + 29] = k12
        this[cursor + 30] = k13

        this[cursor + 31] = k22
        this[cursor + 32] = k23

        this[cursor + 33] = k33

        this[cursor + 34] = b1
        this[cursor + 35] = b2
        this[cursor + 36] = b3

        this[cursor + 37] = l1
        this[cursor + 38] = l2
        this[cursor + 39] = l3
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * CONSTRAINT_DATA_SIZE + 0].toRawBits(),
                this[i * CONSTRAINT_DATA_SIZE + 1].toRawBits(),
                i * CONSTRAINT_DATA_SIZE,
            )

            i++
        }
    }
}

private const val CONSTRAINT_DATA_SIZE = 40
private const val J_OFFSET = 2
private const val IM_OFFSET = 8
private const val E_OFFSET = 10
private const val K_OFFSET = 28
private const val B_OFFSET = 34
private const val L_OFFSET = 37
