package com.ixume.udar.physics.constraint

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.constraint.MatrixMath.solveSymmetric3x3
import com.ixume.udar.physics.constraint.MatrixMath.solveSymmetric4x4
import com.ixume.udar.physics.constraint.MatrixMath.solveSymmetric5x5
import com.ixume.udar.physics.constraint.MatrixMath.solveSymmetric6x6
import com.ixume.udar.physics.constraint.QuatMath.quatMul
import com.ixume.udar.physics.constraint.QuatMath.quatTransform
import com.ixume.udar.physics.constraint.QuatMath.safeNormalize
import com.ixume.udar.physics.constraint.QuatMath.transform
import java.lang.Math.fma
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
import kotlin.math.abs

@OptIn(ExperimentalContracts::class)
object ConstraintMath {
    inline fun addData(
        b1: ActiveBody,
        b2: ActiveBody,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        nx: Float,
        ny: Float,
        nz: Float,

        bias: Float,
        lambda: Float,

        set: (
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
        ) -> Unit,
    ) {
        contract {
            callsInPlace(set, InvocationKind.EXACTLY_ONCE)
        }

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        val j10 = nx
        val j11 = ny
        val j12 = nz
        val j13 = fma(r1y, nz, -r1z * ny)
        val j14 = fma(r1z, nx, -r1x * nz)
        val j15 = fma(r1x, ny, -r1y * nx)

        val j20 = -nx
        val j21 = -ny
        val j22 = -nz
        val j23 = -fma(r2y, nz, -r2z * ny)
        val j24 = -fma(r2z, nx, -r2x * nz)
        val j25 = -fma(r2x, ny, -r2y * nx)

        val ej10 = nx * im1
        val ej11 = ny * im1
        val ej12 = nz * im1
        val ej13 = fma(ii1.m00.toFloat(), j13, fma(ii1.m10.toFloat(), j14, ii1.m20.toFloat() * j15))
        val ej14 = fma(ii1.m01.toFloat(), j13, fma(ii1.m11.toFloat(), j14, ii1.m21.toFloat() * j15))
        val ej15 = fma(ii1.m02.toFloat(), j13, fma(ii1.m12.toFloat(), j14, ii1.m22.toFloat() * j15))

        val ej20 = -nx * im2
        val ej21 = -ny * im2
        val ej22 = -nz * im2
        val ej23 = fma(ii2.m00.toFloat(), j23, fma(ii2.m10.toFloat(), j24, ii2.m20.toFloat() * j25))
        val ej24 = fma(ii2.m01.toFloat(), j23, fma(ii2.m11.toFloat(), j24, ii2.m21.toFloat() * j25))
        val ej25 = fma(ii2.m02.toFloat(), j23, fma(ii2.m12.toFloat(), j24, ii2.m22.toFloat() * j25))

        var iden =
            ej10 * j10 + ej11 * j11 + ej12 * j12 +
            j13 * ej13 + j14 * ej14 + j15 * ej15 +
            ej20 * j20 + ej21 * j21 + ej22 * j22 +
            j23 * ej23 + j24 * ej24 + j25 * ej25

        iden = if (abs(iden) < 1e-6) 0f else (1f / iden)

        set(
            j10,
            j11,
            j12,
            j13,
            j14,
            j15,

            j23,
            j24,
            j25,

            ej13,
            ej14,
            ej15,

            ej23,
            ej24,
            ej25,

            lambda,
            iden,
            bias,
        )
    }


    fun solve3p0rVelocity(
        parent: ConstraintSolver,
        constraintData: ConstraintData3p0r,
        numConstraints: Int,
        relaxation: Float = 1f,
    ) {
        val bodyData = parent.flatBodyData

        val bOffset = ConstraintData3p0r.B_OFFSET
        val jOffset = ConstraintData3p0r.J_OFFSET
        val imOffset = ConstraintData3p0r.IM_OFFSET
        val eOffset = ConstraintData3p0r.E_OFFSET
        val kOffset = ConstraintData3p0r.K_OFFSET
        val lOffset = ConstraintData3p0r.L_OFFSET

        constraintData.forEach(numConstraints) { _, b1Idx, b2Idx, rawIdx ->
            check(b1Idx != -1)
            check(b2Idx != -1)
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

            val b1 = constraintData[rawIdx + bOffset + 0]
            val b2 = constraintData[rawIdx + bOffset + 1]
            val b3 = constraintData[rawIdx + bOffset + 2]

            val r1x = constraintData[rawIdx + jOffset + 0]
            val r1y = constraintData[rawIdx + jOffset + 1]
            val r1z = constraintData[rawIdx + jOffset + 2]

            val r2x = constraintData[rawIdx + jOffset + 3]
            val r2y = constraintData[rawIdx + jOffset + 4]
            val r2z = constraintData[rawIdx + jOffset + 5]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)

            val im1 = constraintData[rawIdx + imOffset + 0]
            val im2 = constraintData[rawIdx + imOffset + 1]

            val e12x = constraintData[rawIdx + eOffset + 0]
            val e12y = constraintData[rawIdx + eOffset + 1]
            val e12z = constraintData[rawIdx + eOffset + 2]

            val e22x = constraintData[rawIdx + eOffset + 3]
            val e22y = constraintData[rawIdx + eOffset + 4]
            val e22z = constraintData[rawIdx + eOffset + 5]

            val e32x = constraintData[rawIdx + eOffset + 6]
            val e32y = constraintData[rawIdx + eOffset + 7]
            val e32z = constraintData[rawIdx + eOffset + 8]

            val e14x = constraintData[rawIdx + eOffset + 9]
            val e14y = constraintData[rawIdx + eOffset + 10]
            val e14z = constraintData[rawIdx + eOffset + 11]

            val e24x = constraintData[rawIdx + eOffset + 12]
            val e24y = constraintData[rawIdx + eOffset + 13]
            val e24z = constraintData[rawIdx + eOffset + 14]

            val e34x = constraintData[rawIdx + eOffset + 15]
            val e34y = constraintData[rawIdx + eOffset + 16]
            val e34z = constraintData[rawIdx + eOffset + 17]

            val k11 = constraintData[rawIdx + kOffset + 0]
            val k12 = constraintData[rawIdx + kOffset + 1]
            val k13 = constraintData[rawIdx + kOffset + 2]

            val k22 = constraintData[rawIdx + kOffset + 3]
            val k23 = constraintData[rawIdx + kOffset + 4]

            val k33 = constraintData[rawIdx + kOffset + 5]

            val t1 = constraintData[rawIdx + lOffset + 0]
            val t2 = constraintData[rawIdx + lOffset + 1]
            val t3 = constraintData[rawIdx + lOffset + 2]

            val l1: Float
            val l2: Float
            val l3: Float

            solveSymmetric3x3(
                k11, k12, k13,
                k22, k23,
                k33,

                jv1, jv2, jv3,
            ) { s1, s2, s3 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
            }

            constraintData[rawIdx + lOffset + 0] = l1
            constraintData[rawIdx + lOffset + 1] = l2
            constraintData[rawIdx + lOffset + 2] = l3

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3

            bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
            bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
            bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
            bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, e32x * d3)) * relaxation
            bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, e32y * d3)) * relaxation
            bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, e32z * d3)) * relaxation

            bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
            bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
            bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
            bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, e34x * d3)) * relaxation
            bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, e34y * d3)) * relaxation
            bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, e34z * d3)) * relaxation
        }
    }

    inline fun solve3p1rVelocity(
        parent: ConstraintSolver,
        constraintData: ConstraintData3p1r,
        numConstraints: Int,
        relaxation: Float = 1f,

        l4Validator: (l: Float, i: Int) -> Boolean = { f, _ -> f >= INVALID_LAMBDA },
    ) {
        val bodyData = parent.flatBodyData
        val debugData = parent.debugData

        val bOffset = ConstraintData3p1r.B_OFFSET
        val jOffset = ConstraintData3p1r.J_OFFSET
        val imOffset = ConstraintData3p1r.IM_OFFSET
        val eOffset = ConstraintData3p1r.E_OFFSET
        val kOffset = ConstraintData3p1r.K_OFFSET
        val lOffset = ConstraintData3p1r.L_OFFSET

        constraintData.forEach(numConstraints) { _, b1Idx, b2Idx, rawIdx ->
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

            val b1 = constraintData[rawIdx + bOffset + 0]
            val b2 = constraintData[rawIdx + bOffset + 1]
            val b3 = constraintData[rawIdx + bOffset + 2]
            val b4 = constraintData[rawIdx + bOffset + 3]

            val r1x = constraintData[rawIdx + jOffset + 0]
            val r1y = constraintData[rawIdx + jOffset + 1]
            val r1z = constraintData[rawIdx + jOffset + 2]

            val r2x = constraintData[rawIdx + jOffset + 3]
            val r2y = constraintData[rawIdx + jOffset + 4]
            val r2z = constraintData[rawIdx + jOffset + 5]

            val j42x = constraintData[rawIdx + jOffset + 6]
            val j42y = constraintData[rawIdx + jOffset + 7]
            val j42z = constraintData[rawIdx + jOffset + 8]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)
            val jv4 =
                -fma(j42x, v4, fma(j42y, v5, fma(j42z, v6, fma(-j42x, v10, fma(-j42y, v11, fma(-j42z, v12, b4))))))

            val im1 = constraintData[rawIdx + imOffset + 0]
            val im2 = constraintData[rawIdx + imOffset + 1]

            val e12x = constraintData[rawIdx + eOffset + 0]
            val e12y = constraintData[rawIdx + eOffset + 1]
            val e12z = constraintData[rawIdx + eOffset + 2]

            val e22x = constraintData[rawIdx + eOffset + 3]
            val e22y = constraintData[rawIdx + eOffset + 4]
            val e22z = constraintData[rawIdx + eOffset + 5]

            val e32x = constraintData[rawIdx + eOffset + 6]
            val e32y = constraintData[rawIdx + eOffset + 7]
            val e32z = constraintData[rawIdx + eOffset + 8]

            val e42x = constraintData[rawIdx + eOffset + 9]
            val e42y = constraintData[rawIdx + eOffset + 10]
            val e42z = constraintData[rawIdx + eOffset + 11]

            val e14x = constraintData[rawIdx + eOffset + 12]
            val e14y = constraintData[rawIdx + eOffset + 13]
            val e14z = constraintData[rawIdx + eOffset + 14]

            val e24x = constraintData[rawIdx + eOffset + 15]
            val e24y = constraintData[rawIdx + eOffset + 16]
            val e24z = constraintData[rawIdx + eOffset + 17]

            val e34x = constraintData[rawIdx + eOffset + 18]
            val e34y = constraintData[rawIdx + eOffset + 19]
            val e34z = constraintData[rawIdx + eOffset + 20]

            val e44x = constraintData[rawIdx + eOffset + 21]
            val e44y = constraintData[rawIdx + eOffset + 22]
            val e44z = constraintData[rawIdx + eOffset + 23]

            val k11 = constraintData[rawIdx + kOffset + 0]
            val k12 = constraintData[rawIdx + kOffset + 1]
            val k13 = constraintData[rawIdx + kOffset + 2]
            val k14 = constraintData[rawIdx + kOffset + 3]

            val k22 = constraintData[rawIdx + kOffset + 4]
            val k23 = constraintData[rawIdx + kOffset + 5]
            val k24 = constraintData[rawIdx + kOffset + 6]

            val k33 = constraintData[rawIdx + kOffset + 7]
            val k34 = constraintData[rawIdx + kOffset + 8]

            val k44 = constraintData[rawIdx + kOffset + 9]

            val t1 = constraintData[rawIdx + lOffset + 0]
            val t2 = constraintData[rawIdx + lOffset + 1]
            val t3 = constraintData[rawIdx + lOffset + 2]
            val t4 = constraintData[rawIdx + lOffset + 3]

            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float

            solveSymmetric4x4(
                k11, k12, k13, k14,
                k22, k23, k24,
                k33, k34,
                k44,

                jv1, jv2, jv3, jv4,
            ) { s1, s2, s3, s4 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
                l4 = t4 + s4
            }

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3
            val d4 = l4 - t4

            val valid4 = l4Validator(d4, rawIdx + lOffset + 3)

            if (!valid4) {
                val fl1: Float
                val fl2: Float
                val fl3: Float
                solveSymmetric3x3(
                    k11, k12, k13,
                    k22, k23,
                    k33,

                    jv1, jv2, jv3,
                ) { s1, s2, s3 ->
                    fl1 = t1 + s1
                    fl2 = t2 + s2
                    fl3 = t3 + s3
                }

                val nl1 = fl1 - t1
                val nl2 = fl2 - t2
                val nl3 = fl3 - t3

                debugData += nl1
                debugData += nl2
                debugData += nl3

                constraintData[rawIdx + lOffset + 0] = fl1
                constraintData[rawIdx + lOffset + 1] = fl2
                constraintData[rawIdx + lOffset + 2] = fl3

                bodyData[b1Idx * 6 + 0] += -im1 * nl1 * relaxation
                bodyData[b1Idx * 6 + 1] += -im1 * nl2 * relaxation
                bodyData[b1Idx * 6 + 2] += -im1 * nl3 * relaxation
                bodyData[b1Idx * 6 + 3] += fma(e12x, nl1, fma(e22x, nl2, e32x * nl3)) * relaxation
                bodyData[b1Idx * 6 + 4] += fma(e12y, nl1, fma(e22y, nl2, e32y * nl3)) * relaxation
                bodyData[b1Idx * 6 + 5] += fma(e12z, nl1, fma(e22z, nl2, e32z * nl3)) * relaxation

                bodyData[b2Idx * 6 + 0] += im2 * nl1 * relaxation
                bodyData[b2Idx * 6 + 1] += im2 * nl2 * relaxation
                bodyData[b2Idx * 6 + 2] += im2 * nl3 * relaxation
                bodyData[b2Idx * 6 + 3] += fma(e14x, nl1, fma(e24x, nl2, e34x * nl3)) * relaxation
                bodyData[b2Idx * 6 + 4] += fma(e14y, nl1, fma(e24y, nl2, e34y * nl3)) * relaxation
                bodyData[b2Idx * 6 + 5] += fma(e14z, nl1, fma(e24z, nl2, e34z * nl3)) * relaxation
            } else {
                constraintData[rawIdx + lOffset + 0] = l1
                constraintData[rawIdx + lOffset + 1] = l2
                constraintData[rawIdx + lOffset + 2] = l3
                constraintData[rawIdx + lOffset + 3] = l4

                debugData += d1
                debugData += d2
                debugData += d3
                debugData += d4

                bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, fma(e32x, d3, e42x * d4))) * relaxation
                bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, fma(e32y, d3, e42y * d4))) * relaxation
                bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, fma(e32z, d3, e42z * d4))) * relaxation

                bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, fma(e34x, d3, e44x * d4))) * relaxation
                bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, fma(e34y, d3, e44y * d4))) * relaxation
                bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, fma(e34z, d3, e44z * d4))) * relaxation
            }
        }
    }

    inline fun solve3p2rVelocity(
        parent: ConstraintSolver,
        constraintData: ConstraintData3p2r,
        numConstraints: Int,
        relaxation: Float = 1f,

        l4Validator: (l: Float, i: Int) -> Boolean = { f, _ -> f >= INVALID_LAMBDA },
        l5Validator: (l: Float, i: Int) -> Boolean = { f, _ -> f >= INVALID_LAMBDA },
    ) {
        val bodyData = parent.flatBodyData
        val debugData = parent.debugData

        val bOffset = ConstraintData3p2r.B_OFFSET
        val jOffset = ConstraintData3p2r.J_OFFSET
        val imOffset = ConstraintData3p2r.IM_OFFSET
        val eOffset = ConstraintData3p2r.E_OFFSET
        val kOffset = ConstraintData3p2r.K_OFFSET
        val lOffset = ConstraintData3p2r.L_OFFSET

        constraintData.forEach(numConstraints) { _, b1Idx, b2Idx, rawIdx ->
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

            val b1 = constraintData[rawIdx + bOffset + 0]
            val b2 = constraintData[rawIdx + bOffset + 1]
            val b3 = constraintData[rawIdx + bOffset + 2]
            val b4 = constraintData[rawIdx + bOffset + 3]
            val b5 = constraintData[rawIdx + bOffset + 4]

            val r1x = constraintData[rawIdx + jOffset + 0]
            val r1y = constraintData[rawIdx + jOffset + 1]
            val r1z = constraintData[rawIdx + jOffset + 2]

            val r2x = constraintData[rawIdx + jOffset + 3]
            val r2y = constraintData[rawIdx + jOffset + 4]
            val r2z = constraintData[rawIdx + jOffset + 5]

            val j42x = constraintData[rawIdx + jOffset + 6]
            val j42y = constraintData[rawIdx + jOffset + 7]
            val j42z = constraintData[rawIdx + jOffset + 8]

            val j52x = constraintData[rawIdx + jOffset + 9]
            val j52y = constraintData[rawIdx + jOffset + 10]
            val j52z = constraintData[rawIdx + jOffset + 11]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)
            val jv4 =
                -fma(j42x, v4, fma(j42y, v5, fma(j42z, v6, fma(-j42x, v10, fma(-j42y, v11, fma(-j42z, v12, b4))))))
            val jv5 =
                -fma(j52x, v4, fma(j52y, v5, fma(j52z, v6, fma(-j52x, v10, fma(-j52y, v11, fma(-j52z, v12, b5))))))

            val im1 = constraintData[rawIdx + imOffset + 0]
            val im2 = constraintData[rawIdx + imOffset + 1]

            val e12x = constraintData[rawIdx + eOffset + 0]
            val e12y = constraintData[rawIdx + eOffset + 1]
            val e12z = constraintData[rawIdx + eOffset + 2]

            val e22x = constraintData[rawIdx + eOffset + 3]
            val e22y = constraintData[rawIdx + eOffset + 4]
            val e22z = constraintData[rawIdx + eOffset + 5]

            val e32x = constraintData[rawIdx + eOffset + 6]
            val e32y = constraintData[rawIdx + eOffset + 7]
            val e32z = constraintData[rawIdx + eOffset + 8]

            val e42x = constraintData[rawIdx + eOffset + 9]
            val e42y = constraintData[rawIdx + eOffset + 10]
            val e42z = constraintData[rawIdx + eOffset + 11]

            val e52x = constraintData[rawIdx + eOffset + 12]
            val e52y = constraintData[rawIdx + eOffset + 13]
            val e52z = constraintData[rawIdx + eOffset + 14]

            val e14x = constraintData[rawIdx + eOffset + 15]
            val e14y = constraintData[rawIdx + eOffset + 16]
            val e14z = constraintData[rawIdx + eOffset + 17]

            val e24x = constraintData[rawIdx + eOffset + 18]
            val e24y = constraintData[rawIdx + eOffset + 19]
            val e24z = constraintData[rawIdx + eOffset + 20]

            val e34x = constraintData[rawIdx + eOffset + 21]
            val e34y = constraintData[rawIdx + eOffset + 22]
            val e34z = constraintData[rawIdx + eOffset + 23]

            val e44x = constraintData[rawIdx + eOffset + 24]
            val e44y = constraintData[rawIdx + eOffset + 25]
            val e44z = constraintData[rawIdx + eOffset + 26]

            val e54x = constraintData[rawIdx + eOffset + 27]
            val e54y = constraintData[rawIdx + eOffset + 28]
            val e54z = constraintData[rawIdx + eOffset + 29]

            val k11 = constraintData[rawIdx + kOffset + 0]
            val k12 = constraintData[rawIdx + kOffset + 1]
            val k13 = constraintData[rawIdx + kOffset + 2]
            val k14 = constraintData[rawIdx + kOffset + 3]
            val k15 = constraintData[rawIdx + kOffset + 4]

            val k22 = constraintData[rawIdx + kOffset + 5]
            val k23 = constraintData[rawIdx + kOffset + 6]
            val k24 = constraintData[rawIdx + kOffset + 7]
            val k25 = constraintData[rawIdx + kOffset + 8]

            val k33 = constraintData[rawIdx + kOffset + 9]
            val k34 = constraintData[rawIdx + kOffset + 10]
            val k35 = constraintData[rawIdx + kOffset + 11]

            val k44 = constraintData[rawIdx + kOffset + 12]
            val k45 = constraintData[rawIdx + kOffset + 13]

            val k55 = constraintData[rawIdx + kOffset + 14]

            val t1 = constraintData[rawIdx + lOffset + 0]
            val t2 = constraintData[rawIdx + lOffset + 1]
            val t3 = constraintData[rawIdx + lOffset + 2]
            val t4 = constraintData[rawIdx + lOffset + 3]
            val t5 = constraintData[rawIdx + lOffset + 4]

            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float
            val l5: Float

            solveSymmetric5x5(
                k11, k12, k13, k14, k15,
                k22, k23, k24, k25,
                k33, k34, k35,
                k44, k45,
                k55,

                jv1, jv2, jv3, jv4, jv5,
            ) { s1, s2, s3, s4, s5 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
                l4 = t4 + s4
                l5 = t5 + s5
            }

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3
            val d4 = l4 - t4
            val d5 = l5 - t5

            val valid4 = l4Validator(d4, rawIdx + lOffset + 3)
            val valid5 = l5Validator(d5, rawIdx + lOffset + 4)

            if (valid4 != valid5) {
                val jv4r: Float
                val t4r: Float
                val k14r: Float
                val k24r: Float
                val k34r: Float
                val k44r: Float
                val l4or: Int
                val e42xr: Float
                val e42yr: Float
                val e42zr: Float
                val e44xr: Float
                val e44yr: Float
                val e44zr: Float
                val b4r: Float
                if (valid4) {
                    jv4r = jv4
                    t4r = t4
                    k14r = k14
                    k24r = k24
                    k34r = k34
                    k44r = k44
                    l4or = 3
                    e42xr = e42x
                    e42yr = e42y
                    e42zr = e42z
                    e44xr = e44x
                    e44yr = e44y
                    e44zr = e44z
                    b4r = b4
                } else {
                    jv4r = jv5
                    t4r = t5
                    k14r = k15
                    k24r = k25
                    k34r = k35
                    k44r = k55
                    l4or = 4
                    e42xr = e52x
                    e42yr = e52y
                    e42zr = e52z
                    e44xr = e54x
                    e44yr = e54y
                    e44zr = e54z
                    b4r = b5
                }
                //one valid
                val l1: Float
                val l2: Float
                val l3: Float
                val l4: Float

                solveSymmetric4x4(
                    k11, k12, k13, k14r,
                    k22, k23, k24r,
                    k33, k34r,
                    k44r,

                    jv1, jv2, jv3, jv4r,
                ) { s1, s2, s3, s4 ->
                    l1 = t1 + s1
                    l2 = t2 + s2
                    l3 = t3 + s3
                    l4 = t4r + s4
                }

                val d1 = l1 - t1
                val d2 = l2 - t2
                val d3 = l3 - t3
                val d4 = l4 - t4r

                debugData += d1
                debugData += d2
                debugData += d3
                debugData += d4

                constraintData[rawIdx + lOffset + 0] = l1
                constraintData[rawIdx + lOffset + 1] = l2
                constraintData[rawIdx + lOffset + 2] = l3
                constraintData[rawIdx + lOffset + l4or] = l4

                bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, fma(e32x, d3, e42xr * d4))) * relaxation
                bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, fma(e32y, d3, e42yr * d4))) * relaxation
                bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, fma(e32z, d3, e42zr * d4))) * relaxation

                bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, fma(e34x, d3, e44xr * d4))) * relaxation
                bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, fma(e34y, d3, e44yr * d4))) * relaxation
                bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, fma(e34z, d3, e44zr * d4))) * relaxation
            } else if (valid4) {
                //both valid
                constraintData[rawIdx + lOffset + 0] = l1
                constraintData[rawIdx + lOffset + 1] = l2
                constraintData[rawIdx + lOffset + 2] = l3
                constraintData[rawIdx + lOffset + 3] = l4
                constraintData[rawIdx + lOffset + 4] = l5

                debugData += d1
                debugData += d2
                debugData += d3
                debugData += d4
                debugData += d5

                bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, fma(e32x, d3, fma(e42x, d4, e52x * d5)))) * relaxation
                bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, fma(e32y, d3, fma(e42y, d4, e52y * d5)))) * relaxation
                bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, fma(e32z, d3, fma(e42z, d4, e52z * d5)))) * relaxation

                bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, fma(e34x, d3, fma(e44x, d4, e54x * d5)))) * relaxation
                bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, fma(e34y, d3, fma(e44y, d4, e54y * d5)))) * relaxation
                bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, fma(e34z, d3, fma(e44z, d4, e54z * d5)))) * relaxation
            } else {
                //both invalid
                val l1: Float
                val l2: Float
                val l3: Float

                solveSymmetric3x3(
                    k11, k12, k13,
                    k22, k23,
                    k33,

                    jv1, jv2, jv3,
                ) { s1, s2, s3 ->
                    l1 = t1 + s1
                    l2 = t2 + s2
                    l3 = t3 + s3
                }

                val d1 = l1 - t1
                val d2 = l2 - t2
                val d3 = l3 - t3

                debugData += d1
                debugData += d2
                debugData += d3

                constraintData[rawIdx + lOffset + 0] = l1
                constraintData[rawIdx + lOffset + 1] = l2
                constraintData[rawIdx + lOffset + 2] = l3

                bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, e32x * d3)) * relaxation
                bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, e32y * d3)) * relaxation
                bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, e32z * d3)) * relaxation

                bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, e34x * d3)) * relaxation
                bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, e34y * d3)) * relaxation
                bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, e34z * d3)) * relaxation
            }
        }
    }

    inline fun solve3p3rVelocity(
        parent: ConstraintSolver,
        constraintData: ConstraintData3p3r,
        numConstraints: Int,
        relaxation: Float = 1f,

        l4Validator: (l: Float, i: Int) -> Boolean = { f, _ -> f >= INVALID_LAMBDA },
        l5Validator: (l: Float, i: Int) -> Boolean = { f, _ -> f >= INVALID_LAMBDA },
        l6Validator: (l: Float, i: Int) -> Boolean = { f, _ -> f >= INVALID_LAMBDA },
    ) {
        val bodyData = parent.flatBodyData
        val debugData = parent.debugData

        val bOffset = ConstraintData3p3r.B_OFFSET
        val jOffset = ConstraintData3p3r.J_OFFSET
        val imOffset = ConstraintData3p3r.IM_OFFSET
        val eOffset = ConstraintData3p3r.E_OFFSET
        val kOffset = ConstraintData3p3r.K_OFFSET
        val lOffset = ConstraintData3p3r.L_OFFSET

        constraintData.forEach(numConstraints) { _, b1Idx, b2Idx, rawIdx ->
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

            val b1 = constraintData[rawIdx + bOffset + 0]
            val b2 = constraintData[rawIdx + bOffset + 1]
            val b3 = constraintData[rawIdx + bOffset + 2]
            val b4 = constraintData[rawIdx + bOffset + 3]
            val b5 = constraintData[rawIdx + bOffset + 4]
            val b6 = constraintData[rawIdx + bOffset + 5]

            val r1x = constraintData[rawIdx + jOffset + 0]
            val r1y = constraintData[rawIdx + jOffset + 1]
            val r1z = constraintData[rawIdx + jOffset + 2]

            val r2x = constraintData[rawIdx + jOffset + 3]
            val r2y = constraintData[rawIdx + jOffset + 4]
            val r2z = constraintData[rawIdx + jOffset + 5]

            val j42x = constraintData[rawIdx + jOffset + 6]
            val j42y = constraintData[rawIdx + jOffset + 7]
            val j42z = constraintData[rawIdx + jOffset + 8]

            val j52x = constraintData[rawIdx + jOffset + 9]
            val j52y = constraintData[rawIdx + jOffset + 10]
            val j52z = constraintData[rawIdx + jOffset + 11]

            val j62x = constraintData[rawIdx + jOffset + 12]
            val j62y = constraintData[rawIdx + jOffset + 13]
            val j62z = constraintData[rawIdx + jOffset + 14]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)
            val jv4 =
                -fma(j42x, v4, fma(j42y, v5, fma(j42z, v6, fma(-j42x, v10, fma(-j42y, v11, fma(-j42z, v12, b4))))))
            val jv5 =
                -fma(j52x, v4, fma(j52y, v5, fma(j52z, v6, fma(-j52x, v10, fma(-j52y, v11, fma(-j52z, v12, b5))))))
            val jv6 =
                -fma(j62x, v4, fma(j62y, v5, fma(j62z, v6, fma(-j62x, v10, fma(-j62y, v11, fma(-j62z, v12, b6))))))

            val im1 = constraintData[rawIdx + imOffset + 0]
            val im2 = constraintData[rawIdx + imOffset + 1]

            val e12x = constraintData[rawIdx + eOffset + 0]
            val e12y = constraintData[rawIdx + eOffset + 1]
            val e12z = constraintData[rawIdx + eOffset + 2]

            val e22x = constraintData[rawIdx + eOffset + 3]
            val e22y = constraintData[rawIdx + eOffset + 4]
            val e22z = constraintData[rawIdx + eOffset + 5]

            val e32x = constraintData[rawIdx + eOffset + 6]
            val e32y = constraintData[rawIdx + eOffset + 7]
            val e32z = constraintData[rawIdx + eOffset + 8]

            val e42x = constraintData[rawIdx + eOffset + 9]
            val e42y = constraintData[rawIdx + eOffset + 10]
            val e42z = constraintData[rawIdx + eOffset + 11]

            val e52x = constraintData[rawIdx + eOffset + 12]
            val e52y = constraintData[rawIdx + eOffset + 13]
            val e52z = constraintData[rawIdx + eOffset + 14]

            val e62x = constraintData[rawIdx + eOffset + 15]
            val e62y = constraintData[rawIdx + eOffset + 16]
            val e62z = constraintData[rawIdx + eOffset + 17]

            val e14x = constraintData[rawIdx + eOffset + 18]
            val e14y = constraintData[rawIdx + eOffset + 19]
            val e14z = constraintData[rawIdx + eOffset + 20]

            val e24x = constraintData[rawIdx + eOffset + 21]
            val e24y = constraintData[rawIdx + eOffset + 22]
            val e24z = constraintData[rawIdx + eOffset + 23]

            val e34x = constraintData[rawIdx + eOffset + 24]
            val e34y = constraintData[rawIdx + eOffset + 25]
            val e34z = constraintData[rawIdx + eOffset + 26]

            val e44x = constraintData[rawIdx + eOffset + 27]
            val e44y = constraintData[rawIdx + eOffset + 28]
            val e44z = constraintData[rawIdx + eOffset + 29]

            val e54x = constraintData[rawIdx + eOffset + 30]
            val e54y = constraintData[rawIdx + eOffset + 31]
            val e54z = constraintData[rawIdx + eOffset + 32]

            val e64x = constraintData[rawIdx + eOffset + 33]
            val e64y = constraintData[rawIdx + eOffset + 34]
            val e64z = constraintData[rawIdx + eOffset + 35]

            val k11 = constraintData[rawIdx + kOffset + 0]
            val k12 = constraintData[rawIdx + kOffset + 1]
            val k13 = constraintData[rawIdx + kOffset + 2]
            val k14 = constraintData[rawIdx + kOffset + 3]
            val k15 = constraintData[rawIdx + kOffset + 4]
            val k16 = constraintData[rawIdx + kOffset + 5]

            val k22 = constraintData[rawIdx + kOffset + 6]
            val k23 = constraintData[rawIdx + kOffset + 7]
            val k24 = constraintData[rawIdx + kOffset + 8]
            val k25 = constraintData[rawIdx + kOffset + 9]
            val k26 = constraintData[rawIdx + kOffset + 10]

            val k33 = constraintData[rawIdx + kOffset + 11]
            val k34 = constraintData[rawIdx + kOffset + 12]
            val k35 = constraintData[rawIdx + kOffset + 13]
            val k36 = constraintData[rawIdx + kOffset + 14]

            val k44 = constraintData[rawIdx + kOffset + 15]
            val k45 = constraintData[rawIdx + kOffset + 16]
            val k46 = constraintData[rawIdx + kOffset + 17]

            val k55 = constraintData[rawIdx + kOffset + 18]
            val k56 = constraintData[rawIdx + kOffset + 19]

            val k66 = constraintData[rawIdx + kOffset + 20]

            val t1 = constraintData[rawIdx + lOffset + 0]
            val t2 = constraintData[rawIdx + lOffset + 1]
            val t3 = constraintData[rawIdx + lOffset + 2]
            val t4 = constraintData[rawIdx + lOffset + 3]
            val t5 = constraintData[rawIdx + lOffset + 4]
            val t6 = constraintData[rawIdx + lOffset + 5]

            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float
            val l5: Float
            val l6: Float

            solveSymmetric6x6(
                k11, k12, k13, k14, k15, k16,
                k22, k23, k24, k25, k26,
                k33, k34, k35, k36,
                k44, k45, k46,
                k55, k56,
                k66,

                jv1, jv2, jv3, jv4, jv5, jv6,
            ) { s1, s2, s3, s4, s5, s6 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
                l4 = t4 + s4
                l5 = t5 + s5
                l6 = t6 + s6
            }

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3
            val d4 = l4 - t4
            val d5 = l5 - t5
            val d6 = l6 - t6

            val valid4 = l4Validator(d4, rawIdx + lOffset + 3)
            val valid5 = l5Validator(d5, rawIdx + lOffset + 4)
            val valid6 = l6Validator(d6, rawIdx + lOffset + 5)

            var numValid = 0
            if (valid4) numValid++
            if (valid5) numValid++
            if (valid6) numValid++
            when (numValid) {
                3 -> {
                    debugData += d1
                    debugData += d2
                    debugData += d3
                    debugData += d4
                    debugData += d5
                    debugData += d6

                    constraintData[rawIdx + lOffset + 0] = l1
                    constraintData[rawIdx + lOffset + 1] = l2
                    constraintData[rawIdx + lOffset + 2] = l3
                    constraintData[rawIdx + lOffset + 3] = l4
                    constraintData[rawIdx + lOffset + 4] = l5
                    constraintData[rawIdx + lOffset + 5] = l6

                    bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                    bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                    bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                    bodyData[b1Idx * 6 + 3] += fma(
                        e12x,
                        d1,
                        fma(e22x, d2, fma(e32x, d3, fma(e42x, d4, fma(e52x, d5, e62x * d6))))
                    ) * relaxation
                    bodyData[b1Idx * 6 + 4] += fma(
                        e12y,
                        d1,
                        fma(e22y, d2, fma(e32y, d3, fma(e42y, d4, fma(e52y, d5, e62y * d6))))
                    ) * relaxation
                    bodyData[b1Idx * 6 + 5] += fma(
                        e12z,
                        d1,
                        fma(e22z, d2, fma(e32z, d3, fma(e42z, d4, fma(e52z, d5, e62z * d6))))
                    ) * relaxation

                    bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                    bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                    bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                    bodyData[b2Idx * 6 + 3] += fma(
                        e14x,
                        d1,
                        fma(e24x, d2, fma(e34x, d3, fma(e44x, d4, fma(e54x, d5, e64x * d6))))
                    ) * relaxation
                    bodyData[b2Idx * 6 + 4] += fma(
                        e14y,
                        d1,
                        fma(e24y, d2, fma(e34y, d3, fma(e44y, d4, fma(e54y, d5, e64y * d6))))
                    ) * relaxation
                    bodyData[b2Idx * 6 + 5] += fma(
                        e14z,
                        d1,
                        fma(e24z, d2, fma(e34z, d3, fma(e44z, d4, fma(e54z, d5, e64z * d6))))
                    ) * relaxation
                }

                2 -> {
                    //(4, 5), (4, 6), (5, 6)
                    val jv4r: Float
                    val t4r: Float
                    val k14r: Float
                    val k24r: Float
                    val k34r: Float
                    val k44r: Float
                    val l4or: Int
                    val j42xr: Float
                    val j42yr: Float
                    val j42zr: Float
                    val e42xr: Float
                    val e42yr: Float
                    val e42zr: Float
                    val e44xr: Float
                    val e44yr: Float
                    val e44zr: Float
                    val b4r: Float

                    val jv5r: Float
                    val t5r: Float
                    val k15r: Float
                    val k25r: Float
                    val k35r: Float
                    val k45r: Float
                    val k55r: Float
                    val l5or: Int
                    val j52xr: Float
                    val j52yr: Float
                    val j52zr: Float
                    val e52xr: Float
                    val e52yr: Float
                    val e52zr: Float
                    val e54xr: Float
                    val e54yr: Float
                    val e54zr: Float
                    val b5r: Float

                    if (valid4 && valid5) { //4, 5
                        jv4r = jv4
                        t4r = t4
                        k14r = k14
                        k24r = k24
                        k34r = k34
                        k44r = k44
                        l4or = 3
                        j42xr = j42x
                        j42yr = j42y
                        j42zr = j42z
                        e42xr = e42x
                        e42yr = e42y
                        e42zr = e42z
                        e44xr = e44x
                        e44yr = e44y
                        e44zr = e44z
                        b4r = b4

                        jv5r = jv5
                        t5r = t5
                        k15r = k15
                        k25r = k25
                        k35r = k35
                        k45r = k45
                        k55r = k55
                        l5or = 4
                        j52xr = j52x
                        j52yr = j52y
                        j52zr = j52z
                        e52xr = e52x
                        e52yr = e52y
                        e52zr = e52z
                        e54xr = e54x
                        e54yr = e54y
                        e54zr = e54z
                        b5r = b5
                    } else if (valid4) {//4, 6
                        jv4r = jv4
                        t4r = t4
                        k14r = k14
                        k24r = k24
                        k34r = k34
                        k44r = k44
                        l4or = 3
                        j42xr = j42x
                        j42yr = j42y
                        j42zr = j42z
                        e42xr = e42x
                        e42yr = e42y
                        e42zr = e42z
                        e44xr = e44x
                        e44yr = e44y
                        e44zr = e44z
                        b4r = b4

                        jv5r = jv6
                        t5r = t6
                        k15r = k16
                        k25r = k26
                        k35r = k36
                        k45r = k46
                        k55r = k66
                        l5or = 5
                        j52xr = j62x
                        j52yr = j62y
                        j52zr = j62z
                        e52xr = e62x
                        e52yr = e62y
                        e52zr = e62z
                        e54xr = e64x
                        e54yr = e64y
                        e54zr = e64z
                        b5r = b6
                    } else {//5, 6
                        jv4r = jv5
                        t4r = t5
                        k14r = k15
                        k24r = k25
                        k34r = k35
                        k44r = k55
                        l4or = 4
                        j42xr = j52x
                        j42yr = j52y
                        j42zr = j52z
                        e42xr = e52x
                        e42yr = e52y
                        e42zr = e52z
                        e44xr = e54x
                        e44yr = e54y
                        e44zr = e54z
                        b4r = b5

                        jv5r = jv6
                        t5r = t6
                        k15r = k16
                        k25r = k26
                        k35r = k36
                        k45r = k56
                        k55r = k66
                        l5or = 5
                        j52xr = j62x
                        j52yr = j62y
                        j52zr = j62z
                        e52xr = e62x
                        e52yr = e62y
                        e52zr = e62z
                        e54xr = e64x
                        e54yr = e64y
                        e54zr = e64z
                        b5r = b6
                    }

                    val l1: Float
                    val l2: Float
                    val l3: Float
                    val l4: Float
                    val l5: Float

                    solveSymmetric5x5(
                        k11, k12, k13, k14r, k15r,
                        k22, k23, k24r, k25r,
                        k33, k34r, k35r,
                        k44r, k45r,
                        k55r,

                        jv1, jv2, jv3, jv4r, jv5r,
                    ) { s1, s2, s3, s4, s5 ->
                        l1 = t1 + s1
                        l2 = t2 + s2
                        l3 = t3 + s3
                        l4 = t4r + s4
                        l5 = t5r + s5
                    }

                    val d1 = l1 - t1
                    val d2 = l2 - t2
                    val d3 = l3 - t3
                    val d4 = l4 - t4r
                    val d5 = l5 - t5r

                    debugData += d1
                    debugData += d2
                    debugData += d3
                    debugData += d4
                    debugData += d5

                    constraintData[rawIdx + lOffset + 0] = l1
                    constraintData[rawIdx + lOffset + 1] = l2
                    constraintData[rawIdx + lOffset + 2] = l3
                    constraintData[rawIdx + lOffset + l4or] = l4
                    constraintData[rawIdx + lOffset + l5or] = l5

                    bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                    bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                    bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                    bodyData[b1Idx * 6 + 3] += fma(
                        e12x,
                        d1,
                        fma(e22x, d2, fma(e32x, d3, fma(e42xr, d4, e52xr * d5)))
                    ) * relaxation
                    bodyData[b1Idx * 6 + 4] += fma(
                        e12y,
                        d1,
                        fma(e22y, d2, fma(e32y, d3, fma(e42yr, d4, e52yr * d5)))
                    ) * relaxation
                    bodyData[b1Idx * 6 + 5] += fma(
                        e12z,
                        d1,
                        fma(e22z, d2, fma(e32z, d3, fma(e42zr, d4, e52zr * d5)))
                    ) * relaxation

                    bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                    bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                    bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                    bodyData[b2Idx * 6 + 3] += fma(
                        e14x,
                        d1,
                        fma(e24x, d2, fma(e34x, d3, fma(e44xr, d4, e54xr * d5)))
                    ) * relaxation
                    bodyData[b2Idx * 6 + 4] += fma(
                        e14y,
                        d1,
                        fma(e24y, d2, fma(e34y, d3, fma(e44yr, d4, e54yr * d5)))
                    ) * relaxation
                    bodyData[b2Idx * 6 + 5] += fma(
                        e14z,
                        d1,
                        fma(e24z, d2, fma(e34z, d3, fma(e44zr, d4, e54zr * d5)))
                    ) * relaxation
                }

                1 -> {
                    val jv4r: Float
                    val t4r: Float
                    val k14r: Float
                    val k24r: Float
                    val k34r: Float
                    val k44r: Float
                    val l4or: Int
                    val j42xr: Float
                    val j42yr: Float
                    val j42zr: Float
                    val e42xr: Float
                    val e42yr: Float
                    val e42zr: Float
                    val e44xr: Float
                    val e44yr: Float
                    val e44zr: Float
                    val b4r: Float
                    if (valid4) {
                        jv4r = jv4
                        t4r = t4
                        k14r = k14
                        k24r = k24
                        k34r = k34
                        k44r = k44
                        l4or = 3
                        j42xr = j42x
                        j42yr = j42y
                        j42zr = j42z
                        e42xr = e42x
                        e42yr = e42y
                        e42zr = e42z
                        e44xr = e44x
                        e44yr = e44y
                        e44zr = e44z
                        b4r = b4
                    } else if (valid5) {
                        jv4r = jv5
                        t4r = t5
                        k14r = k15
                        k24r = k25
                        k34r = k35
                        k44r = k55
                        l4or = 4
                        j42xr = j52x
                        j42yr = j52y
                        j42zr = j52z
                        e42xr = e52x
                        e42yr = e52y
                        e42zr = e52z
                        e44xr = e54x
                        e44yr = e54y
                        e44zr = e54z
                        b4r = b5
                    } else {
                        jv4r = jv6
                        t4r = t6
                        k14r = k16
                        k24r = k26
                        k34r = k36
                        k44r = k66
                        l4or = 5
                        j42xr = j62x
                        j42yr = j62y
                        j42zr = j62z
                        e42xr = e62x
                        e42yr = e62y
                        e42zr = e62z
                        e44xr = e64x
                        e44yr = e64y
                        e44zr = e64z
                        b4r = b6
                    }

                    val l1: Float
                    val l2: Float
                    val l3: Float
                    val l4: Float

                    solveSymmetric4x4(
                        k11, k12, k13, k14r,
                        k22, k23, k24r,
                        k33, k34r,
                        k44r,

                        jv1, jv2, jv3, jv4r,
                    ) { s1, s2, s3, s4 ->
                        l1 = t1 + s1
                        l2 = t2 + s2
                        l3 = t3 + s3
                        l4 = t4r + s4
                    }

                    val d1 = l1 - t1
                    val d2 = l2 - t2
                    val d3 = l3 - t3
                    val d4 = l4 - t4r

                    debugData += d1
                    debugData += d2
                    debugData += d3
                    debugData += d4

                    constraintData[rawIdx + lOffset + 0] = l1
                    constraintData[rawIdx + lOffset + 1] = l2
                    constraintData[rawIdx + lOffset + 2] = l3
                    constraintData[rawIdx + lOffset + l4or] = l4

                    bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                    bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                    bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                    bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, fma(e32x, d3, e42xr * d4))) * relaxation
                    bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, fma(e32y, d3, e42yr * d4))) * relaxation
                    bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, fma(e32z, d3, e42zr * d4))) * relaxation

                    bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                    bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                    bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                    bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, fma(e34x, d3, e44xr * d4))) * relaxation
                    bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, fma(e34y, d3, e44yr * d4))) * relaxation
                    bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, fma(e34z, d3, e44zr * d4))) * relaxation
                }

                else -> {
                    //both invalid
                    val l1: Float
                    val l2: Float
                    val l3: Float

                    solveSymmetric3x3(
                        k11, k12, k13,
                        k22, k23,
                        k33,

                        jv1, jv2, jv3,
                    ) { s1, s2, s3 ->
                        l1 = t1 + s1
                        l2 = t2 + s2
                        l3 = t3 + s3
                    }

                    val d1 = l1 - t1
                    val d2 = l2 - t2
                    val d3 = l3 - t3

                    debugData += d1
                    debugData += d2
                    debugData += d3

                    constraintData[rawIdx + lOffset + 0] = l1
                    constraintData[rawIdx + lOffset + 1] = l2
                    constraintData[rawIdx + lOffset + 2] = l3

                    bodyData[b1Idx * 6 + 0] += -im1 * d1 * relaxation
                    bodyData[b1Idx * 6 + 1] += -im1 * d2 * relaxation
                    bodyData[b1Idx * 6 + 2] += -im1 * d3 * relaxation
                    bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, e32x * d3)) * relaxation
                    bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, e32y * d3)) * relaxation
                    bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, e32z * d3)) * relaxation

                    bodyData[b2Idx * 6 + 0] += im2 * d1 * relaxation
                    bodyData[b2Idx * 6 + 1] += im2 * d2 * relaxation
                    bodyData[b2Idx * 6 + 2] += im2 * d3 * relaxation
                    bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, e34x * d3)) * relaxation
                    bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, e34y * d3)) * relaxation
                    bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, e34z * d3)) * relaxation
                }
            }
        }
    }

    fun solve3p0rPosition(
        b1: ActiveBody,
        b2: ActiveBody,

        rr1x: Float,
        rr1y: Float,
        rr1z: Float,

        rr2x: Float,
        rr2y: Float,
        rr2z: Float,

        erp: Float,
    ) {
        val q1 = b1.q
        val q2 = b2.q

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        val r1x: Float
        val r1y: Float
        val r1z: Float

        q1.transform(rr1x.toDouble(), rr1y.toDouble(), rr1z.toDouble()) { x, y, z ->
            r1x = x.toFloat()
            r1y = y.toFloat()
            r1z = z.toFloat()
        }

        val p1x = r1x + b1.pos.x.toFloat()
        val p1y = r1y + b1.pos.y.toFloat()
        val p1z = r1z + b1.pos.z.toFloat()

        val r2x: Float
        val r2y: Float
        val r2z: Float

        q2.transform(rr2x.toDouble(), rr2y.toDouble(), rr2z.toDouble()) { x, y, z ->
            r2x = x.toFloat()
            r2y = y.toFloat()
            r2z = z.toFloat()
        }

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

        solveSymmetric3x3(
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

        quatTransform(
            -q1.x, -q1.y, -q1.z, q1.w,
            erp * d4.toDouble(),
            erp * d5.toDouble(),
            erp * d6.toDouble(),
        ) { x, y, z ->
            quatMul(
                q1.x, q1.y, q1.z, q1.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q1.add(x, y, z, w).safeNormalize()
            }
        }

        b2.pos.add(
            erp * d7.toDouble(),
            erp * d8.toDouble(),
            erp * d9.toDouble(),
        )

        quatTransform(
            -q2.x, -q2.y, -q2.z, q2.w,
            erp * d10.toDouble(),
            erp * d11.toDouble(),
            erp * d12.toDouble(),
        ) { x, y, z ->
            quatMul(
                q2.x, q2.y, q2.z, q2.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q2.add(x, y, z, w).safeNormalize()
            }
        }

        if (abs(q1.x) + abs(q1.y) + abs(q1.z) + abs(q1.w) < 1e-4) {
            throw IllegalStateException()
        }

        if (abs(q2.x) + abs(q2.y) + abs(q2.z) + abs(q2.w) < 1e-4) {
            throw IllegalStateException()
        }
    }

    /**
     * rr1, rr2 = local relative positions on b1, b2
     * ra = world-space axis
     */
    fun solve3p1rPosition(
        b1: ActiveBody,
        b2: ActiveBody,

        rr1x: Float,
        rr1y: Float,
        rr1z: Float,

        rr2x: Float,
        rr2y: Float,
        rr2z: Float,

        ax: Float,
        ay: Float,
        az: Float,

        ea: Float,

        erp: Float,
    ) {
        val q1 = b1.q
        val q2 = b2.q

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        val r1x: Float
        val r1y: Float
        val r1z: Float

        q1.transform(rr1x.toDouble(), rr1y.toDouble(), rr1z.toDouble()) { x, y, z ->
            r1x = x.toFloat()
            r1y = y.toFloat()
            r1z = z.toFloat()
        }

        val p1x = r1x + b1.pos.x.toFloat()
        val p1y = r1y + b1.pos.y.toFloat()
        val p1z = r1z + b1.pos.z.toFloat()

        val r2x: Float
        val r2y: Float
        val r2z: Float

        q2.transform(rr2x.toDouble(), rr2y.toDouble(), rr2z.toDouble()) { x, y, z ->
            r2x = x.toFloat()
            r2y = y.toFloat()
            r2z = z.toFloat()
        }

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

        val j42x = -ax
        val j42y = -ay
        val j42z = -az

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

        val ej42x = fma(ii1.m00.toFloat(), j42x, fma(ii1.m10.toFloat(), j42y, ii1.m20.toFloat() * j42z))
        val ej42y = fma(ii1.m01.toFloat(), j42x, fma(ii1.m11.toFloat(), j42y, ii1.m21.toFloat() * j42z))
        val ej42z = fma(ii1.m02.toFloat(), j42x, fma(ii1.m12.toFloat(), j42y, ii1.m22.toFloat() * j42z))

        val ej44x = -fma(ii2.m00.toFloat(), j42x, fma(ii2.m10.toFloat(), j42y, ii2.m20.toFloat() * j42z))
        val ej44y = -fma(ii2.m01.toFloat(), j42x, fma(ii2.m11.toFloat(), j42y, ii2.m21.toFloat() * j42z))
        val ej44z = -fma(ii2.m02.toFloat(), j42x, fma(ii2.m12.toFloat(), j42y, ii2.m22.toFloat() * j42z))

        val d1x = ej12x - ej14x
        val d1y = ej12y - ej14y
        val d1z = ej12z - ej14z

        val d2x = ej22x - ej24x
        val d2y = ej22y - ej24y
        val d2z = ej22z - ej24z

        val d3x = ej32x - ej34x
        val d3y = ej32y - ej34y
        val d3z = ej32z - ej34z

        val d4x = ej42x - ej44x
        val d4y = ej42y - ej44y
        val d4z = ej42z - ej44z

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
        val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))

        val k22 =
            im1 +
            fma(ej22x, j22x, ej22z * j22z) +
            im2 +
            fma(ej24x, j24x, ej24z * j24z)
        val k23 =
            fma(ej22x, j32x, ej22y * j32y) +
            fma(ej24x, j34x, ej24y * j34y)
        val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))

        val k33 =
            im1 +
            fma(ej32x, j32x, ej32y * j32y) +
            im2 +
            fma(ej34x, j34x, ej34y * j34y)
        val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))

        val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))

        val l1: Float
        val l2: Float
        val l3: Float
        val l4: Float

        solveSymmetric4x4(
            k11, k12, k13, k14,
            k22, k23, k24,
            k33, k34,
            k44,

            -e1, -e2, -e3, -ea,
        ) { s1, s2, s3, s4 ->
            l1 = s1
            l2 = s2
            l3 = s3
            l4 = s4
        }

        val d1 = -im1 * l1
        val d2 = -im1 * l2
        val d3 = -im1 * l3
        val d4 = fma(ej12x, l1, fma(ej22x, l2, fma(ej32x, l3, ej42x * l4)))
        val d5 = fma(ej12y, l1, fma(ej22y, l2, fma(ej32y, l3, ej42y * l4)))
        val d6 = fma(ej12z, l1, fma(ej22z, l2, fma(ej32z, l3, ej42z * l4)))

        val d7 = im2 * l1
        val d8 = im2 * l2
        val d9 = im2 * l3
        val d10 = fma(ej14x, l1, fma(ej24x, l2, fma(ej34x, l3, ej44x * l4)))
        val d11 = fma(ej14y, l1, fma(ej24y, l2, fma(ej34y, l3, ej44y * l4)))
        val d12 = fma(ej14z, l1, fma(ej24z, l2, fma(ej34z, l3, ej44z * l4)))

        b1.pos.add(
            erp * d1.toDouble(),
            erp * d2.toDouble(),
            erp * d3.toDouble(),
        )

        quatTransform(
            -q1.x, -q1.y, -q1.z, q1.w,
            erp * d4.toDouble(),
            erp * d5.toDouble(),
            erp * d6.toDouble(),
        ) { x, y, z ->
            quatMul(
                q1.x, q1.y, q1.z, q1.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q1.add(x, y, z, w).safeNormalize()
            }
        }

        b2.pos.add(
            erp * d7.toDouble(),
            erp * d8.toDouble(),
            erp * d9.toDouble(),
        )

        quatTransform(
            -q2.x, -q2.y, -q2.z, q2.w,
            erp * d10.toDouble(),
            erp * d11.toDouble(),
            erp * d12.toDouble(),
        ) { x, y, z ->
            quatMul(
                q2.x, q2.y, q2.z, q2.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q2.add(x, y, z, w).safeNormalize()
            }
        }

        if (abs(q1.x) + abs(q1.y) + abs(q1.z) + abs(q1.w) < 1e-4) {
            throw IllegalStateException()
        }

        if (abs(q2.x) + abs(q2.y) + abs(q2.z) + abs(q2.w) < 1e-4) {
            throw IllegalStateException()
        }
    }

    /**
     * rr1, rr2 = local relative positions on b1, b2
     * ra = world-space axis
     * rb = world-space axis
     */
    fun solve3p2rPosition(
        b1: ActiveBody,
        b2: ActiveBody,

        rr1x: Float,
        rr1y: Float,
        rr1z: Float,

        rr2x: Float,
        rr2y: Float,
        rr2z: Float,

        ax: Float,
        ay: Float,
        az: Float,

        bx: Float,
        by: Float,
        bz: Float,

        ea: Float,
        eb: Float,

        erp: Float,
    ) {
        val q1 = b1.q
        val q2 = b2.q

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        val r1x: Float
        val r1y: Float
        val r1z: Float

        q1.transform(rr1x.toDouble(), rr1y.toDouble(), rr1z.toDouble()) { x, y, z ->
            r1x = x.toFloat()
            r1y = y.toFloat()
            r1z = z.toFloat()
        }

        val p1x = r1x + b1.pos.x.toFloat()
        val p1y = r1y + b1.pos.y.toFloat()
        val p1z = r1z + b1.pos.z.toFloat()

        val r2x: Float
        val r2y: Float
        val r2z: Float

        q2.transform(rr2x.toDouble(), rr2y.toDouble(), rr2z.toDouble()) { x, y, z ->
            r2x = x.toFloat()
            r2y = y.toFloat()
            r2z = z.toFloat()
        }

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

        val j42x = -ax
        val j42y = -ay
        val j42z = -az

        val j52x = -bx
        val j52y = -by
        val j52z = -bz

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

        val ej42x = fma(ii1.m00.toFloat(), j42x, fma(ii1.m10.toFloat(), j42y, ii1.m20.toFloat() * j42z))
        val ej42y = fma(ii1.m01.toFloat(), j42x, fma(ii1.m11.toFloat(), j42y, ii1.m21.toFloat() * j42z))
        val ej42z = fma(ii1.m02.toFloat(), j42x, fma(ii1.m12.toFloat(), j42y, ii1.m22.toFloat() * j42z))

        val ej44x = -fma(ii2.m00.toFloat(), j42x, fma(ii2.m10.toFloat(), j42y, ii2.m20.toFloat() * j42z))
        val ej44y = -fma(ii2.m01.toFloat(), j42x, fma(ii2.m11.toFloat(), j42y, ii2.m21.toFloat() * j42z))
        val ej44z = -fma(ii2.m02.toFloat(), j42x, fma(ii2.m12.toFloat(), j42y, ii2.m22.toFloat() * j42z))

        val ej52x = fma(ii1.m00.toFloat(), j52x, fma(ii1.m10.toFloat(), j52y, ii1.m20.toFloat() * j52z))
        val ej52y = fma(ii1.m01.toFloat(), j52x, fma(ii1.m11.toFloat(), j52y, ii1.m21.toFloat() * j52z))
        val ej52z = fma(ii1.m02.toFloat(), j52x, fma(ii1.m12.toFloat(), j52y, ii1.m22.toFloat() * j52z))

        val ej54x = -fma(ii2.m00.toFloat(), j52x, fma(ii2.m10.toFloat(), j52y, ii2.m20.toFloat() * j52z))
        val ej54y = -fma(ii2.m01.toFloat(), j52x, fma(ii2.m11.toFloat(), j52y, ii2.m21.toFloat() * j52z))
        val ej54z = -fma(ii2.m02.toFloat(), j52x, fma(ii2.m12.toFloat(), j52y, ii2.m22.toFloat() * j52z))

        val d1x = ej12x - ej14x
        val d1y = ej12y - ej14y
        val d1z = ej12z - ej14z

        val d2x = ej22x - ej24x
        val d2y = ej22y - ej24y
        val d2z = ej22z - ej24z

        val d3x = ej32x - ej34x
        val d3y = ej32y - ej34y
        val d3z = ej32z - ej34z

        val d4x = ej42x - ej44x
        val d4y = ej42y - ej44y
        val d4z = ej42z - ej44z

        val d5x = ej52x - ej54x
        val d5y = ej52y - ej54y
        val d5z = ej52z - ej54z

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
        val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))
        val k15 = fma(d1x, j52x, fma(d1y, j52y, d1z * j52z))

        val k22 =
            im1 +
            fma(ej22x, j22x, ej22z * j22z) +
            im2 +
            fma(ej24x, j24x, ej24z * j24z)
        val k23 =
            fma(ej22x, j32x, ej22y * j32y) +
            fma(ej24x, j34x, ej24y * j34y)
        val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))
        val k25 = fma(d2x, j52x, fma(d2y, j52y, d2z * j52z))

        val k33 =
            im1 +
            fma(ej32x, j32x, ej32y * j32y) +
            im2 +
            fma(ej34x, j34x, ej34y * j34y)
        val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))
        val k35 = fma(d3x, j52x, fma(d3y, j52y, d3z * j52z))

        val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))
        val k45 = fma(d4x, j52x, fma(d4y, j52y, d4z * j52z))

        val k55 = fma(d5x, j52x, fma(d5y, j52y, d5z * j52z))

        val l1: Float
        val l2: Float
        val l3: Float
        val l4: Float
        val l5: Float

        solveSymmetric5x5(
            k11, k12, k13, k14, k15,
            k22, k23, k24, k25,
            k33, k34, k35,
            k44, k45,
            k55,

            -e1, -e2, -e3, -ea, -eb,
        ) { s1, s2, s3, s4, s5 ->
            l1 = s1
            l2 = s2
            l3 = s3
            l4 = s4
            l5 = s5
        }

        val d1 = -im1 * l1
        val d2 = -im1 * l2
        val d3 = -im1 * l3
        val d4 = fma(ej12x, l1, fma(ej22x, l2, fma(ej32x, l3, fma(ej42x, l4, ej52x * l5))))
        val d5 = fma(ej12y, l1, fma(ej22y, l2, fma(ej32y, l3, fma(ej42y, l4, ej52y * l5))))
        val d6 = fma(ej12z, l1, fma(ej22z, l2, fma(ej32z, l3, fma(ej42z, l4, ej52z * l5))))

        val d7 = im2 * l1
        val d8 = im2 * l2
        val d9 = im2 * l3
        val d10 = fma(ej14x, l1, fma(ej24x, l2, fma(ej34x, l3, fma(ej44x, l4, ej54x * l5))))
        val d11 = fma(ej14y, l1, fma(ej24y, l2, fma(ej34y, l3, fma(ej44y, l4, ej54y * l5))))
        val d12 = fma(ej14z, l1, fma(ej24z, l2, fma(ej34z, l3, fma(ej44z, l4, ej54z * l5))))

        b1.pos.add(
            erp * d1.toDouble(),
            erp * d2.toDouble(),
            erp * d3.toDouble(),
        )

        quatTransform(
            -q1.x, -q1.y, -q1.z, q1.w,
            erp * d4.toDouble(),
            erp * d5.toDouble(),
            erp * d6.toDouble(),
        ) { x, y, z ->
            quatMul(
                q1.x, q1.y, q1.z, q1.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q1.add(x, y, z, w).safeNormalize()
            }
        }

        b2.pos.add(
            erp * d7.toDouble(),
            erp * d8.toDouble(),
            erp * d9.toDouble(),
        )

        quatTransform(
            -q2.x, -q2.y, -q2.z, q2.w,
            erp * d10.toDouble(),
            erp * d11.toDouble(),
            erp * d12.toDouble(),
        ) { x, y, z ->
            quatMul(
                q2.x, q2.y, q2.z, q2.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q2.add(x, y, z, w).safeNormalize()
            }
        }

        if (abs(q1.x) + abs(q1.y) + abs(q1.z) + abs(q1.w) < 1e-4) {
            throw IllegalStateException()
        }

        if (abs(q2.x) + abs(q2.y) + abs(q2.z) + abs(q2.w) < 1e-4) {
            throw IllegalStateException()
        }
    }

    /**
     * rr1, rr2 = local relative positions on b1, b2
     * ra = world-space axis
     * rb = world-space axis
     * rc = world-space axis
     */
    fun solve3p3rPosition(
        b1: ActiveBody,
        b2: ActiveBody,

        rr1x: Float,
        rr1y: Float,
        rr1z: Float,

        rr2x: Float,
        rr2y: Float,
        rr2z: Float,

        ax: Float,
        ay: Float,
        az: Float,

        bx: Float,
        by: Float,
        bz: Float,

        cx: Float,
        cy: Float,
        cz: Float,

        ea: Float,
        eb: Float,
        ec: Float,

        erp: Float,
    ) {
        val q1 = b1.q
        val q2 = b2.q

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        val r1x: Float
        val r1y: Float
        val r1z: Float

        q1.transform(rr1x.toDouble(), rr1y.toDouble(), rr1z.toDouble()) { x, y, z ->
            r1x = x.toFloat()
            r1y = y.toFloat()
            r1z = z.toFloat()
        }

        val p1x = r1x + b1.pos.x.toFloat()
        val p1y = r1y + b1.pos.y.toFloat()
        val p1z = r1z + b1.pos.z.toFloat()

        val r2x: Float
        val r2y: Float
        val r2z: Float

        q2.transform(rr2x.toDouble(), rr2y.toDouble(), rr2z.toDouble()) { x, y, z ->
            r2x = x.toFloat()
            r2y = y.toFloat()
            r2z = z.toFloat()
        }

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

        val j42x = -ax
        val j42y = -ay
        val j42z = -az

        val j52x = -bx
        val j52y = -by
        val j52z = -bz

        val j62x = -cx
        val j62y = -cy
        val j62z = -cz

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

        val ej42x = fma(ii1.m00.toFloat(), j42x, fma(ii1.m10.toFloat(), j42y, ii1.m20.toFloat() * j42z))
        val ej42y = fma(ii1.m01.toFloat(), j42x, fma(ii1.m11.toFloat(), j42y, ii1.m21.toFloat() * j42z))
        val ej42z = fma(ii1.m02.toFloat(), j42x, fma(ii1.m12.toFloat(), j42y, ii1.m22.toFloat() * j42z))

        val ej44x = -fma(ii2.m00.toFloat(), j42x, fma(ii2.m10.toFloat(), j42y, ii2.m20.toFloat() * j42z))
        val ej44y = -fma(ii2.m01.toFloat(), j42x, fma(ii2.m11.toFloat(), j42y, ii2.m21.toFloat() * j42z))
        val ej44z = -fma(ii2.m02.toFloat(), j42x, fma(ii2.m12.toFloat(), j42y, ii2.m22.toFloat() * j42z))

        val ej52x = fma(ii1.m00.toFloat(), j52x, fma(ii1.m10.toFloat(), j52y, ii1.m20.toFloat() * j52z))
        val ej52y = fma(ii1.m01.toFloat(), j52x, fma(ii1.m11.toFloat(), j52y, ii1.m21.toFloat() * j52z))
        val ej52z = fma(ii1.m02.toFloat(), j52x, fma(ii1.m12.toFloat(), j52y, ii1.m22.toFloat() * j52z))

        val ej54x = -fma(ii2.m00.toFloat(), j52x, fma(ii2.m10.toFloat(), j52y, ii2.m20.toFloat() * j52z))
        val ej54y = -fma(ii2.m01.toFloat(), j52x, fma(ii2.m11.toFloat(), j52y, ii2.m21.toFloat() * j52z))
        val ej54z = -fma(ii2.m02.toFloat(), j52x, fma(ii2.m12.toFloat(), j52y, ii2.m22.toFloat() * j52z))

        val ej62x = fma(ii1.m00.toFloat(), j62x, fma(ii1.m10.toFloat(), j62y, ii1.m20.toFloat() * j62z))
        val ej62y = fma(ii1.m01.toFloat(), j62x, fma(ii1.m11.toFloat(), j62y, ii1.m21.toFloat() * j62z))
        val ej62z = fma(ii1.m02.toFloat(), j62x, fma(ii1.m12.toFloat(), j62y, ii1.m22.toFloat() * j62z))

        val ej64x = -fma(ii2.m00.toFloat(), j62x, fma(ii2.m10.toFloat(), j62y, ii2.m20.toFloat() * j62z))
        val ej64y = -fma(ii2.m01.toFloat(), j62x, fma(ii2.m11.toFloat(), j62y, ii2.m21.toFloat() * j62z))
        val ej64z = -fma(ii2.m02.toFloat(), j62x, fma(ii2.m12.toFloat(), j62y, ii2.m22.toFloat() * j62z))

        val d1x = ej12x - ej14x
        val d1y = ej12y - ej14y
        val d1z = ej12z - ej14z

        val d2x = ej22x - ej24x
        val d2y = ej22y - ej24y
        val d2z = ej22z - ej24z

        val d3x = ej32x - ej34x
        val d3y = ej32y - ej34y
        val d3z = ej32z - ej34z

        val d4x = ej42x - ej44x
        val d4y = ej42y - ej44y
        val d4z = ej42z - ej44z

        val d5x = ej52x - ej54x
        val d5y = ej52y - ej54y
        val d5z = ej52z - ej54z

        val d6x = ej62x - ej64x
        val d6y = ej62y - ej64y
        val d6z = ej62z - ej64z

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
        val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))
        val k15 = fma(d1x, j52x, fma(d1y, j52y, d1z * j52z))
        val k16 = fma(d1x, j62x, fma(d1y, j62y, d1z * j62z))

        val k22 =
            im1 +
            fma(ej22x, j22x, ej22z * j22z) +
            im2 +
            fma(ej24x, j24x, ej24z * j24z)
        val k23 =
            fma(ej22x, j32x, ej22y * j32y) +
            fma(ej24x, j34x, ej24y * j34y)
        val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))
        val k25 = fma(d2x, j52x, fma(d2y, j52y, d2z * j52z))
        val k26 = fma(d2x, j62x, fma(d2y, j62y, d2z * j62z))

        val k33 =
            im1 +
            fma(ej32x, j32x, ej32y * j32y) +
            im2 +
            fma(ej34x, j34x, ej34y * j34y)
        val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))
        val k35 = fma(d3x, j52x, fma(d3y, j52y, d3z * j52z))
        val k36 = fma(d3x, j62x, fma(d3y, j62y, d3z * j62z))

        val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))
        val k45 = fma(d4x, j52x, fma(d4y, j52y, d4z * j52z))
        val k46 = fma(d4x, j62x, fma(d4y, j62y, d4z * j62z))

        val k55 = fma(d5x, j52x, fma(d5y, j52y, d5z * j52z))
        val k56 = fma(d5x, j62x, fma(d5y, j62y, d5z * j62z))

        val k66 = fma(d6x, j62x, fma(d6y, j62y, d6z * j62z))

        val l1: Float
        val l2: Float
        val l3: Float
        val l4: Float
        val l5: Float
        val l6: Float

        solveSymmetric6x6(
            k11, k12, k13, k14, k15, k16,
            k22, k23, k24, k25, k26,
            k33, k34, k35, k36,
            k44, k45, k46,
            k55, k56,
            k66,

            -e1, -e2, -e3, -ea, -eb, -ec,
        ) { s1, s2, s3, s4, s5, s6 ->
            l1 = s1
            l2 = s2
            l3 = s3
            l4 = s4
            l5 = s5
            l6 = s6
        }

        val d1 = -im1 * l1
        val d2 = -im1 * l2
        val d3 = -im1 * l3
        val d4 = fma(ej12x, l1, fma(ej22x, l2, fma(ej32x, l3, fma(ej42x, l4, fma(ej52x, l5, ej62x * l6)))))
        val d5 = fma(ej12y, l1, fma(ej22y, l2, fma(ej32y, l3, fma(ej42y, l4, fma(ej52y, l5, ej62y * l6)))))
        val d6 = fma(ej12z, l1, fma(ej22z, l2, fma(ej32z, l3, fma(ej42z, l4, fma(ej52z, l5, ej62z * l6)))))

        val d7 = im2 * l1
        val d8 = im2 * l2
        val d9 = im2 * l3
        val d10 = fma(ej14x, l1, fma(ej24x, l2, fma(ej34x, l3, fma(ej44x, l4, fma(ej54x, l5, ej64x * l6)))))
        val d11 = fma(ej14y, l1, fma(ej24y, l2, fma(ej34y, l3, fma(ej44y, l4, fma(ej54y, l5, ej64y * l6)))))
        val d12 = fma(ej14z, l1, fma(ej24z, l2, fma(ej34z, l3, fma(ej44z, l4, fma(ej54z, l5, ej64z * l6)))))

        b1.pos.add(
            erp * d1.toDouble(),
            erp * d2.toDouble(),
            erp * d3.toDouble(),
        )

        quatTransform(
            -q1.x, -q1.y, -q1.z, q1.w,
            erp * d4.toDouble(),
            erp * d5.toDouble(),
            erp * d6.toDouble(),
        ) { x, y, z ->
            quatMul(
                q1.x, q1.y, q1.z, q1.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q1.add(x, y, z, w).safeNormalize()
            }
        }

        b2.pos.add(
            erp * d7.toDouble(),
            erp * d8.toDouble(),
            erp * d9.toDouble(),
        )

        quatTransform(
            -q2.x, -q2.y, -q2.z, q2.w,
            erp * d10.toDouble(),
            erp * d11.toDouble(),
            erp * d12.toDouble(),
        ) { x, y, z ->
            quatMul(
                q2.x, q2.y, q2.z, q2.w,
                x, y, z, 0.0,
            ) { x, y, z, w ->
                q2.add(x, y, z, w).safeNormalize()
            }
        }
    }
}

const val INVALID_LAMBDA = -1e-4f