package com.ixume.udar.physics.position

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.ConstraintData3p0r
import com.ixume.udar.physics.constraint.ConstraintMath
import com.ixume.udar.physics.constraint.ConstraintSolver
import com.ixume.udar.physics.constraint.QuatMath.transform
import java.lang.Math.fma
import kotlin.math.abs
import kotlin.math.pow

class PointConstraintSolver(val parent: ConstraintSolver) {
    private var constraintData = ConstraintData3p0r(0)
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
            constraintData = ConstraintData3p0r(s)
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

            val r1x: Float
            val r1y: Float
            val r1z: Float

            q1.transform(constraint.r1x.toDouble(), constraint.r1y.toDouble(), constraint.r1z.toDouble()) { x, y, z ->
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

            q2.transform(constraint.r2x.toDouble(), constraint.r2y.toDouble(), constraint.r2z.toDouble()) { x, y, z ->
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
                cursor = i * ConstraintData3p0r.DATA_SIZE,
                constraintIdx = i,
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

    fun solveVelocity(iteration: Int) {
        val bodyData = parent.flatBodyData
        ConstraintMath.solve3p0rVelocity(
            bodyData = bodyData,
            constraintData = constraintData,
            numConstraints = numConstraints,
            l1Transform = { l, i ->
                parent.debugDeltaLambdas[iteration - 1] += abs(l - constraintData.value[i])
                l
            },
            l2Transform = { l, i ->
                parent.debugDeltaLambdas[iteration - 1] += abs(l - constraintData.value[i])
                l
            },
            l3Transform = { l, i ->
                parent.debugDeltaLambdas[iteration - 1] += abs(l - constraintData.value[i])
                l
            },
        )
    }

    fun solvePosition() {
        constraintData.forEach(numConstraints) { i, _, _, rawIdx ->
            val constraint = rawConstraints[i]

            ConstraintMath.solve3p0rPosition(
                constraint.b1, constraint.b2,
                constraint.r1x, constraint.r1y, constraint.r1z,
                constraint.r2x, constraint.r2y, constraint.r2z,
                erp
            )
        }
    }
}