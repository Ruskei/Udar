package com.ixume.udar.physics.cone

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.*
import com.ixume.udar.physics.constraint.MatrixMath.cross
import com.ixume.udar.physics.constraint.MatrixMath.dot
import com.ixume.udar.physics.constraint.MatrixMath.normalize
import com.ixume.udar.physics.constraint.MatrixMath.reject
import com.ixume.udar.physics.constraint.MiscMath.biasedSign
import com.ixume.udar.physics.constraint.QuatMath.transform
import java.lang.Math.fma
import kotlin.math.*

class ConeConstraintSolver(val parent: ConstraintSolver) {
    private var constraintData3x3 = ConstraintData3p0r(0)
    private var numConstraints3x3 = 0
    private var constraintData4x4 = ConstraintData3p1r(0)
    private var numConstraints4x4 = 0
    private var constraintData5x5 = ConstraintData3p2r(0)
    private var numConstraints5x5 = 0
    private var constraintData6x6 = ConstraintData3p3r(0)
    private var numConstraints6x6 = 0

    private var dt = Udar.CONFIG.timeStep.toFloat()
    private var bias = Udar.CONFIG.coneConstraint.bias
    private var slop = Udar.CONFIG.coneConstraint.slop
    private var carryover = Udar.CONFIG.coneConstraint.carryover
    private var relaxation = Udar.CONFIG.coneConstraint.relaxation
    private var frictionTorque = Udar.CONFIG.coneConstraint.frictionTorque
    private fun effectiveERP() =
        1f - (1.0 - Udar.CONFIG.coneConstraint.erp.toDouble()).pow(1.0 / Udar.CONFIG.collision.posIterations.toDouble())
            .toFloat()

    private var erp = effectiveERP()

    private lateinit var rawConstraints: List<ConeConstraint>

    fun setup(constraints: List<ConeConstraint>) {
        dt = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.coneConstraint.bias
        slop = Udar.CONFIG.coneConstraint.slop
        carryover = Udar.CONFIG.coneConstraint.carryover
        relaxation = Udar.CONFIG.coneConstraint.relaxation
        frictionTorque = Udar.CONFIG.coneConstraint.frictionTorque

        erp = effectiveERP()

        rawConstraints = constraints

        val totalNumConstraints = constraints.size

        if (constraintData3x3.sizeFor(totalNumConstraints) != constraintData3x3.value.size)
            constraintData3x3 = ConstraintData3p0r(constraintData3x3.sizeFor(totalNumConstraints))
        if (constraintData4x4.sizeFor(totalNumConstraints) != constraintData4x4.value.size)
            constraintData4x4 = ConstraintData3p1r(constraintData4x4.sizeFor(totalNumConstraints))
        if (constraintData5x5.sizeFor(totalNumConstraints) != constraintData5x5.value.size)
            constraintData5x5 = ConstraintData3p2r(constraintData5x5.sizeFor(totalNumConstraints))
        if (constraintData6x6.sizeFor(totalNumConstraints) != constraintData6x6.value.size)
            constraintData6x6 = ConstraintData3p3r(constraintData6x6.sizeFor(totalNumConstraints))

        numConstraints3x3 = 0
        numConstraints4x4 = 0
        numConstraints5x5 = 0
        numConstraints6x6 = 0

        var i = 0
        while (i < totalNumConstraints) {
            val constraint = constraints[i]


            val b1 = constraint.b1
            val b2 = constraint.b2

            check(b1.idx != -1)
            check(b2.idx != -1)

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

            val bias1 = bias * (p2x - p1x) / dt
            val bias2 = bias * (p2y - p1y) / dt
            val bias3 = bias * (p2z - p1z) / dt

            val x1x: Float
            val x1y: Float
            val x1z: Float

            q1.transform(constraint.x1x.toDouble(), constraint.x1y.toDouble(), constraint.x1z.toDouble()) { x, y, z ->
                normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                    x1x = x
                    x1y = y
                    x1z = z
                }
            }

            val y1x: Float
            val y1y: Float
            val y1z: Float

            q1.transform(constraint.y1x.toDouble(), constraint.y1y.toDouble(), constraint.y1z.toDouble()) { x, y, z ->
                normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                    y1x = x
                    y1y = y
                    y1z = z
                }
            }

            val x2x: Float
            val x2y: Float
            val x2z: Float

            q2.transform(constraint.x2x.toDouble(), constraint.x2y.toDouble(), constraint.x2z.toDouble()) { x, y, z ->
                normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                    x2x = x
                    x2y = y
                    x2z = z
                }
            }

            val z2x: Float
            val z2y: Float
            val z2z: Float

            q2.transform(constraint.z2x.toDouble(), constraint.z2y.toDouble(), constraint.z2z.toDouble()) { x, y, z ->
                normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                    z2x = x
                    z2y = y
                    z2z = z
                }
            }

            val z1x: Float
            val z1y: Float
            val z1z: Float

            cross(
                x1x, x1y, x1z,
                y1x, y1y, y1z,
            ) { cx, cy, cz ->
                z1x = cx
                z1y = cy
                z1z = cz
            }

            val theta: Float

            reject(
                x1x, x1y, x1z, //from
                z2x, z2y, z2z
            ) { cx, cy, cz ->
                var t: Float
                normalize(cx, cy, cz) { cx, cy, cz ->
                    val cos =
                        dot(
                            cx, cy, cz,
                            z1x, z1y, z1z,
                        )

                    val sin: Float

                    cross(
                        cx, cy, cz,
                        z1x, z1y, z1z,
                    ) { cx, cy, cz ->
                        sin = dot(cx, cy, cz, x1x, x1y, x1z)
                    }

                    t = -atan2(sin, cos)
                }

                theta = if (t.isNaN()) 0f else t
            }

            val sinTheta = sin(theta)

            val phi: Float

            reject(
                y1x, y1y, y1z, //from
                z2x, z2y, z2z
            ) { cx, cy, cz ->
                var t: Float
                normalize(cx, cy, cz) { cx, cy, cz ->
                    val cos =
                        dot(
                            cx, cy, cz,
                            z1x, z1y, z1z
                        )

                    val sin: Float

                    cross(
                        cx, cy, cz,
                        z1x, z1y, z1z,
                    ) { cx, cy, cz ->
                        sin = dot(cx, cy, cz, y1x, y1y, y1z)
                    }

                    t = -atan2(sin, cos)
                }


                phi = if (t.isNaN()) 0f else t
            }

            val sinPhi = sin(phi)

            val sinMaxTheta = sin(constraint.maxXAngle)
            val sinMaxPhi = sin(constraint.maxYAngle)

            val r =
                sinTheta * sinTheta / (sinMaxTheta * sinMaxTheta) +
                sinPhi * sinPhi / (sinMaxPhi * sinMaxPhi)

            val constrainSwing: Boolean
            val eTheta: Float
            val ePhi: Float

            val jThetaX: Float
            val jThetaY: Float
            val jThetaZ: Float

            val jPhiX: Float
            val jPhiY: Float
            val jPhiZ: Float

            if (r > 1) {
                constrainSwing = true
                val clampedTheta: Float
                val clampedPhi: Float
                if (abs(sinPhi) < abs(sinTheta)) { // swap for stability and avoid division by 0
                    val k = sinPhi / sinTheta
                    val lambda =
                        theta.biasedSign * abs(sinMaxTheta * sinMaxPhi) / sqrt(k * k * sinMaxTheta * sinMaxTheta + sinMaxPhi * sinMaxPhi)

                    clampedTheta = asin(lambda)
                    clampedPhi = asin(lambda * k)
                } else {
                    val k = sinTheta / sinPhi
                    val lambda =
                        phi.biasedSign * abs(sinMaxTheta * sinMaxPhi) / sqrt(sinMaxTheta * sinMaxTheta + k * k * sinMaxPhi * sinMaxPhi)

                    clampedTheta = asin(lambda * k)
                    clampedPhi = asin(lambda)
                }

                eTheta = theta.biasedSign * (clampedTheta - theta)
                jThetaX = theta.biasedSign * x1x
                jThetaY = theta.biasedSign * x1y
                jThetaZ = theta.biasedSign * x1z
                ePhi = phi.biasedSign * (clampedPhi - phi)
                jPhiX = phi.biasedSign * y1x
                jPhiY = phi.biasedSign * y1y
                jPhiZ = phi.biasedSign * y1z
            } else {
                constrainSwing = false
                eTheta = 0f
                ePhi = 0f
                jThetaX = 0f
                jThetaY = 0f
                jThetaZ = 0f
                jPhiX = 0f
                jPhiY = 0f
                jPhiZ = 0f
            }

            val biasTheta = bias * eTheta / dt
            val biasPhi = bias * ePhi / dt

            val constrainTwist: Boolean
            val eTwist: Float
            val psi: Float
            val jTwistX: Float
            val jTwistY: Float
            val jTwistZ: Float

            reject(
                z2x, z2y, z2z, //from
                x1x, x1y, x1z
            ) { cx, cy, cz ->
                if (fma(cx, cx, fma(cy, cy, cz * cz)) < 1e-4) {
                    constrainTwist = false
                    psi = 0f
                    eTwist = 0f
                    jTwistX = 0f
                    jTwistY = 0f
                    jTwistZ = 0f
                    return@reject
                }

                normalize(cx, cy, cz) { cx, cy, cz ->
                    val cos = dot(cx, cy, cz, x2x, x2y, x2z)
                    val sin: Float
                    cross(
                        cx, cy, cz,
                        x2x, x2y, x2z,
                    ) { cx, cy, cz ->
                        sin = dot(cx, cy, cz, z2x, z2y, z2z)
                    }

                    psi = atan2(sin, cos)
                    if (psi < constraint.minTwistAngle) {
                        constrainTwist = true
                        eTwist = psi - constraint.minTwistAngle
                        jTwistX = -z2x
                        jTwistY = -z2y
                        jTwistZ = -z2z
                    } else if (psi > constraint.maxTwistAngle) {
                        constrainTwist = true
                        eTwist = constraint.maxTwistAngle - psi
                        jTwistX = z2x
                        jTwistY = z2y
                        jTwistZ = z2z
                    } else {
                        constrainTwist = false
                        eTwist = 0f
                        jTwistX = 0f
                        jTwistY = 0f
                        jTwistZ = 0f
                    }
                }
            }


            val biasTwist = bias * eTwist / dt

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

            if (!constrainTwist && !constrainSwing) { //3
                constraintData3x3.set(
                    numConstraints3x3++ * ConstraintData3p0r.DATA_SIZE,
                    i,
                    b1.idx, b2.idx,
                    r1x, r1y, r1z,
                    r2x, r2y, r2z,
                    im1, im2,
                    ej12x, ej12y, ej12z,
                    ej22x, ej22y, ej22z,
                    ej32x, ej32y, ej32z,
                    ej14x, ej14y, ej14z,
                    ej24x, ej24y, ej24z,
                    ej34x, ej34y, ej34z,
                    k11, k12, k13,
                    k22, k23,
                    k33,
                    bias1, bias2, bias3,
                    constraint.λ3x31, constraint.λ3x32, constraint.λ3x33,
                )
            } else if (constrainTwist && !constrainSwing) { //4
                val j42x = jTwistX
                val j42y = jTwistY
                val j42z = jTwistZ

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

                val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))
                val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))
                val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))
                val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))

                constraintData4x4.set(
                    numConstraints4x4++ * ConstraintData3p1r.DATA_SIZE,
                    i,
                    b1.idx, b2.idx,
                    r1x, r1y, r1z,
                    r2x, r2y, r2z,
                    j42x, j42y, j42z,
                    im1, im2,
                    ej12x, ej12y, ej12z,
                    ej22x, ej22y, ej22z,
                    ej32x, ej32y, ej32z,
                    ej42x, ej42y, ej42z,
                    ej14x, ej14y, ej14z,
                    ej24x, ej24y, ej24z,
                    ej34x, ej34y, ej34z,
                    ej44x, ej44y, ej44z,
                    k11, k12, k13, k14,
                    k22, k23, k24,
                    k33, k34,
                    k44,
                    bias1, bias2, bias3, biasTwist,
                    0f, 0f, 0f, 0f,
                )
            } else if (!constrainTwist) { //5
                val j42x = jThetaX
                val j42y = jThetaY
                val j42z = jThetaZ
                check(abs(sqrt(j42x * j42x + j42y * j42y + j42z * j42z) - 1) < 1e-4)

                val j52x = jPhiX
                val j52y = jPhiY
                val j52z = jPhiZ
                check(abs(sqrt(j52x * j52x + j52y * j52y + j52z * j52z) - 1) < 1e-4)

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

                val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))
                val k15 = fma(d1x, j52x, fma(d1y, j52y, d1z * j52z))

                val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))
                val k25 = fma(d2x, j52x, fma(d2y, j52y, d2z * j52z))

                val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))
                val k35 = fma(d3x, j52x, fma(d3y, j52y, d3z * j52z))

                val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))
                val k45 = fma(d4x, j52x, fma(d4y, j52y, d4z * j52z))

                val k55 = fma(d5x, j52x, fma(d5y, j52y, d5z * j52z))

                constraintData5x5.set(
                    numConstraints5x5++ * ConstraintData3p2r.DATA_SIZE,
                    i,
                    b1.idx, b2.idx,
                    r1x, r1y, r1z,
                    r2x, r2y, r2z,
                    j42x, j42y, j42z,
                    j52x, j52y, j52z,
                    im1, im2,
                    ej12x, ej12y, ej12z,
                    ej22x, ej22y, ej22z,
                    ej32x, ej32y, ej32z,
                    ej42x, ej42y, ej42z,
                    ej52x, ej52y, ej52z,
                    ej14x, ej14y, ej14z,
                    ej24x, ej24y, ej24z,
                    ej34x, ej34y, ej34z,
                    ej44x, ej44y, ej44z,
                    ej54x, ej54y, ej54z,
                    k11, k12, k13, k14, k15,
                    k22, k23, k24, k25,
                    k33, k34, k35,
                    k44, k45,
                    k55,
                    bias1, bias2, bias3, biasTheta, biasPhi,
                    0f, 0f, 0f, 0f, 0f,
                )
            } else { //6
                val j42x = jThetaX
                val j42y = jThetaY
                val j42z = jThetaZ

                val j52x = jPhiX
                val j52y = jPhiY
                val j52z = jPhiZ

                val j62x = jTwistX
                val j62y = jTwistY
                val j62z = jTwistZ

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

                val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))
                val k15 = fma(d1x, j52x, fma(d1y, j52y, d1z * j52z))
                val k16 = fma(d1x, j62x, fma(d1y, j62y, d1z * j62z))

                val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))
                val k25 = fma(d2x, j52x, fma(d2y, j52y, d2z * j52z))
                val k26 = fma(d2x, j62x, fma(d2y, j62y, d2z * j62z))

                val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))
                val k35 = fma(d3x, j52x, fma(d3y, j52y, d3z * j52z))
                val k36 = fma(d3x, j62x, fma(d3y, j62y, d3z * j62z))

                val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))
                val k45 = fma(d4x, j52x, fma(d4y, j52y, d4z * j52z))
                val k46 = fma(d4x, j62x, fma(d4y, j62y, d4z * j62z))

                val k55 = fma(d5x, j52x, fma(d5y, j52y, d5z * j52z))
                val k56 = fma(d5x, j62x, fma(d5y, j62y, d5z * j62z))

                val k66 = fma(d6x, j62x, fma(d6y, j62y, d6z * j62z))

                constraintData6x6.set(
                    numConstraints6x6++ * ConstraintData3p3r.DATA_SIZE,
                    i,
                    b1.idx, b2.idx,
                    r1x, r1y, r1z,
                    r2x, r2y, r2z,
                    j42x, j42y, j42z,
                    j52x, j52y, j52z,
                    j62x, j62y, j62z,
                    im1, im2,
                    ej12x, ej12y, ej12z,
                    ej22x, ej22y, ej22z,
                    ej32x, ej32y, ej32z,
                    ej42x, ej42y, ej42z,
                    ej52x, ej52y, ej52z,
                    ej62x, ej62y, ej62z,
                    ej14x, ej14y, ej14z,
                    ej24x, ej24y, ej24z,
                    ej34x, ej34y, ej34z,
                    ej44x, ej44y, ej44z,
                    ej54x, ej54y, ej54z,
                    ej64x, ej64y, ej64z,
                    k11, k12, k13, k14, k15, k16,
                    k22, k23, k24, k25, k26,
                    k33, k34, k35, k36,
                    k44, k45, k46,
                    k55, k56,
                    k66,
                    bias1, bias2, bias3, biasTheta, biasPhi, biasTwist,
                    0f, 0f, 0f, 0f, 0f, 0f,
                )
            }

            i++
        }

        ConstraintMath.warm3p0r(
            parent = parent,
            constraintData = constraintData3x3,
            numConstraints = numConstraints3x3,
            relaxation = relaxation,
        )
    }

    fun solveVelocity() {
        ConstraintMath.solve3p0rVelocity(
            parent = parent,
            constraintData = constraintData3x3,
            numConstraints = numConstraints3x3,
            relaxation = relaxation,
        )

        ConstraintMath.solve3p1rVelocity(
            parent = parent,
            constraintData = constraintData4x4,
            numConstraints = numConstraints4x4,
            relaxation = relaxation,
        )

        ConstraintMath.solve3p2rVelocity(
            parent = parent,
            constraintData = constraintData5x5,
            numConstraints = numConstraints5x5,
            relaxation = relaxation,
        )

        ConstraintMath.solve3p3rVelocity(
            parent = parent,
            constraintData = constraintData6x6,
            numConstraints = numConstraints6x6,
            relaxation = relaxation,
        )
    }

    fun heatUp() {
        if (carryover <= 0f) return
        constraintData3x3.forEach(numConstraints3x3) { idx, b1Idx, b2Idx, rawIdx ->
            val c = rawConstraints[idx]
            c.λ3x31 = constraintData3x3[rawIdx + ConstraintData3p0r.L_OFFSET + 0] * carryover
            c.λ3x32 = constraintData3x3[rawIdx + ConstraintData3p0r.L_OFFSET + 1] * carryover
            c.λ3x33 = constraintData3x3[rawIdx + ConstraintData3p0r.L_OFFSET + 2] * carryover
        }
    }

    fun solvePosition() {
        constraintData3x3.forEach(numConstraints3x3) { i, _, _, _ -> solvePosition(rawConstraints[i]) }
        constraintData4x4.forEach(numConstraints4x4) { i, _, _, _ -> solvePosition(rawConstraints[i]) }
        constraintData5x5.forEach(numConstraints5x5) { i, _, _, _ -> solvePosition(rawConstraints[i]) }
        constraintData6x6.forEach(numConstraints6x6) { i, _, _, _ -> solvePosition(rawConstraints[i]) }
    }

    private fun solvePosition(constraint: ConeConstraint) {
        val b1 = constraint.b1
        val b2 = constraint.b2

        val q1 = b1.q
        val q2 = b2.q

        val x1x: Float
        val x1y: Float
        val x1z: Float

        q1.transform(constraint.x1x.toDouble(), constraint.x1y.toDouble(), constraint.x1z.toDouble()) { x, y, z ->
            normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                x1x = x
                x1y = y
                x1z = z
            }
        }

        val y1x: Float
        val y1y: Float
        val y1z: Float

        q1.transform(constraint.y1x.toDouble(), constraint.y1y.toDouble(), constraint.y1z.toDouble()) { x, y, z ->
            normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                y1x = x
                y1y = y
                y1z = z
            }
        }

        val x2x: Float
        val x2y: Float
        val x2z: Float

        q2.transform(constraint.x2x.toDouble(), constraint.x2y.toDouble(), constraint.x2z.toDouble()) { x, y, z ->
            normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                x2x = x
                x2y = y
                x2z = z
            }
        }

        val z2x: Float
        val z2y: Float
        val z2z: Float

        q2.transform(constraint.z2x.toDouble(), constraint.z2y.toDouble(), constraint.z2z.toDouble()) { x, y, z ->
            normalize(x.toFloat(), y.toFloat(), z.toFloat()) { x, y, z ->
                z2x = x
                z2y = y
                z2z = z
            }
        }

        val z1x: Float
        val z1y: Float
        val z1z: Float

        cross(
            x1x, x1y, x1z,
            y1x, y1y, y1z,
        ) { cx, cy, cz ->
            z1x = cx
            z1y = cy
            z1z = cz
        }

        val theta: Float

        reject(
            x1x, x1y, x1z, //from
            z2x, z2y, z2z
        ) { cx, cy, cz ->
            var t: Float
            normalize(cx, cy, cz) { cx, cy, cz ->
                val cos =
                    dot(
                        cx, cy, cz,
                        z1x, z1y, z1z,
                    )

                val sin: Float

                cross(
                    cx, cy, cz,
                    z1x, z1y, z1z,
                ) { cx, cy, cz ->
                    sin = dot(cx, cy, cz, x1x, x1y, x1z)
                }

                t = -atan2(sin, cos)
            }

            theta = if (t.isNaN()) 0f else t
        }

        val sinTheta = sin(theta)

        val phi: Float

        reject(
            y1x, y1y, y1z, //from
            z2x, z2y, z2z
        ) { cx, cy, cz ->
            var t: Float
            normalize(cx, cy, cz) { cx, cy, cz ->
                val cos =
                    dot(
                        cx, cy, cz,
                        z1x, z1y, z1z
                    )

                val sin: Float

                cross(
                    cx, cy, cz,
                    z1x, z1y, z1z,
                ) { cx, cy, cz ->
                    sin = dot(cx, cy, cz, y1x, y1y, y1z)
                }

                t = -atan2(sin, cos)
            }


            phi = if (t.isNaN()) 0f else t
        }

        val sinPhi = sin(phi)

        val sinMaxTheta = sin(constraint.maxXAngle)
        val sinMaxPhi = sin(constraint.maxYAngle)

        val r =
            sinTheta * sinTheta / (sinMaxTheta * sinMaxTheta) +
            sinPhi * sinPhi / (sinMaxPhi * sinMaxPhi)

        val constrainSwing: Boolean
        val eTheta: Float
        val ePhi: Float

        val jThetaX: Float
        val jThetaY: Float
        val jThetaZ: Float

        val jPhiX: Float
        val jPhiY: Float
        val jPhiZ: Float

        if (r > 1) {
            constrainSwing = true
            val clampedTheta: Float
            val clampedPhi: Float
            if (abs(sinPhi) < abs(sinTheta)) { // swap for stability and avoid division by 0
                val k = sinPhi / sinTheta
                val lambda =
                    theta.biasedSign * abs(sinMaxTheta * sinMaxPhi) / sqrt(k * k * sinMaxTheta * sinMaxTheta + sinMaxPhi * sinMaxPhi)

                clampedTheta = asin(lambda)
                clampedPhi = asin(lambda * k)
            } else {
                val k = sinTheta / sinPhi
                val lambda =
                    phi.biasedSign * abs(sinMaxTheta * sinMaxPhi) / sqrt(sinMaxTheta * sinMaxTheta + k * k * sinMaxPhi * sinMaxPhi)

                clampedTheta = asin(lambda * k)
                clampedPhi = asin(lambda)
            }

            eTheta = theta.biasedSign * (clampedTheta - theta)
            jThetaX = theta.biasedSign * x1x
            jThetaY = theta.biasedSign * x1y
            jThetaZ = theta.biasedSign * x1z
            ePhi = phi.biasedSign * (clampedPhi - phi)
            jPhiX = phi.biasedSign * y1x
            jPhiY = phi.biasedSign * y1y
            jPhiZ = phi.biasedSign * y1z
        } else {
            constrainSwing = false
            eTheta = 0f
            ePhi = 0f
            jThetaX = 0f
            jThetaY = 0f
            jThetaZ = 0f
            jPhiX = 0f
            jPhiY = 0f
            jPhiZ = 0f
        }

        val constrainTwist: Boolean
        val eTwist: Float
        val psi: Float
        val jTwistX: Float
        val jTwistY: Float
        val jTwistZ: Float

        reject(
            z2x, z2y, z2z, //from
            x1x, x1y, x1z
        ) { cx, cy, cz ->
            if (fma(cx, cx, fma(cy, cy, cz * cz)) < 1e-4) {
                constrainTwist = false
                psi = 0f
                eTwist = 0f
                jTwistX = 0f
                jTwistY = 0f
                jTwistZ = 0f
                return@reject
            }

            normalize(cx, cy, cz) { cx, cy, cz ->
                val cos = dot(cx, cy, cz, x2x, x2y, x2z)
                val sin: Float
                cross(
                    cx, cy, cz,
                    x2x, x2y, x2z,
                ) { cx, cy, cz ->
                    sin = dot(cx, cy, cz, z2x, z2y, z2z)
                }

                psi = atan2(sin, cos)
                if (psi < constraint.minTwistAngle) {
                    constrainTwist = true
                    eTwist = psi - constraint.minTwistAngle
                    jTwistX = -z2x
                    jTwistY = -z2y
                    jTwistZ = -z2z
                } else if (psi > constraint.maxTwistAngle) {
                    constrainTwist = true
                    eTwist = constraint.maxTwistAngle - psi
                    jTwistX = z2x
                    jTwistY = z2y
                    jTwistZ = z2z
                } else {
                    constrainTwist = false
                    eTwist = 0f
                    jTwistX = 0f
                    jTwistY = 0f
                    jTwistZ = 0f
                }
            }
        }

        if (!constrainTwist && !constrainSwing) {
            ConstraintMath.solve3p0rPosition(
                constraint.b1, constraint.b2,
                constraint.r1x, constraint.r1y, constraint.r1z,
                constraint.r2x, constraint.r2y, constraint.r2z,
                erp
            )
        } else if (constrainTwist && !constrainSwing) {
            ConstraintMath.solve3p1rPosition(
                constraint.b1, constraint.b2,
                constraint.r1x, constraint.r1y, constraint.r1z,
                constraint.r2x, constraint.r2y, constraint.r2z,
                -jTwistX, -jTwistY, -jTwistZ,
                eTwist,
                erp,
            )
        } else if (!constrainTwist) {
            ConstraintMath.solve3p2rPosition(
                constraint.b1, constraint.b2,
                constraint.r1x, constraint.r1y, constraint.r1z,
                constraint.r2x, constraint.r2y, constraint.r2z,
                -jThetaX, -jThetaY, -jThetaZ,
                -jPhiX, -jPhiY, -jPhiZ,
                eTheta, ePhi,
                erp,
            )
        } else {
            ConstraintMath.solve3p3rPosition(
                constraint.b1, constraint.b2,
                constraint.r1x, constraint.r1y, constraint.r1z,
                constraint.r2x, constraint.r2y, constraint.r2z,
                -jThetaX, -jThetaY, -jThetaZ,
                -jPhiX, -jPhiY, -jPhiZ,
                -jTwistX, -jTwistY, -jTwistZ,
                eTheta, ePhi, eTwist,
                erp,
            )
        }
    }
}