package com.ixume.udar.physics.hinge

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.*
import org.joml.Quaterniond
import org.joml.Vector3d
import java.lang.Math.fma
import kotlin.math.*

class HingeConstraintSolver(val parent: ConstraintSolver) {
    private var unlimitedConstraintData = ConstraintData3p2r(0)
    private var unlimitedNumConstraints = 0
    private var limitedConstraintData = ConstraintData3p3r(0)
    private var limitedNumConstraints = 0
    private var frictionConstraintData = ConstraintData3p3r(0)
    private var frictionNumConstraints = 0

    private var dt = Udar.CONFIG.timeStep.toFloat()
    private var bias = Udar.CONFIG.hingeConstraint.bias
    private var slop = Udar.CONFIG.hingeConstraint.slop
    private var carryover = Udar.CONFIG.hingeConstraint.carryover
    private var relaxation = Udar.CONFIG.hingeConstraint.relaxation
    private var frictionTorque = Udar.CONFIG.hingeConstraint.frictionTorque
    private fun effectiveERP() =
        1f - (1.0 - Udar.CONFIG.hingeConstraint.erp.toDouble()).pow(1.0 / Udar.CONFIG.collision.posIterations.toDouble())
            .toFloat()

    private var erp = effectiveERP()

    private lateinit var rawConstraints: List<HingeConstraint>

    private val _tempV = Vector3d()
    private val _tempQ = Quaterniond()

    fun setup(constraints: List<HingeConstraint>) {
        dt = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.hingeConstraint.bias
        slop = Udar.CONFIG.hingeConstraint.slop
        carryover = Udar.CONFIG.hingeConstraint.carryover
        relaxation = Udar.CONFIG.hingeConstraint.relaxation
        frictionTorque = Udar.CONFIG.hingeConstraint.frictionTorque
        erp = effectiveERP()

        rawConstraints = constraints

        val totalNumConstraints = constraints.size

        if (limitedConstraintData.sizeFor(totalNumConstraints) != limitedConstraintData.value.size) {
            val s = limitedConstraintData.sizeFor(totalNumConstraints)
            limitedConstraintData = ConstraintData3p3r(s)
        }

        if (frictionConstraintData.sizeFor(totalNumConstraints) != frictionConstraintData.value.size) {
            val s = frictionConstraintData.sizeFor(totalNumConstraints)
            frictionConstraintData = ConstraintData3p3r(s)
        }

        if (unlimitedConstraintData.sizeFor(totalNumConstraints) != unlimitedConstraintData.value.size) {
            val s = unlimitedConstraintData.sizeFor(totalNumConstraints)
            unlimitedConstraintData = ConstraintData3p2r(s)
        }

        unlimitedNumConstraints = 0
        limitedNumConstraints = 0
        frictionNumConstraints = 0

        var i = 0
        while (i < totalNumConstraints) {
            val constraint = constraints[i]

            val b1 = constraint.b1
            val b2 = constraint.b2

            val q1 = b1.q
            val q2 = b2.q

            val im1 = b1.inverseMass.toFloat()
            val im2 = b2.inverseMass.toFloat()

            val ii1 = b1.inverseInertia
            val ii2 = b2.inverseInertia

            q1.transform(constraint.p1x.toDouble(), constraint.p1y.toDouble(), constraint.p1z.toDouble(), _tempV)

            val r1x = _tempV.x.toFloat()
            val r1y = _tempV.y.toFloat()
            val r1z = _tempV.z.toFloat()

            val p1x = r1x + b1.pos.x.toFloat()
            val p1y = r1y + b1.pos.y.toFloat()
            val p1z = r1z + b1.pos.z.toFloat()

            q2.transform(constraint.p2x.toDouble(), constraint.p2y.toDouble(), constraint.p2z.toDouble(), _tempV)

            val r2x = _tempV.x.toFloat()
            val r2y = _tempV.y.toFloat()
            val r2z = _tempV.z.toFloat()

            val p2x = r2x + b2.pos.x.toFloat()
            val p2y = r2y + b2.pos.y.toFloat()
            val p2z = r2z + b2.pos.z.toFloat()

            val nx = p2x - p1x
            val ny = p2y - p1y
            val nz = p2z - p1z

            q1.transform(constraint.a1x.toDouble(), constraint.a1y.toDouble(), constraint.a1z.toDouble(), _tempV)
                .normalize()

            val a1rx = _tempV.x.toFloat()
            val a1ry = _tempV.y.toFloat()
            val a1rz = _tempV.z.toFloat()

            q2.transform(constraint.a2x.toDouble(), constraint.a2y.toDouble(), constraint.a2z.toDouble(), _tempV)
                .normalize()

            val a2rx = _tempV.x.toFloat()
            val a2ry = _tempV.y.toFloat()
            val a2rz = _tempV.z.toFloat()

            val aa2rx = abs(a2rx)
            val aa2ry = abs(a2ry)
            val aa2rz = abs(a2rz)

            var bx: Float
            var by: Float
            var bz: Float
            if (aa2rx <= aa2ry && aa2rx <= aa2rz) {
                bx = 0f
                by = -a2rz
                bz = a2ry
            } else if (aa2ry <= aa2rx && aa2ry <= aa2rz) {
                bx = a2rz
                by = 0f
                bz = -a2rx
            } else {
                bx = -a2ry
                by = a2rx
                bz = 0f
            }
            val bl = sqrt((bx * bx + by * by + bz * bz).toDouble()).toFloat()
            if (bl <= 1e-6f) {
                bx = 0f
                by = 1f
                bz = 0f
            } else {
                bx /= bl
                by /= bl
                bz /= bl
            }

            val cx = a2ry * bz - a2rz * by
            val cy = a2rz * bx - a2rx * bz
            val cz = a2rx * by - a2ry * bx

            q1.transform(constraint.n1x.toDouble(), constraint.n1y.toDouble(), constraint.n1z.toDouble(), _tempV)
                .normalize()

            val ux = _tempV.x.toFloat()
            val uy = _tempV.y.toFloat()
            val uz = _tempV.z.toFloat()

            q2.transform(constraint.n2x.toDouble(), constraint.n2y.toDouble(), constraint.n2z.toDouble(), _tempV)
                .normalize()

            val n2rx = _tempV.x.toFloat()
            val n2ry = _tempV.y.toFloat()
            val n2rz = _tempV.z.toFloat()

            val m = fma(a1rx, n2rx, fma(a1ry, n2ry, a1rz * n2rz))

            var vx = n2rx - m * a1rx
            var vy = n2ry - m * a1ry
            var vz = n2rz - m * a1rz
            val vl = sqrt(vx * vx + vy * vy + vz * vz)
            if (vl < 1e-6f) {
                vx = 1f
                vy = 0f
                vz = 0f
            } else {
                vx /= vl
                vy /= vl
                vz /= vl
            }

            val sint =
                fma(uy, vz, -uz * vy) * a1rx +
                fma(uz, vx, -ux * vz) * a1ry +
                fma(ux, vy, -uy * vx) * a1rz

            val cost = fma(ux, vx, fma(uy, vy, uz * vz))

            val theta = atan2(sint, cost)
            val min = constraint.min
            val max = constraint.max

            val bias1 = bias * nx / dt
            val bias2 = bias * ny / dt
            val bias3 = bias * nz / dt
            val bias4 = bias * fma(a1rx, bx, fma(a1ry, by, a1rz * bz)) / dt
            val bias5 = bias * fma(a1rx, cx, fma(a1ry, cy, a1rz * cz)) / dt

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

            val j42x = -fma(by, a1rz, -bz * a1ry)
            val j42y = -fma(bz, a1rx, -bx * a1rz)
            val j42z = -fma(bx, a1ry, -by * a1rx)

            val j52x = -fma(cy, a1rz, -cz * a1ry)
            val j52y = -fma(cz, a1rx, -cx * a1rz)
            val j52z = -fma(cx, a1ry, -cy * a1rx)

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

            if (frictionTorque == 0f && theta in min..max) {
                unlimitedConstraintData.set(
                    constraintIdx = i,
                    cursor = unlimitedNumConstraints++ * ConstraintData3p2r.DATA_SIZE,

                    body1Idx = b1.idx,
                    body2Idx = b2.idx,

                    r1x = r1x,
                    r1y = r1y,
                    r1z = r1z,

                    r2x = r2x,
                    r2y = r2y,
                    r2z = r2z,

                    j42x = j42x,
                    j42y = j42y,
                    j42z = j42z,

                    j52x = j52x,
                    j52y = j52y,
                    j52z = j52z,

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

                    e42x = ej42x,
                    e42y = ej42y,
                    e42z = ej42z,

                    e44x = ej44x,
                    e44y = ej44y,
                    e44z = ej44z,

                    e52x = ej52x,
                    e52y = ej52y,
                    e52z = ej52z,

                    e54x = ej54x,
                    e54y = ej54y,
                    e54z = ej54z,

                    k11 = k11,
                    k12 = k12,
                    k13 = k13,
                    k14 = k14,
                    k15 = k15,

                    k22 = k22,
                    k23 = k23,
                    k24 = k24,
                    k25 = k25,

                    k33 = k33,
                    k34 = k34,
                    k35 = k35,

                    k44 = k44,
                    k45 = k45,

                    k55 = k55,

                    b1 = bias1,
                    b2 = bias2,
                    b3 = bias3,
                    b4 = bias4,
                    b5 = bias5,

                    l1 = 0f,
                    l2 = 0f,
                    l3 = 0f,
                    l4 = 0f,
                    l5 = 0f,
                )
            } else {
                val bias6: Float
                val j62x: Float
                val j62y: Float
                val j62z: Float
                if (theta > max) {
                    bias6 = bias * (max - theta) / dt
                    j62x = a1rx
                    j62y = a1ry
                    j62z = a1rz
                } else if (theta < min) {
                    bias6 = bias * (theta - min) / dt
                    j62x = -a1rx
                    j62y = -a1ry
                    j62z = -a1rz
                } else {
                    bias6 = 0f
                    j62x = -a1rx
                    j62y = -a1ry
                    j62z = -a1rz
                }

                val ej62x = fma(ii1.m00.toFloat(), j62x, fma(ii1.m10.toFloat(), j62y, ii1.m20.toFloat() * j62z))
                val ej62y = fma(ii1.m01.toFloat(), j62x, fma(ii1.m11.toFloat(), j62y, ii1.m21.toFloat() * j62z))
                val ej62z = fma(ii1.m02.toFloat(), j62x, fma(ii1.m12.toFloat(), j62y, ii1.m22.toFloat() * j62z))

                val ej64x = -fma(ii2.m00.toFloat(), j62x, fma(ii2.m10.toFloat(), j62y, ii2.m20.toFloat() * j62z))
                val ej64y = -fma(ii2.m01.toFloat(), j62x, fma(ii2.m11.toFloat(), j62y, ii2.m21.toFloat() * j62z))
                val ej64z = -fma(ii2.m02.toFloat(), j62x, fma(ii2.m12.toFloat(), j62y, ii2.m22.toFloat() * j62z))

                val d6x = ej62x - ej64x
                val d6y = ej62y - ej64y
                val d6z = ej62z - ej64z

                val k16 = fma(d1x, j62x, fma(d1y, j62y, d1z * j62z))
                val k26 = fma(d2x, j62x, fma(d2y, j62y, d2z * j62z))
                val k36 = fma(d3x, j62x, fma(d3y, j62y, d3z * j62z))
                val k46 = fma(d4x, j62x, fma(d4y, j62y, d4z * j62z))
                val k56 = fma(d5x, j62x, fma(d5y, j62y, d5z * j62z))
                val k66 = fma(d6x, j62x, fma(d6y, j62y, d6z * j62z))

                val data: ConstraintData3p3r
                val cursor: Int

                if (theta in min..max) {
                    data = frictionConstraintData
                    cursor = frictionNumConstraints++
                } else {
                    data = limitedConstraintData
                    cursor = limitedNumConstraints++
                }

                data.set(
                    constraintIdx = i,
                    cursor = cursor * ConstraintData3p3r.DATA_SIZE,

                    body1Idx = b1.idx,
                    body2Idx = b2.idx,

                    r1x = r1x,
                    r1y = r1y,
                    r1z = r1z,

                    r2x = r2x,
                    r2y = r2y,
                    r2z = r2z,

                    j42x = j42x,
                    j42y = j42y,
                    j42z = j42z,

                    j52x = j52x,
                    j52y = j52y,
                    j52z = j52z,

                    j62x = j62x,
                    j62y = j62y,
                    j62z = j62z,

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

                    e42x = ej42x,
                    e42y = ej42y,
                    e42z = ej42z,

                    e44x = ej44x,
                    e44y = ej44y,
                    e44z = ej44z,

                    e52x = ej52x,
                    e52y = ej52y,
                    e52z = ej52z,

                    e54x = ej54x,
                    e54y = ej54y,
                    e54z = ej54z,

                    e62x = ej62x,
                    e62y = ej62y,
                    e62z = ej62z,

                    e64x = ej64x,
                    e64y = ej64y,
                    e64z = ej64z,

                    k11 = k11,
                    k12 = k12,
                    k13 = k13,
                    k14 = k14,
                    k15 = k15,
                    k16 = k16,

                    k22 = k22,
                    k23 = k23,
                    k24 = k24,
                    k25 = k25,
                    k26 = k26,

                    k33 = k33,
                    k34 = k34,
                    k35 = k35,
                    k36 = k36,

                    k44 = k44,
                    k45 = k45,
                    k46 = k46,

                    k55 = k55,
                    k56 = k56,

                    k66 = k66,

                    b1 = bias1,
                    b2 = bias2,
                    b3 = bias3,
                    b4 = bias4,
                    b5 = bias5,
                    b6 = bias6,

                    l1 = 0f,
                    l2 = 0f,
                    l3 = 0f,
                    l4 = 0f,
                    l5 = 0f,
                    l6 = 0f,
                )
            }

            i++
        }
    }

    fun solveVelocity() {
        val bodyData = parent.flatBodyData
        ConstraintMath.solve3p2rVelocity(
            bodyData,
            unlimitedConstraintData,
            unlimitedNumConstraints,
        )

        ConstraintMath.solve3p3rVelocity(
            bodyData,
            limitedConstraintData,
            limitedNumConstraints,
            l6Transform = { max(0f, it) }
        )

        ConstraintMath.solve3p3rVelocity(
            bodyData,
            frictionConstraintData,
            frictionNumConstraints,
            l6Transform = { max(-frictionTorque, min(frictionTorque, it)) }
        )
    }

    fun solvePosition() {
        unlimitedConstraintData.forEach(
            numConstraints = unlimitedNumConstraints,
        ) { constraintIdx, _, _, _ ->
            val constraint = rawConstraints[constraintIdx]
            solvePosition(constraint)
        }

        frictionConstraintData.forEach(
            numConstraints = frictionNumConstraints,
        ) { constraintIdx, _, _, _ ->
            val constraint = rawConstraints[constraintIdx]
            solvePosition(constraint)
        }

        limitedConstraintData.forEach(
            numConstraints = limitedNumConstraints,
        ) { constraintIdx, _, _, _ ->
            val constraint = rawConstraints[constraintIdx]
            solvePosition(constraint)
        }
    }

    private fun solvePosition(constraint: HingeConstraint) {
        val b1 = constraint.b1
        val b2 = constraint.b2

        val q1 = b1.q
        val q2 = b2.q

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        q1.transform(constraint.p1x.toDouble(), constraint.p1y.toDouble(), constraint.p1z.toDouble(), _tempV)

        val r1x = _tempV.x.toFloat()
        val r1y = _tempV.y.toFloat()
        val r1z = _tempV.z.toFloat()

        val p1x = r1x + b1.pos.x.toFloat()
        val p1y = r1y + b1.pos.y.toFloat()
        val p1z = r1z + b1.pos.z.toFloat()

        q2.transform(constraint.p2x.toDouble(), constraint.p2y.toDouble(), constraint.p2z.toDouble(), _tempV)

        val r2x = _tempV.x.toFloat()
        val r2y = _tempV.y.toFloat()
        val r2z = _tempV.z.toFloat()

        val p2x = r2x + b2.pos.x.toFloat()
        val p2y = r2y + b2.pos.y.toFloat()
        val p2z = r2z + b2.pos.z.toFloat()

        val nx = p2x - p1x
        val ny = p2y - p1y
        val nz = p2z - p1z

        q1.transform(constraint.a1x.toDouble(), constraint.a1y.toDouble(), constraint.a1z.toDouble(), _tempV)
            .normalize()

        val a1rx = _tempV.x.toFloat()
        val a1ry = _tempV.y.toFloat()
        val a1rz = _tempV.z.toFloat()

        q2.transform(constraint.a2x.toDouble(), constraint.a2y.toDouble(), constraint.a2z.toDouble(), _tempV)
            .normalize()

        val a2rx = _tempV.x.toFloat()
        val a2ry = _tempV.y.toFloat()
        val a2rz = _tempV.z.toFloat()

        val aa2rx = abs(a2rx)
        val aa2ry = abs(a2ry)
        val aa2rz = abs(a2rz)

        var bx: Float
        var by: Float
        var bz: Float
        if (aa2rx <= aa2ry && aa2rx <= aa2rz) {
            bx = 0f
            by = -a2rz
            bz = a2ry
        } else if (aa2ry <= aa2rx && aa2ry <= aa2rz) {
            bx = a2rz
            by = 0f
            bz = -a2rx
        } else {
            bx = -a2ry
            by = a2rx
            bz = 0f
        }
        val bl = sqrt((bx * bx + by * by + bz * bz).toDouble()).toFloat()
        if (bl <= 1e-6f) {
            bx = 0f
            by = 1f
            bz = 0f
        } else {
            bx /= bl
            by /= bl
            bz /= bl
        }

        val cx = a2ry * bz - a2rz * by
        val cy = a2rz * bx - a2rx * bz
        val cz = a2rx * by - a2ry * bx

        q1.transform(constraint.n1x.toDouble(), constraint.n1y.toDouble(), constraint.n1z.toDouble(), _tempV)
            .normalize()

        val ux = _tempV.x.toFloat()
        val uy = _tempV.y.toFloat()
        val uz = _tempV.z.toFloat()

        q2.transform(constraint.n2x.toDouble(), constraint.n2y.toDouble(), constraint.n2z.toDouble(), _tempV)
            .normalize()

        val n2rx = _tempV.x.toFloat()
        val n2ry = _tempV.y.toFloat()
        val n2rz = _tempV.z.toFloat()

        val m = fma(a1rx, n2rx, fma(a1ry, n2ry, a1rz * n2rz))

        var vx = n2rx - m * a1rx
        var vy = n2ry - m * a1ry
        var vz = n2rz - m * a1rz
        val vl = sqrt(vx * vx + vy * vy + vz * vz)
        if (vl < 1e-6f) {
            vx = 1f
            vy = 0f
            vz = 0f
        } else {
            vx /= vl
            vy /= vl
            vz /= vl
        }

        val sint =
            fma(uy, vz, -uz * vy) * a1rx +
            fma(uz, vx, -ux * vz) * a1ry +
            fma(ux, vy, -uy * vx) * a1rz

        val cost = fma(ux, vx, fma(uy, vy, uz * vz))

        val theta = atan2(sint, cost)
        val min = constraint.min
        val max = constraint.max

        val e1 = nx
        val e2 = ny
        val e3 = nz
        val e4 = fma(a1rx, bx, fma(a1ry, by, a1rz * bz))
        val e5 = fma(a1rx, cx, fma(a1ry, cy, a1rz * cz))

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

        val j42x = -fma(by, a1rz, -bz * a1ry)
        val j42y = -fma(bz, a1rx, -bx * a1rz)
        val j42z = -fma(bx, a1ry, -by * a1rx)

        val j52x = -fma(cy, a1rz, -cz * a1ry)
        val j52y = -fma(cz, a1rx, -cx * a1rz)
        val j52z = -fma(cx, a1ry, -cy * a1rx)

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

        val d1: Float
        val d2: Float
        val d3: Float
        val d4: Float
        val d5: Float
        val d6: Float

        val d7: Float
        val d8: Float
        val d9: Float
        val d10: Float
        val d11: Float
        val d12: Float

        if (theta in min..max) {
            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float
            val l5: Float

            MatrixMath.solveSymmetric5x5(
                k11, k12, k13, k14, k15,
                k22, k23, k24, k25,
                k33, k34, k35,
                k44, k45,
                k55,

                -e1, -e2, -e3, -e4, -e5
            ) { s1, s2, s3, s4, s5 ->
                l1 = s1
                l2 = s2
                l3 = s3
                l4 = s4
                l5 = s5
            }

            d1 = -im1 * l1
            d2 = -im1 * l2
            d3 = -im1 * l3
            d4 = fma(ej12x, l1, fma(ej22x, l2, fma(ej32x, l3, fma(ej42x, l4, ej52x * l5))))
            d5 = fma(ej12y, l1, fma(ej22y, l2, fma(ej32y, l3, fma(ej42y, l4, ej52y * l5))))
            d6 = fma(ej12z, l1, fma(ej22z, l2, fma(ej32z, l3, fma(ej42z, l4, ej52z * l5))))

            d7 = im2 * l1
            d8 = im2 * l2
            d9 = im2 * l3
            d10 = fma(ej14x, l1, fma(ej24x, l2, fma(ej34x, l3, fma(ej44x, l4, ej54x * l5))))
            d11 = fma(ej14y, l1, fma(ej24y, l2, fma(ej34y, l3, fma(ej44y, l4, ej54y * l5))))
            d12 = fma(ej14z, l1, fma(ej24z, l2, fma(ej34z, l3, fma(ej44z, l4, ej54z * l5))))
        } else {
            val e6: Float
            val j62x: Float
            val j62y: Float
            val j62z: Float
            if (theta > max) {
                e6 = max - theta
                j62x = a1rx
                j62y = a1ry
                j62z = a1rz
            } else if (theta < min) {
                e6 = theta - min
                j62x = -a1rx
                j62y = -a1ry
                j62z = -a1rz
            } else {
                e6 = 0f
                j62x = -a1rx
                j62y = -a1ry
                j62z = -a1rz
            }

            val ej62x = fma(ii1.m00.toFloat(), j62x, fma(ii1.m10.toFloat(), j62y, ii1.m20.toFloat() * j62z))
            val ej62y = fma(ii1.m01.toFloat(), j62x, fma(ii1.m11.toFloat(), j62y, ii1.m21.toFloat() * j62z))
            val ej62z = fma(ii1.m02.toFloat(), j62x, fma(ii1.m12.toFloat(), j62y, ii1.m22.toFloat() * j62z))

            val ej64x = -fma(ii2.m00.toFloat(), j62x, fma(ii2.m10.toFloat(), j62y, ii2.m20.toFloat() * j62z))
            val ej64y = -fma(ii2.m01.toFloat(), j62x, fma(ii2.m11.toFloat(), j62y, ii2.m21.toFloat() * j62z))
            val ej64z = -fma(ii2.m02.toFloat(), j62x, fma(ii2.m12.toFloat(), j62y, ii2.m22.toFloat() * j62z))

            val d6x = ej62x - ej64x
            val d6y = ej62y - ej64y
            val d6z = ej62z - ej64z

            val k16 = fma(d1x, j62x, fma(d1y, j62y, d1z * j62z))
            val k26 = fma(d2x, j62x, fma(d2y, j62y, d2z * j62z))
            val k36 = fma(d3x, j62x, fma(d3y, j62y, d3z * j62z))
            val k46 = fma(d4x, j62x, fma(d4y, j62y, d4z * j62z))
            val k56 = fma(d5x, j62x, fma(d5y, j62y, d5z * j62z))
            val k66 = fma(d6x, j62x, fma(d6y, j62y, d6z * j62z))
            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float
            val l5: Float
            val l6: Float

            MatrixMath.solveSymmetric6x6(
                k11, k12, k13, k14, k15, k16,
                k22, k23, k24, k25, k26,
                k33, k34, k35, k36,
                k44, k45, k46,
                k55, k56,
                k66,

                -e1, -e2, -e3, -e4, -e5, -e6
            ) { s1, s2, s3, s4, s5, s6 ->
                l1 = s1
                l2 = s2
                l3 = s3
                l4 = s4
                l5 = s5
                l6 = s6
            }

            d1 = -im1 * l1
            d2 = -im1 * l2
            d3 = -im1 * l3
            d4 = fma(
                ej12x,
                l1,
                fma(ej22x, l2, fma(ej32x, l3, fma(ej42x, l4, fma(ej52x, l5, ej62x * l6))))
            )
            d5 = fma(
                ej12y,
                l1,
                fma(ej22y, l2, fma(ej32y, l3, fma(ej42y, l4, fma(ej52y, l5, ej62y * l6))))
            )
            d6 = fma(
                ej12z,
                l1,
                fma(ej22z, l2, fma(ej32z, l3, fma(ej42z, l4, fma(ej52z, l5, ej62z * l6))))
            )

            d7 = im2 * l1
            d8 = im2 * l2
            d9 = im2 * l3
            d10 = fma(
                ej14x,
                l1,
                fma(ej24x, l2, fma(ej34x, l3, fma(ej44x, l4, fma(ej54x, l5, ej64x * l6))))
            )
            d11 = fma(
                ej14y,
                l1,
                fma(ej24y, l2, fma(ej34y, l3, fma(ej44y, l4, fma(ej54y, l5, ej64y * l6))))
            )
            d12 = fma(
                ej14z,
                l1,
                fma(ej24z, l2, fma(ej34z, l3, fma(ej44z, l4, fma(ej54z, l5, ej64z * l6))))
            )
        }

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
    }
}