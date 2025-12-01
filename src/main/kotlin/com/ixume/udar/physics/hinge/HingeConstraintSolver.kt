package com.ixume.udar.physics.hinge

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.ConstraintMath
import com.ixume.udar.physics.constraint.ConstraintSolver
import org.joml.Quaterniond
import org.joml.Vector3d
import java.lang.Math.fma
import kotlin.math.*

class HingeConstraintSolver(val parent: ConstraintSolver) {
    private var unlimitedConstraintData = UnlimitedConstraintData(0)
    private var unlimitedNumConstraints = 0
    private var limitedConstraintData = LimitedConstraintData(0)
    private var limitedNumConstraints = 0
    private var frictionConstraintData = LimitedConstraintData(0)
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
            limitedConstraintData = LimitedConstraintData(s)
        }

        if (frictionConstraintData.sizeFor(totalNumConstraints) != frictionConstraintData.value.size) {
            val s = frictionConstraintData.sizeFor(totalNumConstraints)
            frictionConstraintData = LimitedConstraintData(s)
        }

        if (unlimitedConstraintData.sizeFor(totalNumConstraints) != unlimitedConstraintData.value.size) {
            val s = unlimitedConstraintData.sizeFor(totalNumConstraints)
            unlimitedConstraintData = UnlimitedConstraintData(s)
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
                    cursor = unlimitedNumConstraints++ * UNLIMITED_CONSTRAINT_DATA_SIZE,

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

                val data: LimitedConstraintData
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
                    cursor = cursor * LIMITED_CONSTRAINT_DATA_SIZE,

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
        unlimitedConstraintData.forEach(
            numConstraints = unlimitedNumConstraints,
        ) { _, b1Idx, b2Idx, rawIdx ->
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

            val b1 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 0]
            val b2 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 1]
            val b3 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 2]
            val b4 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 3]
            val b5 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 4]

            val r1x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 0]
            val r1y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 1]
            val r1z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 2]

            val r2x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 3]
            val r2y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 4]
            val r2z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 5]

            val j42x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 6]
            val j42y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 7]
            val j42z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 8]

            val j52x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 9]
            val j52y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 10]
            val j52z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 11]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)
            val jv4 =
                -fma(j42x, v4, fma(j42y, v5, fma(j42z, v6, fma(-j42x, v10, fma(-j42y, v11, fma(-j42z, v12, b4))))))
            val jv5 =
                -fma(j52x, v4, fma(j52y, v5, fma(j52z, v6, fma(-j52x, v10, fma(-j52y, v11, fma(-j52z, v12, b5))))))

            val im1 = unlimitedConstraintData[rawIdx + UNLIMITED_IM_OFFSET + 0]
            val im2 = unlimitedConstraintData[rawIdx + UNLIMITED_IM_OFFSET + 1]

            val e12x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 0]
            val e12y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 1]
            val e12z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 2]

            val e22x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 3]
            val e22y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 4]
            val e22z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 5]

            val e32x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 6]
            val e32y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 7]
            val e32z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 8]

            val e14x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 9]
            val e14y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 10]
            val e14z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 11]

            val e24x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 12]
            val e24y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 13]
            val e24z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 14]

            val e34x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 15]
            val e34y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 16]
            val e34z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 17]

            val e42x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 18]
            val e42y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 19]
            val e42z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 20]

            val e44x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 21]
            val e44y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 22]
            val e44z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 23]

            val e52x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 24]
            val e52y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 25]
            val e52z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 26]

            val e54x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 27]
            val e54y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 28]
            val e54z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 29]

            val k11 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 0]
            val k12 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 1]
            val k13 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 2]
            val k14 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 3]
            val k15 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 4]

            val k22 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 5]
            val k23 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 6]
            val k24 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 7]
            val k25 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 8]

            val k33 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 9]
            val k34 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 10]
            val k35 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 11]

            val k44 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 12]
            val k45 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 13]

            val k55 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 14]

            val t1 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 0]
            val t2 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 1]
            val t3 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 2]
            val t4 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 3]
            val t5 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 4]

            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float
            val l5: Float

            ConstraintMath.solveSymmetric5x5(
                k11, k12, k13, k14, k15,
                k22, k23, k24, k25,
                k33, k34, k35,
                k44, k45,
                k55,

                jv1, jv2, jv3, jv4, jv5
            ) { s1, s2, s3, s4, s5 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
                l4 = t4 + s4
                l5 = t5 + s5
            }

            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 0] = l1
            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 1] = l2
            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 2] = l3
            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 3] = l4
            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 4] = l5

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3
            val d4 = l4 - t4
            val d5 = l5 - t5

            bodyData[b1Idx * 6 + 0] += -im1 * d1
            bodyData[b1Idx * 6 + 1] += -im1 * d2
            bodyData[b1Idx * 6 + 2] += -im1 * d3
            bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, fma(e32x, d3, fma(e42x, d4, e52x * d5))))
            bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, fma(e32y, d3, fma(e42y, d4, e52y * d5))))
            bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, fma(e32z, d3, fma(e42z, d4, e52z * d5))))

            bodyData[b2Idx * 6 + 0] += im2 * d1
            bodyData[b2Idx * 6 + 1] += im2 * d2
            bodyData[b2Idx * 6 + 2] += im2 * d3
            bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, fma(e34x, d3, fma(e44x, d4, e54x * d5))))
            bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, fma(e34y, d3, fma(e44y, d4, e54y * d5))))
            bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, fma(e34z, d3, fma(e44z, d4, e54z * d5))))
        }

        solveLimited(
            data = limitedConstraintData,
            numConstraints = limitedNumConstraints,
        ) { l -> max(0f, l) }

        solveLimited(
            data = frictionConstraintData,
            numConstraints = frictionNumConstraints,
        ) { l -> max(-frictionTorque, min(frictionTorque, l)) }
    }

    private inline fun solveLimited(
        data: LimitedConstraintData,
        numConstraints: Int,
        l6Transform: (lambda: Float) -> Float,
    ) {
        val bodyData = parent.flatBodyData
        data.forEach(
            numConstraints = numConstraints,
        ) { _, b1Idx, b2Idx, rawIdx ->
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

            val b1 = data[rawIdx + LIMITED_B_OFFSET + 0]
            val b2 = data[rawIdx + LIMITED_B_OFFSET + 1]
            val b3 = data[rawIdx + LIMITED_B_OFFSET + 2]
            val b4 = data[rawIdx + LIMITED_B_OFFSET + 3]
            val b5 = data[rawIdx + LIMITED_B_OFFSET + 4]
            val b6 = data[rawIdx + LIMITED_B_OFFSET + 5]

            val r1x = data[rawIdx + LIMITED_J_OFFSET + 0]
            val r1y = data[rawIdx + LIMITED_J_OFFSET + 1]
            val r1z = data[rawIdx + LIMITED_J_OFFSET + 2]

            val r2x = data[rawIdx + LIMITED_J_OFFSET + 3]
            val r2y = data[rawIdx + LIMITED_J_OFFSET + 4]
            val r2z = data[rawIdx + LIMITED_J_OFFSET + 5]

            val j42x = data[rawIdx + LIMITED_J_OFFSET + 6]
            val j42y = data[rawIdx + LIMITED_J_OFFSET + 7]
            val j42z = data[rawIdx + LIMITED_J_OFFSET + 8]

            val j52x = data[rawIdx + LIMITED_J_OFFSET + 9]
            val j52y = data[rawIdx + LIMITED_J_OFFSET + 10]
            val j52z = data[rawIdx + LIMITED_J_OFFSET + 11]

            val j62x = data[rawIdx + LIMITED_J_OFFSET + 12]
            val j62y = data[rawIdx + LIMITED_J_OFFSET + 13]
            val j62z = data[rawIdx + LIMITED_J_OFFSET + 14]

            val jv1 = -(-v1 + -r1z * v5 + r1y * v6 + v7 + r2z * v11 + -r2y * v12 + b1)
            val jv2 = -(-v2 + r1z * v4 + -r1x * v6 + v8 + -r2z * v10 + r2x * v12 + b2)
            val jv3 = -(-v3 + -r1y * v4 + r1x * v5 + v9 + r2y * v10 + -r2x * v11 + b3)
            val jv4 =
                -fma(j42x, v4, fma(j42y, v5, fma(j42z, v6, fma(-j42x, v10, fma(-j42y, v11, fma(-j42z, v12, b4))))))
            val jv5 =
                -fma(j52x, v4, fma(j52y, v5, fma(j52z, v6, fma(-j52x, v10, fma(-j52y, v11, fma(-j52z, v12, b5))))))
            val jv6 =
                -fma(j62x, v4, fma(j62y, v5, fma(j62z, v6, fma(-j62x, v10, fma(-j62y, v11, fma(-j62z, v12, b6))))))

            val im1 = data[rawIdx + LIMITED_IM_OFFSET + 0]
            val im2 = data[rawIdx + LIMITED_IM_OFFSET + 1]

            val e12x = data[rawIdx + LIMITED_E_OFFSET + 0]
            val e12y = data[rawIdx + LIMITED_E_OFFSET + 1]
            val e12z = data[rawIdx + LIMITED_E_OFFSET + 2]

            val e22x = data[rawIdx + LIMITED_E_OFFSET + 3]
            val e22y = data[rawIdx + LIMITED_E_OFFSET + 4]
            val e22z = data[rawIdx + LIMITED_E_OFFSET + 5]

            val e32x = data[rawIdx + LIMITED_E_OFFSET + 6]
            val e32y = data[rawIdx + LIMITED_E_OFFSET + 7]
            val e32z = data[rawIdx + LIMITED_E_OFFSET + 8]

            val e14x = data[rawIdx + LIMITED_E_OFFSET + 9]
            val e14y = data[rawIdx + LIMITED_E_OFFSET + 10]
            val e14z = data[rawIdx + LIMITED_E_OFFSET + 11]

            val e24x = data[rawIdx + LIMITED_E_OFFSET + 12]
            val e24y = data[rawIdx + LIMITED_E_OFFSET + 13]
            val e24z = data[rawIdx + LIMITED_E_OFFSET + 14]

            val e34x = data[rawIdx + LIMITED_E_OFFSET + 15]
            val e34y = data[rawIdx + LIMITED_E_OFFSET + 16]
            val e34z = data[rawIdx + LIMITED_E_OFFSET + 17]

            val e42x = data[rawIdx + LIMITED_E_OFFSET + 18]
            val e42y = data[rawIdx + LIMITED_E_OFFSET + 19]
            val e42z = data[rawIdx + LIMITED_E_OFFSET + 20]

            val e44x = data[rawIdx + LIMITED_E_OFFSET + 21]
            val e44y = data[rawIdx + LIMITED_E_OFFSET + 22]
            val e44z = data[rawIdx + LIMITED_E_OFFSET + 23]

            val e52x = data[rawIdx + LIMITED_E_OFFSET + 24]
            val e52y = data[rawIdx + LIMITED_E_OFFSET + 25]
            val e52z = data[rawIdx + LIMITED_E_OFFSET + 26]

            val e54x = data[rawIdx + LIMITED_E_OFFSET + 27]
            val e54y = data[rawIdx + LIMITED_E_OFFSET + 28]
            val e54z = data[rawIdx + LIMITED_E_OFFSET + 29]

            val e62x = data[rawIdx + LIMITED_E_OFFSET + 30]
            val e62y = data[rawIdx + LIMITED_E_OFFSET + 31]
            val e62z = data[rawIdx + LIMITED_E_OFFSET + 32]

            val e64x = data[rawIdx + LIMITED_E_OFFSET + 33]
            val e64y = data[rawIdx + LIMITED_E_OFFSET + 34]
            val e64z = data[rawIdx + LIMITED_E_OFFSET + 35]

            val k11 = data[rawIdx + LIMITED_K_OFFSET + 0]
            val k12 = data[rawIdx + LIMITED_K_OFFSET + 1]
            val k13 = data[rawIdx + LIMITED_K_OFFSET + 2]
            val k14 = data[rawIdx + LIMITED_K_OFFSET + 3]
            val k15 = data[rawIdx + LIMITED_K_OFFSET + 4]
            val k16 = data[rawIdx + LIMITED_K_OFFSET + 5]

            val k22 = data[rawIdx + LIMITED_K_OFFSET + 6]
            val k23 = data[rawIdx + LIMITED_K_OFFSET + 7]
            val k24 = data[rawIdx + LIMITED_K_OFFSET + 8]
            val k25 = data[rawIdx + LIMITED_K_OFFSET + 9]
            val k26 = data[rawIdx + LIMITED_K_OFFSET + 10]

            val k33 = data[rawIdx + LIMITED_K_OFFSET + 11]
            val k34 = data[rawIdx + LIMITED_K_OFFSET + 12]
            val k35 = data[rawIdx + LIMITED_K_OFFSET + 13]
            val k36 = data[rawIdx + LIMITED_K_OFFSET + 14]

            val k44 = data[rawIdx + LIMITED_K_OFFSET + 15]
            val k45 = data[rawIdx + LIMITED_K_OFFSET + 16]
            val k46 = data[rawIdx + LIMITED_K_OFFSET + 17]

            val k55 = data[rawIdx + LIMITED_K_OFFSET + 18]
            val k56 = data[rawIdx + LIMITED_K_OFFSET + 19]

            val k66 = data[rawIdx + LIMITED_K_OFFSET + 20]

            val t1 = data[rawIdx + LIMITED_L_OFFSET + 0]
            val t2 = data[rawIdx + LIMITED_L_OFFSET + 1]
            val t3 = data[rawIdx + LIMITED_L_OFFSET + 2]
            val t4 = data[rawIdx + LIMITED_L_OFFSET + 3]
            val t5 = data[rawIdx + LIMITED_L_OFFSET + 4]
            val t6 = data[rawIdx + LIMITED_L_OFFSET + 5]

            val l1: Float
            val l2: Float
            val l3: Float
            val l4: Float
            val l5: Float
            val l6: Float

            ConstraintMath.solveSymmetric6x6(
                k11, k12, k13, k14, k15, k16,
                k22, k23, k24, k25, k26,
                k33, k34, k35, k36,
                k44, k45, k46,
                k55, k56,
                k66,

                jv1, jv2, jv3, jv4, jv5, jv6
            ) { s1, s2, s3, s4, s5, s6 ->
                l1 = t1 + s1
                l2 = t2 + s2
                l3 = t3 + s3
                l4 = t4 + s4
                l5 = t5 + s5
                l6 = l6Transform(t6 + s6)
            }

            data[rawIdx + LIMITED_L_OFFSET + 0] = l1
            data[rawIdx + LIMITED_L_OFFSET + 1] = l2
            data[rawIdx + LIMITED_L_OFFSET + 2] = l3
            data[rawIdx + LIMITED_L_OFFSET + 3] = l4
            data[rawIdx + LIMITED_L_OFFSET + 4] = l5
            data[rawIdx + LIMITED_L_OFFSET + 5] = l6

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3
            val d4 = l4 - t4
            val d5 = l5 - t5
            val d6 = l6 - t6

            bodyData[b1Idx * 6 + 0] += -im1 * d1
            bodyData[b1Idx * 6 + 1] += -im1 * d2
            bodyData[b1Idx * 6 + 2] += -im1 * d3
            bodyData[b1Idx * 6 + 3] += fma(
                e12x,
                d1,
                fma(e22x, d2, fma(e32x, d3, fma(e42x, d4, fma(e52x, d5, e62x * d6))))
            )
            bodyData[b1Idx * 6 + 4] += fma(
                e12y,
                d1,
                fma(e22y, d2, fma(e32y, d3, fma(e42y, d4, fma(e52y, d5, e62y * d6))))
            )
            bodyData[b1Idx * 6 + 5] += fma(
                e12z,
                d1,
                fma(e22z, d2, fma(e32z, d3, fma(e42z, d4, fma(e52z, d5, e62z * d6))))
            )

            bodyData[b2Idx * 6 + 0] += im2 * d1
            bodyData[b2Idx * 6 + 1] += im2 * d2
            bodyData[b2Idx * 6 + 2] += im2 * d3
            bodyData[b2Idx * 6 + 3] += fma(
                e14x,
                d1,
                fma(e24x, d2, fma(e34x, d3, fma(e44x, d4, fma(e54x, d5, e64x * d6))))
            )
            bodyData[b2Idx * 6 + 4] += fma(
                e14y,
                d1,
                fma(e24y, d2, fma(e34y, d3, fma(e44y, d4, fma(e54y, d5, e64y * d6))))
            )
            bodyData[b2Idx * 6 + 5] += fma(
                e14z,
                d1,
                fma(e24z, d2, fma(e34z, d3, fma(e44z, d4, fma(e54z, d5, e64z * d6))))
            )
        }
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

            ConstraintMath.solveSymmetric5x5(
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

            ConstraintMath.solveSymmetric6x6(
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

@JvmInline
private value class UnlimitedConstraintData(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * UNLIMITED_CONSTRAINT_DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        constraintIdx: Int,
        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        j42x: Float,
        j42y: Float,
        j42z: Float,

        j52x: Float,
        j52y: Float,
        j52z: Float,

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

        e42x: Float,
        e42y: Float,
        e42z: Float,

        e44x: Float,
        e44y: Float,
        e44z: Float,

        e52x: Float,
        e52y: Float,
        e52z: Float,

        e54x: Float,
        e54y: Float,
        e54z: Float,

        k11: Float,
        k12: Float,
        k13: Float,
        k14: Float,
        k15: Float,

        k22: Float,
        k23: Float,
        k24: Float,
        k25: Float,

        k33: Float,
        k34: Float,
        k35: Float,

        k44: Float,
        k45: Float,

        k55: Float,

        b1: Float,
        b2: Float,
        b3: Float,
        b4: Float,
        b5: Float,

        l1: Float,
        l2: Float,
        l3: Float,
        l4: Float,
        l5: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)

        this[cursor + 3] = r1x
        this[cursor + 4] = r1y
        this[cursor + 5] = r1z

        this[cursor + 6] = r2x
        this[cursor + 7] = r2y
        this[cursor + 8] = r2z

        this[cursor + 9] = j42x
        this[cursor + 10] = j42y
        this[cursor + 11] = j42z

        this[cursor + 12] = j52x
        this[cursor + 13] = j52y
        this[cursor + 14] = j52z

        this[cursor + 15] = im1
        this[cursor + 16] = im2

        this[cursor + 17] = e12x
        this[cursor + 18] = e12y
        this[cursor + 19] = e12z

        this[cursor + 20] = e22x
        this[cursor + 21] = e22y
        this[cursor + 22] = e22z

        this[cursor + 23] = e32x
        this[cursor + 24] = e32y
        this[cursor + 25] = e32z

        this[cursor + 26] = e14x
        this[cursor + 27] = e14y
        this[cursor + 28] = e14z

        this[cursor + 29] = e24x
        this[cursor + 30] = e24y
        this[cursor + 31] = e24z

        this[cursor + 32] = e34x
        this[cursor + 33] = e34y
        this[cursor + 34] = e34z

        this[cursor + 35] = e42x
        this[cursor + 36] = e42y
        this[cursor + 37] = e42z

        this[cursor + 38] = e44x
        this[cursor + 39] = e44y
        this[cursor + 40] = e44z

        this[cursor + 41] = e52x
        this[cursor + 42] = e52y
        this[cursor + 43] = e52z

        this[cursor + 44] = e54x
        this[cursor + 45] = e54y
        this[cursor + 46] = e54z

        this[cursor + 47] = k11
        this[cursor + 48] = k12
        this[cursor + 49] = k13
        this[cursor + 50] = k14
        this[cursor + 51] = k15

        this[cursor + 52] = k22
        this[cursor + 53] = k23
        this[cursor + 54] = k24
        this[cursor + 55] = k25

        this[cursor + 56] = k33
        this[cursor + 57] = k34
        this[cursor + 58] = k35

        this[cursor + 59] = k44
        this[cursor + 60] = k45

        this[cursor + 61] = k55

        this[cursor + 62] = b1
        this[cursor + 63] = b2
        this[cursor + 64] = b3
        this[cursor + 65] = b4
        this[cursor + 66] = b5

        this[cursor + 67] = l1
        this[cursor + 68] = l2
        this[cursor + 69] = l3
        this[cursor + 70] = l4
        this[cursor + 71] = l5
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            constraintIdx: Int,

            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * UNLIMITED_CONSTRAINT_DATA_SIZE + 0].toRawBits(),
                this[i * UNLIMITED_CONSTRAINT_DATA_SIZE + 1].toRawBits(),
                this[i * UNLIMITED_CONSTRAINT_DATA_SIZE + 2].toRawBits(),
                i * UNLIMITED_CONSTRAINT_DATA_SIZE,
            )

            i++
        }
    }
}

@JvmInline
private value class LimitedConstraintData(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * LIMITED_CONSTRAINT_DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        constraintIdx: Int,
        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        j42x: Float,
        j42y: Float,
        j42z: Float,

        j52x: Float,
        j52y: Float,
        j52z: Float,

        j62x: Float,
        j62y: Float,
        j62z: Float,

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

        e42x: Float,
        e42y: Float,
        e42z: Float,

        e44x: Float,
        e44y: Float,
        e44z: Float,

        e52x: Float,
        e52y: Float,
        e52z: Float,

        e54x: Float,
        e54y: Float,
        e54z: Float,

        e62x: Float,
        e62y: Float,
        e62z: Float,

        e64x: Float,
        e64y: Float,
        e64z: Float,

        k11: Float,
        k12: Float,
        k13: Float,
        k14: Float,
        k15: Float,
        k16: Float,

        k22: Float,
        k23: Float,
        k24: Float,
        k25: Float,
        k26: Float,

        k33: Float,
        k34: Float,
        k35: Float,
        k36: Float,

        k44: Float,
        k45: Float,
        k46: Float,

        k55: Float,
        k56: Float,

        k66: Float,

        b1: Float,
        b2: Float,
        b3: Float,
        b4: Float,
        b5: Float,
        b6: Float,

        l1: Float,
        l2: Float,
        l3: Float,
        l4: Float,
        l5: Float,
        l6: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)

        this[cursor + 3] = r1x
        this[cursor + 4] = r1y
        this[cursor + 5] = r1z

        this[cursor + 6] = r2x
        this[cursor + 7] = r2y
        this[cursor + 8] = r2z

        this[cursor + 9] = j42x
        this[cursor + 10] = j42y
        this[cursor + 11] = j42z

        this[cursor + 12] = j52x
        this[cursor + 13] = j52y
        this[cursor + 14] = j52z

        this[cursor + 15] = j62x
        this[cursor + 16] = j62y
        this[cursor + 17] = j62z

        this[cursor + 18] = im1
        this[cursor + 19] = im2

        this[cursor + 20] = e12x
        this[cursor + 21] = e12y
        this[cursor + 22] = e12z

        this[cursor + 23] = e22x
        this[cursor + 24] = e22y
        this[cursor + 25] = e22z

        this[cursor + 26] = e32x
        this[cursor + 27] = e32y
        this[cursor + 28] = e32z

        this[cursor + 29] = e14x
        this[cursor + 30] = e14y
        this[cursor + 31] = e14z

        this[cursor + 32] = e24x
        this[cursor + 33] = e24y
        this[cursor + 34] = e24z

        this[cursor + 35] = e34x
        this[cursor + 36] = e34y
        this[cursor + 37] = e34z

        this[cursor + 38] = e42x
        this[cursor + 39] = e42y
        this[cursor + 40] = e42z

        this[cursor + 41] = e44x
        this[cursor + 42] = e44y
        this[cursor + 43] = e44z

        this[cursor + 44] = e52x
        this[cursor + 45] = e52y
        this[cursor + 46] = e52z

        this[cursor + 47] = e54x
        this[cursor + 48] = e54y
        this[cursor + 49] = e54z

        this[cursor + 50] = e62x
        this[cursor + 51] = e62y
        this[cursor + 52] = e62z

        this[cursor + 53] = e64x
        this[cursor + 54] = e64y
        this[cursor + 55] = e64z

        this[cursor + 56] = k11
        this[cursor + 57] = k12
        this[cursor + 58] = k13
        this[cursor + 59] = k14
        this[cursor + 60] = k15
        this[cursor + 61] = k16

        this[cursor + 62] = k22
        this[cursor + 63] = k23
        this[cursor + 64] = k24
        this[cursor + 65] = k25
        this[cursor + 66] = k26

        this[cursor + 67] = k33
        this[cursor + 68] = k34
        this[cursor + 69] = k35
        this[cursor + 70] = k36

        this[cursor + 71] = k44
        this[cursor + 72] = k45
        this[cursor + 73] = k46

        this[cursor + 74] = k55
        this[cursor + 75] = k56

        this[cursor + 76] = k66

        this[cursor + 77] = b1
        this[cursor + 78] = b2
        this[cursor + 79] = b3
        this[cursor + 80] = b4
        this[cursor + 81] = b5
        this[cursor + 82] = b6

        this[cursor + 83] = l1
        this[cursor + 84] = l2
        this[cursor + 85] = l3
        this[cursor + 86] = l4
        this[cursor + 87] = l5
        this[cursor + 88] = l6
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            constraintIdx: Int,

            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * LIMITED_CONSTRAINT_DATA_SIZE + 0].toRawBits(),
                this[i * LIMITED_CONSTRAINT_DATA_SIZE + 1].toRawBits(),
                this[i * LIMITED_CONSTRAINT_DATA_SIZE + 2].toRawBits(),
                i * LIMITED_CONSTRAINT_DATA_SIZE,
            )

            i++
        }
    }
}

private const val UNLIMITED_CONSTRAINT_DATA_SIZE = 72
private const val UNLIMITED_J_OFFSET = 3
private const val UNLIMITED_IM_OFFSET = 15
private const val UNLIMITED_E_OFFSET = 17
private const val UNLIMITED_K_OFFSET = 47
private const val UNLIMITED_B_OFFSET = 62
private const val UNLIMITED_L_OFFSET = 67

private const val LIMITED_CONSTRAINT_DATA_SIZE = 89
private const val LIMITED_J_OFFSET = 3
private const val LIMITED_IM_OFFSET = 18
private const val LIMITED_E_OFFSET = 20
private const val LIMITED_K_OFFSET = 56
private const val LIMITED_B_OFFSET = 77
private const val LIMITED_L_OFFSET = 83