package com.ixume.udar.physics.hinge

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.ConstraintSolver
import org.joml.Math
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

    private val _tempV = Vector3d()

    fun setup(constraints: List<HingeConstraint>) {
        dt = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.hingeConstraint.bias
        slop = Udar.CONFIG.hingeConstraint.slop
        carryover = Udar.CONFIG.hingeConstraint.carryover
        relaxation = Udar.CONFIG.hingeConstraint.relaxation
        frictionTorque = Udar.CONFIG.hingeConstraint.frictionTorque

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

            val im1 = b1.inverseMass.toFloat()
            val im2 = b2.inverseMass.toFloat()

            val ii1 = b1.inverseInertia
            val ii2 = b2.inverseInertia

            b1.q.transform(constraint.p1x.toDouble(), constraint.p1y.toDouble(), constraint.p1z.toDouble(), _tempV)

            val lr1x = _tempV.x.toFloat()
            val lr1y = _tempV.y.toFloat()
            val lr1z = _tempV.z.toFloat()

            val r1x = lr1x + b1.pos.x.toFloat()
            val r1y = lr1y + b1.pos.y.toFloat()
            val r1z = lr1z + b1.pos.z.toFloat()

            b2.q.transform(constraint.p2x.toDouble(), constraint.p2y.toDouble(), constraint.p2z.toDouble(), _tempV)

            val lr2x = _tempV.x.toFloat()
            val lr2y = _tempV.y.toFloat()
            val lr2z = _tempV.z.toFloat()

            val r2x = lr2x + b2.pos.x.toFloat()
            val r2y = lr2y + b2.pos.y.toFloat()
            val r2z = lr2z + b2.pos.z.toFloat()

            var nx = r2x - r1x
            var ny = r2y - r1y
            var nz = r2z - r1z
            val nl = sqrt(nx * nx + ny * ny + nz * nz)
            if (nl <= 1e-6f) {
                nx = 1f
                ny = 0f
                nz = 0f
            } else {
                nx /= nl
                ny /= nl
                nz /= nl
            }

            b1.q.transform(constraint.a1x.toDouble(), constraint.a1y.toDouble(), constraint.a1z.toDouble(), _tempV)
                .normalize()

            val a1rx = _tempV.x.toFloat()
            val a1ry = _tempV.y.toFloat()
            val a1rz = _tempV.z.toFloat()

            b2.q.transform(constraint.a2x.toDouble(), constraint.a2y.toDouble(), constraint.a2z.toDouble(), _tempV)
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

            b1.q.transform(constraint.n1x.toDouble(), constraint.n1y.toDouble(), constraint.n1z.toDouble(), _tempV)
                .normalize()

            val ux = _tempV.x.toFloat()
            val uy = _tempV.y.toFloat()
            val uz = _tempV.z.toFloat()

            b2.q.transform(constraint.n2x.toDouble(), constraint.n2y.toDouble(), constraint.n2z.toDouble(), _tempV)
                .normalize()

            val n2rx = _tempV.x.toFloat()
            val n2ry = _tempV.y.toFloat()
            val n2rz = _tempV.z.toFloat()

            val m = fma(a1rx, n2rx, fma(a1ry, n2ry, a1rz * n2rz))

            var vx = n2rx - m * a1rx
            var vy = n2ry - m * a1ry
            var vz = n2rx - m * a1rz
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

            val bias1 = bias * nl / dt
            val bias2 = bias * fma(a1rx, bx, fma(a1ry, by, a1rz * bz)) / dt
            val bias3 = bias * fma(a1rx, cx, fma(a1ry, cy, a1rz * cz)) / dt

            val j11x = -nx
            val j11y = -ny
            val j11z = -nz

            val j12x = -fma(lr1y, nz, -lr1z * ny)
            val j12y = -fma(lr1z, nx, -lr1x * nz)
            val j12z = -fma(lr1x, ny, -lr1y * nx)

            val j14x = fma(lr2y, nz, -lr2z * ny)
            val j14y = fma(lr2z, nx, -lr2x * nz)
            val j14z = fma(lr2x, ny, -lr2y * nx)

            val j22x = -fma(by, a1rz, -bz * a1ry)
            val j22y = -fma(bz, a1rx, -bx * a1rz)
            val j22z = -fma(bx, a1ry, -by * a1rx)

            val j32x = -fma(cy, a1rz, -cz * a1ry)
            val j32y = -fma(cz, a1rx, -cx * a1rz)
            val j32z = -fma(cx, a1ry, -cy * a1rx)

            val ej11x = j11x * im1
            val ej11y = j11y * im1
            val ej11z = j11z * im1

            val ej13x = -j11x * im2
            val ej13y = -j11y * im2
            val ej13z = -j11z * im2

            val ej12x = fma(ii1.m00.toFloat(), j12x, fma(ii1.m10.toFloat(), j12y, ii1.m20.toFloat() * j12z))
            val ej12y = fma(ii1.m01.toFloat(), j12x, fma(ii1.m11.toFloat(), j12y, ii1.m21.toFloat() * j12z))
            val ej12z = fma(ii1.m02.toFloat(), j12x, fma(ii1.m12.toFloat(), j12y, ii1.m22.toFloat() * j12z))

            val ej14x = fma(ii2.m00.toFloat(), j14x, fma(ii2.m10.toFloat(), j14y, ii2.m20.toFloat() * j14z))
            val ej14y = fma(ii2.m01.toFloat(), j14x, fma(ii2.m11.toFloat(), j14y, ii2.m21.toFloat() * j14z))
            val ej14z = fma(ii2.m02.toFloat(), j14x, fma(ii2.m12.toFloat(), j14y, ii2.m22.toFloat() * j14z))

            val ej22x = fma(ii1.m00.toFloat(), j22x, fma(ii1.m10.toFloat(), j22y, ii1.m20.toFloat() * j22z))
            val ej22y = fma(ii1.m01.toFloat(), j22x, fma(ii1.m11.toFloat(), j22y, ii1.m21.toFloat() * j22z))
            val ej22z = fma(ii1.m02.toFloat(), j22x, fma(ii1.m12.toFloat(), j22y, ii1.m22.toFloat() * j22z))

            val ej24x = -fma(ii2.m00.toFloat(), j22x, fma(ii2.m10.toFloat(), j22y, ii2.m20.toFloat() * j22z))
            val ej24y = -fma(ii2.m01.toFloat(), j22x, fma(ii2.m11.toFloat(), j22y, ii2.m21.toFloat() * j22z))
            val ej24z = -fma(ii2.m02.toFloat(), j22x, fma(ii2.m12.toFloat(), j22y, ii2.m22.toFloat() * j22z))

            val ej32x = fma(ii1.m00.toFloat(), j32x, fma(ii1.m10.toFloat(), j32y, ii1.m20.toFloat() * j32z))
            val ej32y = fma(ii1.m01.toFloat(), j32x, fma(ii1.m11.toFloat(), j32y, ii1.m21.toFloat() * j32z))
            val ej32z = fma(ii1.m02.toFloat(), j32x, fma(ii1.m12.toFloat(), j32y, ii1.m22.toFloat() * j32z))

            val ej34x = -fma(ii2.m00.toFloat(), j32x, fma(ii2.m10.toFloat(), j32y, ii2.m20.toFloat() * j32z))
            val ej34y = -fma(ii2.m01.toFloat(), j32x, fma(ii2.m11.toFloat(), j32y, ii2.m21.toFloat() * j32z))
            val ej34z = -fma(ii2.m02.toFloat(), j32x, fma(ii2.m12.toFloat(), j32y, ii2.m22.toFloat() * j32z))

            val d1x = ej12x - ej14x
            val d1y = ej12y - ej14y
            val d1z = ej12z - ej14z

            val d2x = ej22x - ej24x
            val d2y = ej22y - ej24y
            val d2z = ej22z - ej24z

            val d3x = ej32x - ej34x
            val d3y = ej32y - ej34y
            val d3z = ej32z - ej34z

            val k11 =
                im1 +
                fma(ej12x, j12x, fma(ej12y, j12y, ej12z * j12z)) +
                im2 +
                fma(ej14x, j14x, fma(ej14y, j14y, ej14z * j14z))
            val k12 = fma(d1x, j22x, fma(d1y, j22y, d1z * j22z))
            val k13 = fma(d1x, j32x, fma(d1y, j32y, d1z * j32z))

            val k22 = fma(d2x, j22x, fma(d2y, j22y, d2z * j22z))
            val k23 = fma(d2x, j32x, fma(d2y, j32y, d2z * j32z))

            val k33 = fma(d3x, j32x, fma(d3y, j32y, d3z * j32z))

            if (frictionTorque == 0f && theta in min..max) {
                val a = fma(k11, k22, -k12 * k12)
                val b = fma(k13, k12, -k11 * k23)
                val c = fma(k12, k23, -k13 * k22)
                val d = Math.fma(a, k33, Math.fma(b, k23, c * k13))
                val s = 1.0f / d
                val n11 = fma(k22, k33, -k23 * k23) * s
                val n12 = fma(k23, k13, -k12 * k33) * s
                val n13 = c * s

                val n22 = fma(k11, k33, -k13 * k13) * s
                val n23 = b * s

                val n33 = a * s

                unlimitedNumConstraints++
                unlimitedConstraintData.set(
                    constraintIdx = i,
                    cursor = i * UNLIMITED_CONSTRAINT_DATA_SIZE,

                    body1Idx = b1.idx,
                    body2Idx = b2.idx,

                    j11x = j11x,
                    j11y = j11y,
                    j11z = j11z,

                    j12x = j12x,
                    j12y = j12y,
                    j12z = j12z,

                    j14x = j14x,
                    j14y = j14y,
                    j14z = j14z,

                    j22x = j22x,
                    j22y = j22y,
                    j22z = j22z,

                    j32x = j32x,
                    j32y = j32y,
                    j32z = j32z,

                    e11x = ej11x,
                    e11y = ej11y,
                    e11z = ej11z,

                    e12x = ej12x,
                    e12y = ej12y,
                    e12z = ej12z,

                    e13x = ej13x,
                    e13y = ej13y,
                    e13z = ej13z,

                    e14x = ej14x,
                    e14y = ej14y,
                    e14z = ej14z,

                    e22x = ej22x,
                    e22y = ej22y,
                    e22z = ej22z,

                    e24x = ej24x,
                    e24y = ej24y,
                    e24z = ej24z,

                    e32x = ej32x,
                    e32y = ej32y,
                    e32z = ej32z,

                    e34x = ej34x,
                    e34y = ej34y,
                    e34z = ej34z,

                    k11 = n11,
                    k12 = n12,
                    k13 = n13,

                    k22 = n22,
                    k23 = n23,

                    k33 = n33,

                    b1 = bias1,
                    b2 = bias2,
                    b3 = bias3,

                    l1 = 0f,
                    l2 = 0f,
                    l3 = 0f,
                )
            } else {
                val bias4: Float
                val j42x: Float
                val j42y: Float
                val j42z: Float
                if (theta > max) {
                    bias4 = bias * (max - theta) / dt
                    j42x = a1rx
                    j42y = a1ry
                    j42z = a1rz
                } else if (theta < min) {
                    bias4 = bias * (theta - min) / dt
                    j42x = -a1rx
                    j42y = -a1ry
                    j42z = -a1rz
                } else {
                    bias4 = 0f
                    j42x = -a1rx
                    j42y = -a1ry
                    j42z = -a1rz
                }

                val ej42x = fma(ii1.m00.toFloat(), j42x, fma(ii1.m10.toFloat(), j42y, ii1.m20.toFloat() * j42z))
                val ej42y = fma(ii1.m01.toFloat(), j42x, fma(ii1.m11.toFloat(), j42y, ii1.m21.toFloat() * j42z))
                val ej42z = fma(ii1.m02.toFloat(), j42x, fma(ii1.m12.toFloat(), j42y, ii1.m22.toFloat() * j42z))

                val ej44x = -fma(ii2.m00.toFloat(), j42x, fma(ii2.m10.toFloat(), j42y, ii2.m20.toFloat() * j42z))
                val ej44y = -fma(ii2.m01.toFloat(), j42x, fma(ii2.m11.toFloat(), j42y, ii2.m21.toFloat() * j42z))
                val ej44z = -fma(ii2.m02.toFloat(), j42x, fma(ii2.m12.toFloat(), j42y, ii2.m22.toFloat() * j42z))

                val d4x = ej42x - ej44x
                val d4y = ej42y - ej44y
                val d4z = ej42z - ej44z

                val k14 = fma(d1x, j42x, fma(d1y, j42y, d1z * j42z))
                val k24 = fma(d2x, j42x, fma(d2y, j42y, d2z * j42z))
                val k34 = fma(d3x, j42x, fma(d3y, j42y, d3z * j42z))
                val k44 = fma(d4x, j42x, fma(d4y, j42y, d4z * j42z))

                val ac = k11 * k22 - k12 * k12
                val bc = k11 * k23 - k13 * k12
                val cc = k11 * k24 - k14 * k12
                val dc = k12 * k23 - k13 * k22
                val ec = k12 * k24 - k14 * k22
                val fc = k13 * k24 - k14 * k23
                val gc = k13 * k24 - k23 * k14
                val hc = k13 * k34 - k33 * k14
                val ic = k13 * k44 - k34 * k14
                val jc = k23 * k34 - k33 * k24
                val kc = k23 * k44 - k34 * k24
                val lc = k33 * k44 - k34 * k34
                var det = ac * lc - bc * kc + cc * jc + dc * ic - ec * hc + fc * gc

                det = if (abs(det) < 1e-6f) 0f else 1f / det
                val n11 = fma(k22, lc, fma(-k23, kc, k24 * jc)) * det
                val n12 = fma(-k12, lc, fma(k13, kc, -k14 * jc)) * det
                val n13 = fma(k24, fc, fma(-k34, ec, k44 * dc)) * det
                val n14 = fma(-k23, fc, fma(k33, ec, -k34 * dc)) * det

                val n22 = fma(k11, lc, fma(-k13, ic, k14 * hc)) * det
                val n23 = fma(-k14, fc, fma(k34, cc, -k44 * bc)) * det
                val n24 = fma(k13, fc, fma(-k33, cc, k34 * bc)) * det

                val n33 = fma(k14, ec, fma(-k24, cc, k44 * ac)) * det
                val n34 = fma(-k13, ec, fma(k23, cc, -k34 * ac)) * det

                val n44 = fma(k13, dc, fma(-k23, bc, k33 * ac)) * det

                val data = if (theta in min..max) {
                    frictionNumConstraints++
                    frictionConstraintData
                } else {
                    limitedNumConstraints++
                    limitedConstraintData
                }

                data.set(
                    constraintIdx = i,
                    cursor = i * LIMITED_CONSTRAINT_DATA_SIZE,

                    body1Idx = b1.idx,
                    body2Idx = b2.idx,

                    j11x = j11x,
                    j11y = j11y,
                    j11z = j11z,

                    j12x = j12x,
                    j12y = j12y,
                    j12z = j12z,

                    j14x = j14x,
                    j14y = j14y,
                    j14z = j14z,

                    j22x = j22x,
                    j22y = j22y,
                    j22z = j22z,

                    j32x = j32x,
                    j32y = j32y,
                    j32z = j32z,

                    j42x = j42x,
                    j42y = j42y,
                    j42z = j42z,

                    e11x = ej11x,
                    e11y = ej11y,
                    e11z = ej11z,

                    e12x = ej12x,
                    e12y = ej12y,
                    e12z = ej12z,

                    e13x = ej13x,
                    e13y = ej13y,
                    e13z = ej13z,

                    e14x = ej14x,
                    e14y = ej14y,
                    e14z = ej14z,

                    e22x = ej22x,
                    e22y = ej22y,
                    e22z = ej22z,

                    e24x = ej24x,
                    e24y = ej24y,
                    e24z = ej24z,

                    e32x = ej32x,
                    e32y = ej32y,
                    e32z = ej32z,

                    e34x = ej34x,
                    e34y = ej34y,
                    e34z = ej34z,

                    e42x = ej42x,
                    e42y = ej42y,
                    e42z = ej42z,

                    e44x = ej44x,
                    e44y = ej44y,
                    e44z = ej44z,

                    k11 = n11,
                    k12 = n12,
                    k13 = n13,
                    k14 = n14,

                    k22 = n22,
                    k23 = n23,
                    k24 = n24,

                    k33 = n33,
                    k34 = n34,

                    k44 = n44,

                    b1 = bias1,
                    b2 = bias2,
                    b3 = bias3,
                    b4 = bias4,

                    l1 = 0f,
                    l2 = 0f,
                    l3 = 0f,
                    l4 = 0f,
                )
            }

            i++
        }
    }

    private inline fun solveLimited(
        data: LimitedConstraintData,
        numConstraints: Int,
        l4Transform: (lambda: Float) -> Float,
    ) {
        val bodyData = parent.flatBodyData
        data.forEach(
            numConstraints = numConstraints,
        ) { _, b1Idx, b2Idx, rawIdx ->
            val v10 = bodyData[b1Idx * 6 + 0]
            val v11 = bodyData[b1Idx * 6 + 1]
            val v12 = bodyData[b1Idx * 6 + 2]
            val v13 = bodyData[b1Idx * 6 + 3]
            val v14 = bodyData[b1Idx * 6 + 4]
            val v15 = bodyData[b1Idx * 6 + 5]

            val v20 = bodyData[b2Idx * 6 + 0]
            val v21 = bodyData[b2Idx * 6 + 1]
            val v22 = bodyData[b2Idx * 6 + 2]
            val v23 = bodyData[b2Idx * 6 + 3]
            val v24 = bodyData[b2Idx * 6 + 4]
            val v25 = bodyData[b2Idx * 6 + 5]

            val j11x = data[rawIdx + LIMITED_J_OFFSET + 0]
            val j11y = data[rawIdx + LIMITED_J_OFFSET + 1]
            val j11z = data[rawIdx + LIMITED_J_OFFSET + 2]

            val j12x = data[rawIdx + LIMITED_J_OFFSET + 3]
            val j12y = data[rawIdx + LIMITED_J_OFFSET + 4]
            val j12z = data[rawIdx + LIMITED_J_OFFSET + 5]

            val j13x = -j11x
            val j13y = -j11y
            val j13z = -j11z

            val j14x = data[rawIdx + LIMITED_J_OFFSET + 6]
            val j14y = data[rawIdx + LIMITED_J_OFFSET + 7]
            val j14z = data[rawIdx + LIMITED_J_OFFSET + 8]

            val j22x = data[rawIdx + LIMITED_J_OFFSET + 9]
            val j22y = data[rawIdx + LIMITED_J_OFFSET + 10]
            val j22z = data[rawIdx + LIMITED_J_OFFSET + 11]

            val j32x = data[rawIdx + LIMITED_J_OFFSET + 12]
            val j32y = data[rawIdx + LIMITED_J_OFFSET + 13]
            val j32z = data[rawIdx + LIMITED_J_OFFSET + 14]

            val j42x = data[rawIdx + LIMITED_J_OFFSET + 15]
            val j42y = data[rawIdx + LIMITED_J_OFFSET + 16]
            val j42z = data[rawIdx + LIMITED_J_OFFSET + 17]

            val e11x = data[rawIdx + LIMITED_E_OFFSET + 0]
            val e11y = data[rawIdx + LIMITED_E_OFFSET + 1]
            val e11z = data[rawIdx + LIMITED_E_OFFSET + 2]

            val e12x = data[rawIdx + LIMITED_E_OFFSET + 3]
            val e12y = data[rawIdx + LIMITED_E_OFFSET + 4]
            val e12z = data[rawIdx + LIMITED_E_OFFSET + 5]

            val e13x = data[rawIdx + LIMITED_E_OFFSET + 6]
            val e13y = data[rawIdx + LIMITED_E_OFFSET + 7]
            val e13z = data[rawIdx + LIMITED_E_OFFSET + 8]

            val e14x = data[rawIdx + LIMITED_E_OFFSET + 9]
            val e14y = data[rawIdx + LIMITED_E_OFFSET + 10]
            val e14z = data[rawIdx + LIMITED_E_OFFSET + 11]

            val e22x = data[rawIdx + LIMITED_E_OFFSET + 12]
            val e22y = data[rawIdx + LIMITED_E_OFFSET + 13]
            val e22z = data[rawIdx + LIMITED_E_OFFSET + 14]

            val e24x = data[rawIdx + LIMITED_E_OFFSET + 15]
            val e24y = data[rawIdx + LIMITED_E_OFFSET + 16]
            val e24z = data[rawIdx + LIMITED_E_OFFSET + 17]

            val e32x = data[rawIdx + LIMITED_E_OFFSET + 18]
            val e32y = data[rawIdx + LIMITED_E_OFFSET + 19]
            val e32z = data[rawIdx + LIMITED_E_OFFSET + 20]

            val e34x = data[rawIdx + LIMITED_E_OFFSET + 21]
            val e34y = data[rawIdx + LIMITED_E_OFFSET + 22]
            val e34z = data[rawIdx + LIMITED_E_OFFSET + 23]

            val e42x = data[rawIdx + LIMITED_E_OFFSET + 24]
            val e42y = data[rawIdx + LIMITED_E_OFFSET + 25]
            val e42z = data[rawIdx + LIMITED_E_OFFSET + 26]

            val e44x = data[rawIdx + LIMITED_E_OFFSET + 27]
            val e44y = data[rawIdx + LIMITED_E_OFFSET + 28]
            val e44z = data[rawIdx + LIMITED_E_OFFSET + 29]

            val k11 = data[rawIdx + LIMITED_K_OFFSET + 0]
            val k12 = data[rawIdx + LIMITED_K_OFFSET + 1]
            val k13 = data[rawIdx + LIMITED_K_OFFSET + 2]
            val k14 = data[rawIdx + LIMITED_K_OFFSET + 3]

            val k22 = data[rawIdx + LIMITED_K_OFFSET + 4]
            val k23 = data[rawIdx + LIMITED_K_OFFSET + 5]
            val k24 = data[rawIdx + LIMITED_K_OFFSET + 6]

            val k33 = data[rawIdx + LIMITED_K_OFFSET + 7]
            val k34 = data[rawIdx + LIMITED_K_OFFSET + 8]

            val k44 = data[rawIdx + LIMITED_K_OFFSET + 9]

            val b1 = data[rawIdx + LIMITED_B_OFFSET + 0]
            val b2 = data[rawIdx + LIMITED_B_OFFSET + 1]
            val b3 = data[rawIdx + LIMITED_B_OFFSET + 2]
            val b4 = data[rawIdx + LIMITED_B_OFFSET + 3]

            val t1 = data[rawIdx + LIMITED_L_OFFSET + 0]
            val t2 = data[rawIdx + LIMITED_L_OFFSET + 1]
            val t3 = data[rawIdx + LIMITED_L_OFFSET + 2]
            val t4 = data[rawIdx + LIMITED_L_OFFSET + 3]

            val jv1 =
                fma(
                    j11x, v10,
                    fma(
                        j11y, v11,
                        fma(
                            j11z, v12,
                            fma(
                                j12x, v13,
                                fma(
                                    j12y, v14,
                                    fma(
                                        j12z, v15,
                                        fma(
                                            j13x, v20,
                                            fma(
                                                j13y, v21,
                                                fma(
                                                    j13z, v22,
                                                    fma(
                                                        j14x, v23,
                                                        fma(
                                                            j14y, v24,
                                                            fma(
                                                                j14z, v25,
                                                                b1
                                                            )
                                                        )
                                                    )
                                                )
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                )

            val jv2 =
                fma(
                    j22x, v13,
                    fma(
                        j22y, v14,
                        fma(
                            j22z, v15,
                            fma(
                                -j22x, v23,
                                fma(
                                    -j22y, v24,
                                    fma(
                                        -j22z, v25,
                                        b2
                                    )
                                )
                            )
                        )
                    )
                )

            val jv3 =
                fma(
                    j32x, v13,
                    fma(
                        j32y, v14,
                        fma(
                            j32z, v15,
                            fma(
                                -j32x, v23,
                                fma(
                                    -j32y, v24,
                                    fma(
                                        -j32z, v25,
                                        b3
                                    )
                                )
                            )
                        )
                    )
                )

            val jv4 =
                fma(
                    j42x, v13,
                    fma(
                        j42y, v14,
                        fma(
                            j42z, v15,
                            fma(
                                -j42x, v24,
                                fma(
                                    -j42y, v24,
                                    fma(
                                        -j42z, v25,
                                        b4
                                    )
                                )
                            )
                        )
                    )
                )

            val l1 =
                t1 - fma(
                    jv1, k11,
                    fma(
                        jv2, k12,
                        fma(
                            jv3, k13,
                            jv4 * k14
                        )
                    )
                )

            val l2 =
                t2 - fma(
                    jv1, k12,
                    fma(
                        jv2, k22,
                        fma(
                            jv3, k23,
                            jv4 * k24
                        )
                    )
                )

            val l3 =
                t3 - fma(
                    jv1, k13,
                    fma(
                        jv2, k23,
                        fma(
                            jv3, k33,
                            jv4 * k34
                        )
                    )
                )

            val l4 =
                l4Transform(
                    t4 - fma(
                        jv1, k14,
                        fma(
                            jv2, k24,
                            fma(
                                jv3, k34,
                                jv4 * k44
                            )
                        )
                    )
                )

            data[rawIdx + LIMITED_L_OFFSET + 0] = l1
            data[rawIdx + LIMITED_L_OFFSET + 1] = l2
            data[rawIdx + LIMITED_L_OFFSET + 2] = l3
            data[rawIdx + LIMITED_L_OFFSET + 3] = l4

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3
            val d4 = l4 - t4

            bodyData[b1Idx * 6 + 0] += e11x * d1
            bodyData[b1Idx * 6 + 1] += e11y * d1
            bodyData[b1Idx * 6 + 2] += e11z * d1
            bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, fma(e32x, d3, e42x * d4)))
            bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, fma(e32y, d3, e42y * d4)))
            bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, fma(e32z, d3, e42z * d4)))

            bodyData[b2Idx * 6 + 0] += e13x * d1
            bodyData[b2Idx * 6 + 1] += e13y * d1
            bodyData[b2Idx * 6 + 2] += e13z * d1
            bodyData[b1Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, fma(e34x, d3, e44x * d4)))
            bodyData[b1Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, fma(e34y, d3, e44y * d4)))
            bodyData[b1Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, fma(e34z, d3, e44z * d4)))
        }
    }

    fun solve() {
        val bodyData = parent.flatBodyData
        unlimitedConstraintData.forEach(
            numConstraints = unlimitedNumConstraints,
        ) { _, b1Idx, b2Idx, rawIdx ->
            val v10 = bodyData[b1Idx * 6 + 0]
            val v11 = bodyData[b1Idx * 6 + 1]
            val v12 = bodyData[b1Idx * 6 + 2]
            val v13 = bodyData[b1Idx * 6 + 3]
            val v14 = bodyData[b1Idx * 6 + 4]
            val v15 = bodyData[b1Idx * 6 + 5]

            val v20 = bodyData[b2Idx * 6 + 0]
            val v21 = bodyData[b2Idx * 6 + 1]
            val v22 = bodyData[b2Idx * 6 + 2]
            val v23 = bodyData[b2Idx * 6 + 3]
            val v24 = bodyData[b2Idx * 6 + 4]
            val v25 = bodyData[b2Idx * 6 + 5]

            val j11x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 0]
            val j11y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 1]
            val j11z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 2]

            val j12x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 3]
            val j12y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 4]
            val j12z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 5]

            val j13x = -j11x
            val j13y = -j11y
            val j13z = -j11z

            val j14x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 6]
            val j14y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 7]
            val j14z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 8]

            val j22x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 9]
            val j22y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 10]
            val j22z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 11]

            val j32x = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 12]
            val j32y = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 13]
            val j32z = unlimitedConstraintData[rawIdx + UNLIMITED_J_OFFSET + 14]

            val e11x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 0]
            val e11y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 1]
            val e11z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 2]

            val e12x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 3]
            val e12y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 4]
            val e12z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 5]

            val e13x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 6]
            val e13y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 7]
            val e13z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 8]

            val e14x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 9]
            val e14y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 10]
            val e14z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 11]

            val e22x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 12]
            val e22y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 13]
            val e22z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 14]

            val e24x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 15]
            val e24y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 16]
            val e24z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 17]

            val e32x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 18]
            val e32y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 19]
            val e32z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 20]

            val e34x = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 21]
            val e34y = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 22]
            val e34z = unlimitedConstraintData[rawIdx + UNLIMITED_E_OFFSET + 23]

            val k11 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 0]
            val k12 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 1]
            val k13 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 2]

            val k22 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 3]
            val k23 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 4]

            val k33 = unlimitedConstraintData[rawIdx + UNLIMITED_K_OFFSET + 5]

            val b1 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 0]
            val b2 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 1]
            val b3 = unlimitedConstraintData[rawIdx + UNLIMITED_B_OFFSET + 2]

            val t1 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 0]
            val t2 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 1]
            val t3 = unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 2]

            val jv1 =
                fma(
                    j11x, v10,
                    fma(
                        j11y, v11,
                        fma(
                            j11z, v12,
                            fma(
                                j12x, v13,
                                fma(
                                    j12y, v14,
                                    fma(
                                        j12z, v15,
                                        fma(
                                            j13x, v20,
                                            fma(
                                                j13y, v21,
                                                fma(
                                                    j13z, v22,
                                                    fma(
                                                        j14x, v23,
                                                        fma(
                                                            j14y, v24,
                                                            fma(
                                                                j14z, v25,
                                                                b1
                                                            )
                                                        )
                                                    )
                                                )
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                )

            val jv2 =
                fma(
                    j22x, v13,
                    fma(
                        j22y, v14,
                        fma(
                            j22z, v15,
                            fma(
                                -j22x, v23,
                                fma(
                                    -j22y, v24,
                                    fma(
                                        -j22z, v25,
                                        b2
                                    )
                                )
                            )
                        )
                    )
                )

            val jv3 =
                fma(
                    j32x, v13,
                    fma(
                        j32y, v14,
                        fma(
                            j32z, v15,
                            fma(
                                -j32x, v23,
                                fma(
                                    -j32y, v24,
                                    fma(
                                        -j32z, v25,
                                        b3
                                    )
                                )
                            )
                        )
                    )
                )

            val l1 =
                t1 - fma(
                    jv1, k11,
                    fma(
                        jv2, k12,
                        jv3 * k13
                    )
                )

            val l2 =
                t2 - fma(
                    jv1, k12,
                    fma(
                        jv2, k22,
                        jv3 * k23
                    )
                )

            val l3 =
                t3 - fma(
                    jv1, k13,
                    fma(
                        jv2, k23,
                        jv3 * k33
                    )
                )

            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 0] = l1
            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 1] = l2
            unlimitedConstraintData[rawIdx + UNLIMITED_L_OFFSET + 2] = l3

            val d1 = l1 - t1
            val d2 = l2 - t2
            val d3 = l3 - t3

            bodyData[b1Idx * 6 + 0] += e11x * d1
            bodyData[b1Idx * 6 + 1] += e11y * d1
            bodyData[b1Idx * 6 + 2] += e11z * d1
            bodyData[b1Idx * 6 + 3] += fma(e12x, d1, fma(e22x, d2, e32x * d3))
            bodyData[b1Idx * 6 + 4] += fma(e12y, d1, fma(e22y, d2, e32y * d3))
            bodyData[b1Idx * 6 + 5] += fma(e12z, d1, fma(e22z, d2, e32z * d3))

            bodyData[b2Idx * 6 + 0] += e13x * d1
            bodyData[b2Idx * 6 + 1] += e13y * d1
            bodyData[b2Idx * 6 + 2] += e13z * d1
            bodyData[b2Idx * 6 + 3] += fma(e14x, d1, fma(e24x, d2, e34x * d3))
            bodyData[b2Idx * 6 + 4] += fma(e14y, d1, fma(e24y, d2, e34y * d3))
            bodyData[b2Idx * 6 + 5] += fma(e14z, d1, fma(e24z, d2, e34z * d3))
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

        j11x: Float,
        j11y: Float,
        j11z: Float,

        j12x: Float,
        j12y: Float,
        j12z: Float,

        j14x: Float,
        j14y: Float,
        j14z: Float,

        j22x: Float,
        j22y: Float,
        j22z: Float,

        j32x: Float,
        j32y: Float,
        j32z: Float,

        e11x: Float,
        e11y: Float,
        e11z: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e13x: Float,
        e13y: Float,
        e13z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

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
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)
        this[cursor + 3] = j11x
        this[cursor + 4] = j11y
        this[cursor + 5] = j11z
        this[cursor + 6] = j12x
        this[cursor + 7] = j12y
        this[cursor + 8] = j12z
        this[cursor + 9] = j14x
        this[cursor + 10] = j14y
        this[cursor + 11] = j14z
        this[cursor + 12] = j22x
        this[cursor + 13] = j22y
        this[cursor + 14] = j22z
        this[cursor + 15] = j32x
        this[cursor + 16] = j32y
        this[cursor + 17] = j32z
        this[cursor + 18] = e11x
        this[cursor + 19] = e11y
        this[cursor + 20] = e11z
        this[cursor + 21] = e12x
        this[cursor + 22] = e12y
        this[cursor + 23] = e12z
        this[cursor + 24] = e13x
        this[cursor + 25] = e13y
        this[cursor + 26] = e13z
        this[cursor + 27] = e14x
        this[cursor + 28] = e14y
        this[cursor + 29] = e14z
        this[cursor + 30] = e22x
        this[cursor + 31] = e22y
        this[cursor + 32] = e22z
        this[cursor + 33] = e24x
        this[cursor + 34] = e24y
        this[cursor + 35] = e24z
        this[cursor + 36] = e32x
        this[cursor + 37] = e32y
        this[cursor + 38] = e32z
        this[cursor + 39] = e34x
        this[cursor + 40] = e34y
        this[cursor + 41] = e34z
        this[cursor + 42] = k11
        this[cursor + 43] = k12
        this[cursor + 44] = k13
        this[cursor + 45] = k22
        this[cursor + 46] = k23
        this[cursor + 47] = k33
        this[cursor + 48] = b1
        this[cursor + 49] = b2
        this[cursor + 50] = b3
        this[cursor + 51] = l1
        this[cursor + 52] = l2
        this[cursor + 53] = l3
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

        j11x: Float,
        j11y: Float,
        j11z: Float,

        j12x: Float,
        j12y: Float,
        j12z: Float,

        j14x: Float,
        j14y: Float,
        j14z: Float,

        j22x: Float,
        j22y: Float,
        j22z: Float,

        j32x: Float,
        j32y: Float,
        j32z: Float,

        j42x: Float,
        j42y: Float,
        j42z: Float,

        e11x: Float,
        e11y: Float,
        e11z: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e13x: Float,
        e13y: Float,
        e13z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

        e34x: Float,
        e34y: Float,
        e34z: Float,

        e42x: Float,
        e42y: Float,
        e42z: Float,

        e44x: Float,
        e44y: Float,
        e44z: Float,

        k11: Float,
        k12: Float,
        k13: Float,
        k14: Float,

        k22: Float,
        k23: Float,
        k24: Float,

        k33: Float,
        k34: Float,

        k44: Float,

        b1: Float,
        b2: Float,
        b3: Float,
        b4: Float,

        l1: Float,
        l2: Float,
        l3: Float,
        l4: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)
        this[cursor + 3] = j11x
        this[cursor + 4] = j11y
        this[cursor + 5] = j11z
        this[cursor + 6] = j12x
        this[cursor + 7] = j12y
        this[cursor + 8] = j12z
        this[cursor + 9] = j14x
        this[cursor + 10] = j14y
        this[cursor + 11] = j14z
        this[cursor + 12] = j22x
        this[cursor + 13] = j22y
        this[cursor + 14] = j22z
        this[cursor + 15] = j32x
        this[cursor + 16] = j32y
        this[cursor + 17] = j32z
        this[cursor + 18] = j42x
        this[cursor + 19] = j42y
        this[cursor + 20] = j42z
        this[cursor + 21] = e11x
        this[cursor + 22] = e11y
        this[cursor + 23] = e11z
        this[cursor + 24] = e12x
        this[cursor + 25] = e12y
        this[cursor + 26] = e12z
        this[cursor + 27] = e13x
        this[cursor + 28] = e13y
        this[cursor + 29] = e13z
        this[cursor + 30] = e14x
        this[cursor + 31] = e14y
        this[cursor + 32] = e14z
        this[cursor + 33] = e22x
        this[cursor + 34] = e22y
        this[cursor + 35] = e22z
        this[cursor + 36] = e24x
        this[cursor + 37] = e24y
        this[cursor + 38] = e24z
        this[cursor + 39] = e32x
        this[cursor + 40] = e32y
        this[cursor + 41] = e32z
        this[cursor + 42] = e34x
        this[cursor + 43] = e34y
        this[cursor + 44] = e34z
        this[cursor + 45] = e42x
        this[cursor + 46] = e42y
        this[cursor + 47] = e42z
        this[cursor + 48] = e44x
        this[cursor + 49] = e44y
        this[cursor + 50] = e44z
        this[cursor + 51] = k11
        this[cursor + 52] = k12
        this[cursor + 53] = k13
        this[cursor + 54] = k14
        this[cursor + 55] = k22
        this[cursor + 56] = k23
        this[cursor + 57] = k24
        this[cursor + 58] = k33
        this[cursor + 59] = k34
        this[cursor + 60] = k44
        this[cursor + 61] = b1
        this[cursor + 62] = b2
        this[cursor + 63] = b3
        this[cursor + 64] = b4
        this[cursor + 65] = l1
        this[cursor + 66] = l2
        this[cursor + 67] = l3
        this[cursor + 68] = l4
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

/*
[]UnlimitedConstraint
[]LimitedConstraint

UnlimitedConstraint = {
  constraintIdx: Int,
  body1Idx: Int
  body2Idx: Int
  
  j11x: Float
  j11y: Float
  j11z: Float
  
  j12x: Float
  j12y: Float
  j12z: Float
  
  j14x: Float
  j14y: Float
  j14z: Float
  
  j22x: Float
  j22y: Float
  j22z: Float
  
  j32x: Float
  j32y: Float
  j32z: Float
  
  e11x: Float
  e11y: Float
  e11z: Float
  
  e12x: Float
  e12y: Float
  e12z: Float
  
  e13x: Float
  e13y: Float
  e13z: Float
  
  e14x: Float
  e14y: Float
  e14z: Float
  
  e22x: Float
  e22y: Float
  e22z: Float
  
  e24x: Float
  e24y: Float
  e24z: Float
  
  e32x: Float
  e32y: Float
  e32z: Float
  
  e34x: Float
  e34y: Float
  e34z: Float
  
  k11: Float
  k12: Float
  k13: Float
  
  k22: Float
  k23: Float
  
  k33: Float
  
  b1: Float
  b2: Float
  b3: Float
  
  l1: Float
  l2: Float
  l3: Float
}

LimitedConstraint = {
  constraintIdx: Int,
  body1Idx: Int
  body2Idx: Int
  
  j11x: Float
  j11y: Float
  j11z: Float
  
  j12x: Float
  j12y: Float
  j12z: Float
  
  j14x: Float
  j14y: Float
  j14z: Float
  
  j22x: Float
  j22y: Float
  j22z: Float
  
  j32x: Float
  j32y: Float
  j32z: Float
  
  j42x: Float
  j42y: Float
  j42z: Float
  
  e11x: Float
  e11y: Float
  e11z: Float
  
  e12x: Float
  e12y: Float
  e12z: Float
  
  e13x: Float
  e13y: Float
  e13z: Float
  
  e14x: Float
  e14y: Float
  e14z: Float
  
  e22x: Float
  e22y: Float
  e22z: Float
  
  e24x: Float
  e24y: Float
  e24z: Float
  
  e32x: Float
  e32y: Float
  e32z: Float
  
  e34x: Float
  e34y: Float
  e34z: Float
  
  e42x: Float
  e42y: Float
  e42z: Float
  
  e44x: Float
  e44y: Float
  e44z: Float
  
  k11: Float
  k12: Float
  k13: Float
  k14: Float
  
  k22: Float
  k23: Float
  k24: Float
  
  k33: Float
  k34: Float
  
  k44: Float
  
  b1: Float
  b2: Float
  b3: Float
  b4: Float
  
  l1: Float
  l2: Float
  l3: Float
  l4: Float
}
 */
private const val UNLIMITED_CONSTRAINT_DATA_SIZE = 54
private const val UNLIMITED_J_OFFSET = 3
private const val UNLIMITED_E_OFFSET = 18
private const val UNLIMITED_K_OFFSET = 42
private const val UNLIMITED_B_OFFSET = 48
private const val UNLIMITED_L_OFFSET = 51

private const val LIMITED_CONSTRAINT_DATA_SIZE = 69
private const val LIMITED_J_OFFSET = 3
private const val LIMITED_E_OFFSET = 21
private const val LIMITED_K_OFFSET = 51
private const val LIMITED_B_OFFSET = 61
private const val LIMITED_L_OFFSET = 65
