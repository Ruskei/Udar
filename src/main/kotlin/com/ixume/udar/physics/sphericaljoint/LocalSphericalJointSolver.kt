package com.ixume.udar.physics.sphericaljoint

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.*
import org.joml.Quaternionf
import org.joml.Vector3f
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/*
distance constraint between 2 points P_a and P_b on bodies a and b can be modeled as:
P_a - P_b == 0
... (P_a_x - P_b_x)^2 = 0
(P_a_x - P_b_x) * ((V_a_x + (O_a x R_a).x) - (V_b_x + (O_b x R_b).x)) + ... = 0
r . (V_a + O_a x R_a - V_B - O_b x R_b) = 0
r . V_a + r . O_a x R_a - r . V_b - r . O_b x R_b
J = [r, R_a x r, -r, -R_b x r]
V = [V_a, O_a, V_b, O_b]
J(V + dV) = 0
dV = l J

JV + JlJ = 0
(-JV) / (JJ) = l
 */
class LocalSphericalJointSolver(val constraintSolver: LocalConstraintSolver) {
    private var timeStep = Udar.Companion.CONFIG.timeStep.toFloat()
    private var bias = Udar.Companion.CONFIG.sphericalJoint.bias.toFloat()
    private var slop = Udar.Companion.CONFIG.sphericalJoint.slop.toFloat()

    private val constraints = constraintSolver.physicsWorld.sphericalJointConstraints.constraints
    private var toSolve = 0

    private var jointData: FloatArray = FloatArray(1)

    private val _q = Quaternionf()
    private val _c_j1Temp = Vector3f()
    private val _c_j3Temp = Vector3f()
    private val _c_vec3 = Vector3f()

    fun setup() {
        timeStep = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.sphericalJoint.bias.toFloat()
        slop = Udar.CONFIG.sphericalJoint.slop.toFloat()

        constructFlatConstraintData()
    }

    private fun constructFlatConstraintData() {
        val numc = constraints.size()
        val relevantData = run {
            if (jointData.size < numc * SPHERICAL_JOINT_DATA_FLOATS) {
                jointData = FloatArray(max(jointData.size * 2, numc * SPHERICAL_JOINT_DATA_FLOATS))
            }

            jointData
        }

        val n = FloatBuffer.wrap(relevantData)

        toSolve = 0
        constraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, rax, ray, raz, rbx, rby, rbz ->
            val bodyA = constraintSolver.physicsWorld.activeBodies.fastGet(bodyAIdx)!!
            val bodyB = constraintSolver.physicsWorld.activeBodies.fastGet(bodyBIdx)!!
            // relative positions are local, so first rotate them
            // then find real distance by also transforming them into world space to get `r`
            _c_vec3.set(rax, ray, raz).rotate(_q.set(bodyA.q))
            val _rax = _c_vec3.x
            val _ray = _c_vec3.y
            val _raz = _c_vec3.z
            val r_rax = _rax + bodyA.pos.x.toFloat()
            val r_ray = _ray + bodyA.pos.y.toFloat()
            val r_raz = _raz + bodyA.pos.z.toFloat()
            _c_vec3.set(rbx, rby, rbz).rotate(_q.set(bodyB.q))
            val _rbx = _c_vec3.x
            val _rby = _c_vec3.y
            val _rbz = _c_vec3.z
            val r_rbx = _rbx + bodyB.pos.x.toFloat()
            val r_rby = _rby + bodyB.pos.y.toFloat()
            val r_rbz = _rbz + bodyB.pos.z.toFloat()

            val rx = r_rax - r_rbx
            val ry = r_ray - r_rby
            val rz = r_raz - r_rbz

            val j1x = fma(_ray, rz, -_raz * ry)
            val j1y = fma(_raz, rx, -_rax * rz)
            val j1z = fma(_rax, ry, -_ray * rx)

            val j3x = -fma(_rby, rz, -_rbz * ry)
            val j3y = -fma(_rbz, rx, -_rbx * rz)
            val j3z = -fma(_rbx, ry, -_rby * rx)

            val rr = fma(rx, rx, fma(ry, ry, rz * rz))
            if (rr < 1e-11) return@forEach
            val d = sqrt(rr)
            n.put(rx)
            n.put(ry)
            n.put(rz)

            n.put(j1x)
            n.put(j1y)
            n.put(j1z)

            n.put(j3x)
            n.put(j3y)
            n.put(j3z)

            n.put(bias / timeStep * max(0f, abs(d) - slop)) // bias

            val bodyAIM = constraints.bodyAIM(constraintIdx)
            val bodyBIM = constraints.bodyBIM(constraintIdx)
            val bodyAII = bodyA.inverseInertia
            val bodyBII = bodyB.inverseInertia

            val den =
                rr * bodyAIM +
                _c_j1Temp.set(j1x, j1y, j1z)._mul(bodyAII).dot(j1x, j1y, j1z) +
                rr * bodyBIM +
                _c_j3Temp.set(j3x, j3y, j3z)._mul(bodyBII).dot(j3x, j3y, j3z)

            n.put(den) // den

            n.put(0f) // lambda

            val dva = _c_vec3.set(rx * bodyAIM, ry * bodyAIM, rz * bodyAIM)
            n.putVector3f(dva) // DVA
            n.putVector3f(_c_j1Temp.set(j1x, j1y, j1z)._mul(bodyAII)) // DOA
            val dvb = _c_vec3.set(-rx * bodyBIM, -ry * bodyBIM, -rz * bodyBIM)
            n.putVector3f(dvb) // DVB
            n.putVector3f(_c_j3Temp.set(j3x, j3y, j3z)._mul(bodyBII)) // DOB

            n.put(Float.fromBits(bodyAIdx)) // my id
            n.put(Float.fromBits(bodyBIdx)) // other id
            toSolve += SPHERICAL_JOINT_DATA_FLOATS
        }
    }

    fun solve() {
        var i = 0
        while (i < toSolve) {
            solve(jointData, i)

            i += SPHERICAL_JOINT_DATA_FLOATS
        }
    }

    private fun solve(data: FloatArray, contactIdx: Int) {
        val flatBodyData = constraintSolver.flatBodyData
        val nx = data[contactIdx + SPHERICAL_JOINT_NORMAL_OFFSET]
        val ny = data[contactIdx + SPHERICAL_JOINT_NORMAL_OFFSET + 1]
        val nz = data[contactIdx + SPHERICAL_JOINT_NORMAL_OFFSET + 2]

        //set nn, j1, and j3
        val bias = data[contactIdx + SPHERICAL_JOINT_BIAS_OFFSET]

        val den = data[contactIdx + SPHERICAL_JOINT_DEN_OFFSET]

        val myIdx = data[contactIdx + SPHERICAL_JOINT_MY_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

        val vAx = flatBodyData[myIdx + V_OFFSET]
        val vAy = flatBodyData[myIdx + V_OFFSET + 1]
        val vAz = flatBodyData[myIdx + V_OFFSET + 2]

        val oAx = flatBodyData[myIdx + O_OFFSET]
        val oAy = flatBodyData[myIdx + O_OFFSET + 1]
        val oAz = flatBodyData[myIdx + O_OFFSET + 2]

        val otherIdx = data[contactIdx + SPHERICAL_JOINT_OTHER_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

        val vBx = flatBodyData[otherIdx + V_OFFSET]
        val vBy = flatBodyData[otherIdx + V_OFFSET + 1]
        val vBz = flatBodyData[otherIdx + V_OFFSET + 2]

        val oBx = flatBodyData[otherIdx + O_OFFSET]
        val oBy = flatBodyData[otherIdx + O_OFFSET + 1]
        val oBz = flatBodyData[otherIdx + O_OFFSET + 2]

        var lambda = fma(
            nx, vAx, fma(
                ny, vAy, fma(
                    nz, vAz, fma(
                        data[contactIdx + SPHERICAL_JOINT_J1_OFFSET], oAx, fma(
                            data[contactIdx + SPHERICAL_JOINT_J1_OFFSET + 1], oAy, fma(
                                data[contactIdx + SPHERICAL_JOINT_J1_OFFSET + 2], oAz, fma(
                                    -nx, vBx, fma(
                                        -ny, vBy, fma(
                                            -nz, vBz, fma(
                                                data[contactIdx + SPHERICAL_JOINT_J3_OFFSET], oBx, fma(
                                                    data[contactIdx + SPHERICAL_JOINT_J3_OFFSET + 1], oBy, fma(
                                                        data[contactIdx + SPHERICAL_JOINT_J3_OFFSET + 2], oBz, bias
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
        ) / den

        val l = data[contactIdx + SPHERICAL_JOINT_LAMBDA_OFFSET]

        data[contactIdx + SPHERICAL_JOINT_LAMBDA_OFFSET] = max(0f, l + lambda)
        lambda = data[contactIdx + SPHERICAL_JOINT_LAMBDA_OFFSET] - l


//        if (maxLambda == -Float.MAX_VALUE || abs(lambda) > abs(maxLambda)) {
//            maxLambda = lambda
//        }
//
        flatBodyData[myIdx] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET] * lambda)
        flatBodyData[myIdx + 1] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 1] * lambda)
        flatBodyData[myIdx + 2] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 2] * lambda)

        flatBodyData[myIdx + 3] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 3] * lambda)
        flatBodyData[myIdx + 4] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 4] * lambda)
        flatBodyData[myIdx + 5] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 5] * lambda)

        flatBodyData[otherIdx] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 6] * lambda)
        flatBodyData[otherIdx + 1] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 7] * lambda)
        flatBodyData[otherIdx + 2] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 8] * lambda)

        flatBodyData[otherIdx + 3] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 9] * lambda)
        flatBodyData[otherIdx + 4] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 10] * lambda)
        flatBodyData[otherIdx + 5] -= (data[contactIdx + SPHERICAL_JOINT_DELTA_OFFSET + 11] * lambda)
    }
}

private const val SPHERICAL_JOINT_DATA_FLOATS = 26

private const val SPHERICAL_JOINT_NORMAL_OFFSET = 0
private const val SPHERICAL_JOINT_J1_OFFSET = 3
private const val SPHERICAL_JOINT_J3_OFFSET = 6
private const val SPHERICAL_JOINT_BIAS_OFFSET = 9
private const val SPHERICAL_JOINT_DEN_OFFSET = 10
private const val SPHERICAL_JOINT_LAMBDA_OFFSET = 11
private const val SPHERICAL_JOINT_DELTA_OFFSET = 12
private const val SPHERICAL_JOINT_MY_IDX_OFFSET = 24
private const val SPHERICAL_JOINT_OTHER_IDX_OFFSET = 25