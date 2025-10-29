package com.ixume.udar.physics.sphericaljoint

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.BODY_DATA_FLOATS
import com.ixume.udar.physics.constraint.LocalConstraintSolver
import com.ixume.udar.physics.constraint.O_OFFSET
import com.ixume.udar.physics.constraint.V_OFFSET
import org.joml.Vector3d
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.max
import kotlin.math.sqrt

class LocalSphericalJointSolver(val constraintSolver: LocalConstraintSolver) {
    private var timeStep = Udar.Companion.CONFIG.timeStep.toFloat()
    private var bias = Udar.Companion.CONFIG.sphericalJoint.bias.toFloat()
    private var slop = Udar.Companion.CONFIG.sphericalJoint.slop.toFloat()

    private val constraints = constraintSolver.physicsWorld.sphericalJointConstraints.constraints
    private var toSolve = 0

    private var jointData: FloatArray = FloatArray(1)

    private val _rotatedRA = Vector3d()
    private val _rotatedRB = Vector3d()
    private val _j1scaled = Vector3d()
    private val _j3scaled = Vector3d()


    fun setup() {
        timeStep = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.sphericalJoint.bias.toFloat()
        slop = Udar.CONFIG.sphericalJoint.slop.toFloat()

        constructFlatConstraintData()
    }

    private fun constructFlatConstraintData() {
        val numc = constraints.size()
        val relevantData = run {
            if (jointData.size < numc * OFFSET_12_AA_DATA_SIZE) {
                jointData = FloatArray(max(jointData.size * 2, numc * OFFSET_12_AA_DATA_SIZE))
            }

            jointData
        }

        val n = FloatBuffer.wrap(relevantData)

        toSolve = 0
        constraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, rax, ray, raz, rbx, rby, rbz ->
            /*
            distance constraint between 2 points P_a and P_b on bodies a and b can be modeled as:
            |p_a - p_b| = 0
            differentiating with respect to time:
            (p_a - p_b) . ((v_a + o_a x r_a) - (v_b + o_b x r_b)) = 0
            r = p_a - p_b
            r . v_a + r . (o_a x r_a) - r . v_b - r . (o_b x r_b) = 0
            r . v_a + o_a . (r_a x r) - r . v_b - o_b . (r_b x r) = 0
            J = [r, r_a x r, -r, -r_b x r]
            V = [v_a, o_a, v_b, o_b]
            JV + b = 0
            dV = l M^-1 J
            J(V + dV) + b = 0
            l = -(JV + b) / (J M^-1 J)
             */
            val bodyA = constraintSolver.physicsWorld.activeBodies.fastGet(bodyAIdx)!!
            val bodyB = constraintSolver.physicsWorld.activeBodies.fastGet(bodyBIdx)!!

            val pa = bodyA.pos
            val pb = bodyB.pos

            val ima = bodyA.inverseMass.toFloat()
            val imb = bodyB.inverseMass.toFloat()

            val iia = bodyA.inverseInertia
            val iib = bodyB.inverseInertia

            val qA = bodyA.q
            val qB = bodyB.q

            val rotatedRA = qA.transform(rax.toDouble(), ray.toDouble(), raz.toDouble(), _rotatedRA)
            val rotatedRB = qB.transform(rbx.toDouble(), rby.toDouble(), rbz.toDouble(), _rotatedRB)

            val j0x = (rotatedRA.x - rotatedRB.x + pa.x - pb.x).toFloat()
            val j0y = (rotatedRA.y - rotatedRB.y + pa.y - pb.y).toFloat()
            val j0z = (rotatedRA.z - rotatedRB.z + pa.z - pb.z).toFloat()

            val rr = fma(j0x, j0x, fma(j0y, j0y, j0z * j0z))
            if (rr < 1e-11) return@forEach

            n.put(j0x)
            n.put(j0y)
            n.put(j0z)

            val j1x = fma(rotatedRA.y.toFloat(), j0z, -rotatedRA.z.toFloat() * j0y)
            val j1y = fma(rotatedRA.z.toFloat(), j0x, -rotatedRA.x.toFloat() * j0z)
            val j1z = fma(rotatedRA.x.toFloat(), j0y, -rotatedRA.y.toFloat() * j0x)

            n.put(j1x)
            n.put(j1y)
            n.put(j1z)

            val j3x = -fma(rotatedRB.y.toFloat(), j0z, -rotatedRB.z.toFloat() * j0y)
            val j3y = -fma(rotatedRB.z.toFloat(), j0x, -rotatedRB.x.toFloat() * j0z)
            val j3z = -fma(rotatedRB.x.toFloat(), j0y, -rotatedRB.y.toFloat() * j0x)

            n.put(j3x)
            n.put(j3y)
            n.put(j3z)

            n.put(ima)
            n.put(imb)

            val j1scaled = iia.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

            n.put(j1scaled.x.toFloat())
            n.put(j1scaled.y.toFloat())
            n.put(j1scaled.z.toFloat())

            val j3scaled = iib.transform(j3x.toDouble(), j3y.toDouble(), j3z.toDouble(), _j3scaled)

            n.put(j3scaled.x.toFloat())
            n.put(j3scaled.y.toFloat())
            n.put(j3scaled.z.toFloat())

            val b = bias / timeStep * sqrt(rr)
            n.put(b)

            val den =
                rr * ima +
                rr * imb +
                j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat() +
                j3scaled.dot(j3x.toDouble(), j3y.toDouble(), j3z.toDouble()).toFloat()
            if (den < 1e-11) return@forEach
            n.put(den)

            n.put(Float.fromBits(bodyAIdx))
            n.put(Float.fromBits(bodyBIdx))

            n.put(0f)

            toSolve += OFFSET_12_AA_DATA_SIZE
        }
    }

    fun solve() {
        var i = 0
        while (i < toSolve) {
            solveLinearConstraint(constraintSolver.flatBodyData, jointData, i) { existing, calculated ->
                max(0f, existing + calculated)
            }

            i += OFFSET_12_AA_DATA_SIZE
        }
    }
}

private const val OFFSET_12_AA_DATA_SIZE = 22

private const val OFFSET_12_AA_IMA = 9
private const val OFFSET_12_AA_IMB = 10
private const val OFFSET_12_AA_BIAS = 17
private const val OFFSET_12_AA_DEN = 18
private const val OFFSET_12_AA_A_IDX = 19
private const val OFFSET_12_AA_B_IDX = 20
private const val OFFSET_12_AA_LAMBDA = 21

private inline fun solveLinearConstraint(
    bodyData: FloatArray,
    constraintData: FloatArray,
    idx: Int,
    lambdaTransform: (existing: Float, calculated: Float) -> Float,
) {
    val j0x = constraintData[idx]
    val j0y = constraintData[idx + 1]
    val j0z = constraintData[idx + 2]
    val j1x = constraintData[idx + 3]
    val j1y = constraintData[idx + 4]
    val j1z = constraintData[idx + 5]
    val j2x = -j0x
    val j2y = -j0y
    val j2z = -j0z
    val j3x = constraintData[idx + 6]
    val j3y = constraintData[idx + 7]
    val j3z = constraintData[idx + 8]

    val ima = constraintData[OFFSET_12_AA_IMA]
    val imb = constraintData[OFFSET_12_AA_IMB]

    val v0x = j0x * ima
    val v0y = j0y * ima
    val v0z = j0z * ima
    val v1x = constraintData[idx + 11]
    val v1y = constraintData[idx + 12]
    val v1z = constraintData[idx + 13]
    val v2x = j2x * imb
    val v2y = j2y * imb
    val v2z = j2z * imb
    val v3x = constraintData[idx + 14]
    val v3y = constraintData[idx + 15]
    val v3z = constraintData[idx + 16]

    val bias = constraintData[OFFSET_12_AA_BIAS]
    val den = constraintData[OFFSET_12_AA_DEN]

    val aIdx = constraintData[OFFSET_12_AA_A_IDX].toRawBits() * BODY_DATA_FLOATS
    val bIdx = constraintData[OFFSET_12_AA_B_IDX].toRawBits() * BODY_DATA_FLOATS

    val existing = constraintData[OFFSET_12_AA_LAMBDA]

    val vAx = bodyData[aIdx + V_OFFSET]
    val vAy = bodyData[aIdx + V_OFFSET + 1]
    val vAz = bodyData[aIdx + V_OFFSET + 2]

    val oAx = bodyData[aIdx + O_OFFSET]
    val oAy = bodyData[aIdx + O_OFFSET + 1]
    val oAz = bodyData[aIdx + O_OFFSET + 2]

    val vBx = bodyData[bIdx + V_OFFSET]
    val vBy = bodyData[bIdx + V_OFFSET + 1]
    val vBz = bodyData[bIdx + V_OFFSET + 2]

    val oBx = bodyData[bIdx + O_OFFSET]
    val oBy = bodyData[bIdx + O_OFFSET + 1]
    val oBz = bodyData[bIdx + O_OFFSET + 2]

    val lambda =
        fma(
            j0x, vAx,
            fma(
                j0y, vAy,
                fma(
                    j0z, vAz,
                    fma(
                        j1x, oAx,
                        fma(
                            j1y, oAy,
                            fma(
                                j1z, oAz,
                                fma(
                                    j2x, vBx,
                                    fma(
                                        j2y, vBy,
                                        fma(
                                            j2z, vBz,
                                            fma(
                                                j3x, oBx,
                                                fma(
                                                    j3y, oBy,
                                                    fma(
                                                        j3z, oBz,
                                                        bias
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

    constraintData[OFFSET_12_AA_LAMBDA] = lambdaTransform(existing, lambda)
    val effective = constraintData[OFFSET_12_AA_LAMBDA] - existing

    bodyData[aIdx + 0] -= v0x * effective
    bodyData[aIdx + 1] -= v0y * effective
    bodyData[aIdx + 2] -= v0z * effective

    bodyData[aIdx + 3] -= v1x * effective
    bodyData[aIdx + 4] -= v1y * effective
    bodyData[aIdx + 5] -= v1z * effective

    bodyData[bIdx + 0] -= v2x * effective
    bodyData[bIdx + 1] -= v2y * effective
    bodyData[bIdx + 2] -= v2z * effective

    bodyData[bIdx + 3] -= v3x * effective
    bodyData[bIdx + 4] -= v3y * effective
    bodyData[bIdx + 5] -= v3z * effective
}
