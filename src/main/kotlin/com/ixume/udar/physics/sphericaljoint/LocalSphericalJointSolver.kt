package com.ixume.udar.physics.sphericaljoint

import com.ixume.udar.Udar
import com.ixume.udar.physics.angular.OFFSET_6_AA_DATA_SIZE
import com.ixume.udar.physics.angular.solveAngleConstraint
import com.ixume.udar.physics.constraint.BODY_DATA_FLOATS
import com.ixume.udar.physics.constraint.LocalConstraintData
import com.ixume.udar.physics.constraint.O_OFFSET
import com.ixume.udar.physics.constraint.V_OFFSET
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class LocalSphericalJointSolver(val constraintSolver: LocalConstraintData) {
    private var timeStep = Udar.Companion.CONFIG.timeStep.toFloat()
    private var bias = Udar.Companion.CONFIG.sphericalJoint.bias.toFloat()
    private var slop = Udar.Companion.CONFIG.sphericalJoint.slop.toFloat()
    private var friction = Udar.Companion.CONFIG.sphericalJoint.friction.toFloat()

    private val constraints = constraintSolver.physicsWorld.sphericalJointConstraints.constraints
    private var toSolve = 0
    private var angleData: FloatArray = FloatArray(1)
    private var toSolveAngular = 0

    private var jointAxisData: FloatArray = FloatArray(1)

    private val _rotatedRA = Vector3d()
    private val _rotatedRB = Vector3d()
    private val _j0scaled = Vector3d()
    private val _j1scaled = Vector3d()
    private val _j3scaled = Vector3d()
    private val _ref = Vector3f()
    private val _temp1 = Vector3f()
    private val _t1 = Vector3f()
    private val _t2 = Vector3f()

    fun setup() {
        timeStep = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.sphericalJoint.bias.toFloat()
        slop = Udar.CONFIG.sphericalJoint.slop.toFloat()
        friction = Udar.CONFIG.sphericalJoint.friction.toFloat()

        constructFlatConstraintData()
    }

    private fun constructFlatConstraintData() {
        val numc = constraints.size()

        if (jointAxisData.size < numc * OFFSET_12_AA_DATA_SIZE) {
            jointAxisData = FloatArray(max(jointAxisData.size * 2, numc * OFFSET_12_AA_DATA_SIZE))
            angleData = FloatArray(max(jointAxisData.size * 2, numc * 3 * OFFSET_6_AA_DATA_SIZE))
        }

        val axisBuf = FloatBuffer.wrap(jointAxisData)
        val angleBuf = FloatBuffer.wrap(angleData)

        toSolve = 0
        toSolveAngular = 0
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

            _ref.set(
                (rotatedRA.x - rotatedRB.x + pa.x - pb.x).toFloat(),
                (rotatedRA.y - rotatedRB.y + pa.y - pb.y).toFloat(),
                (rotatedRA.z - rotatedRB.z + pa.z - pb.z).toFloat(),
            )
            val rr = _ref.lengthSquared()
            if (rr < 1e-11) return@forEach
            val n = _temp1.set(_ref).normalize()
            _t1.set(1f, 0f, 0f).orthogonalizeUnit(n)
            _t2.set(n).cross(_t1).normalize()
            val dist = sqrt(rr)

            run axis@{
                val j0x = _ref.x / dist
                val j0y = _ref.y / dist
                val j0z = _ref.z / dist

                axisBuf.put(j0x)
                axisBuf.put(j0y)
                axisBuf.put(j0z)

                val j1x = fma(rotatedRA.y.toFloat(), j0z, -rotatedRA.z.toFloat() * j0y)
                val j1y = fma(rotatedRA.z.toFloat(), j0x, -rotatedRA.x.toFloat() * j0z)
                val j1z = fma(rotatedRA.x.toFloat(), j0y, -rotatedRA.y.toFloat() * j0x)

                axisBuf.put(j1x)
                axisBuf.put(j1y)
                axisBuf.put(j1z)

                val j3x = -fma(rotatedRB.y.toFloat(), j0z, -rotatedRB.z.toFloat() * j0y)
                val j3y = -fma(rotatedRB.z.toFloat(), j0x, -rotatedRB.x.toFloat() * j0z)
                val j3z = -fma(rotatedRB.x.toFloat(), j0y, -rotatedRB.y.toFloat() * j0x)

                axisBuf.put(j3x)
                axisBuf.put(j3y)
                axisBuf.put(j3z)

                axisBuf.put(ima)
                axisBuf.put(imb)

                val j1scaled = iia.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

                axisBuf.put(j1scaled.x.toFloat())
                axisBuf.put(j1scaled.y.toFloat())
                axisBuf.put(j1scaled.z.toFloat())

                val j3scaled = iib.transform(j3x.toDouble(), j3y.toDouble(), j3z.toDouble(), _j3scaled)

                axisBuf.put(j3scaled.x.toFloat())
                axisBuf.put(j3scaled.y.toFloat())
                axisBuf.put(j3scaled.z.toFloat())

                val b = bias / timeStep * max(dist - slop, 0f)
                axisBuf.put(b)

                val den =
                    ima +
                    imb +
                    j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat() +
                    j3scaled.dot(j3x.toDouble(), j3y.toDouble(), j3z.toDouble()).toFloat()
                axisBuf.put(den)

                axisBuf.put(Float.fromBits(bodyAIdx))
                axisBuf.put(Float.fromBits(bodyBIdx))

                axisBuf.put(0f)
            }

            run x@{
                val j0x = 1f
                val j0y = 0f
                val j0z = 0f

                angleBuf.put(j0x)
                angleBuf.put(j0y)
                angleBuf.put(j0z)

                val j0scaled = bodyA.inverseInertia.transform(j0x.toDouble(), j0y.toDouble(), j0z.toDouble(), _j0scaled)

                angleBuf.put(j0scaled.x.toFloat())
                angleBuf.put(j0scaled.y.toFloat())
                angleBuf.put(j0scaled.z.toFloat())

                val j1x = -j0x
                val j1y = -j0y
                val j1z = -j0z

                val j1scaled = bodyB.inverseInertia.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

                angleBuf.put(j1scaled.x.toFloat())
                angleBuf.put(j1scaled.y.toFloat())
                angleBuf.put(j1scaled.z.toFloat())

                val b = 0f
                angleBuf.put(b)

                val den =
                    j0scaled.dot(j0x.toDouble(), j0y.toDouble(), j0z.toDouble()).toFloat() +
                    j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat()
                angleBuf.put(den)

                angleBuf.put(Float.fromBits(bodyAIdx))
                angleBuf.put(Float.fromBits(bodyBIdx))

                angleBuf.put(0f)
            }
            
            run y@{
                val j0x = 0f
                val j0y = 1f
                val j0z = 0f

                angleBuf.put(j0x)
                angleBuf.put(j0y)
                angleBuf.put(j0z)

                val j0scaled = bodyA.inverseInertia.transform(j0x.toDouble(), j0y.toDouble(), j0z.toDouble(), _j0scaled)

                angleBuf.put(j0scaled.x.toFloat())
                angleBuf.put(j0scaled.y.toFloat())
                angleBuf.put(j0scaled.z.toFloat())

                val j1x = -j0x
                val j1y = -j0y
                val j1z = -j0z

                val j1scaled = bodyB.inverseInertia.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

                angleBuf.put(j1scaled.x.toFloat())
                angleBuf.put(j1scaled.y.toFloat())
                angleBuf.put(j1scaled.z.toFloat())

                val b = 0f
                angleBuf.put(b)

                val den =
                    j0scaled.dot(j0x.toDouble(), j0y.toDouble(), j0z.toDouble()).toFloat() +
                    j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat()
                angleBuf.put(den)

                angleBuf.put(Float.fromBits(bodyAIdx))
                angleBuf.put(Float.fromBits(bodyBIdx))

                angleBuf.put(0f)
            }
            
            run z@{
                val j0x = 0f
                val j0y = 0f
                val j0z = 1f

                angleBuf.put(j0x)
                angleBuf.put(j0y)
                angleBuf.put(j0z)

                val j0scaled = bodyA.inverseInertia.transform(j0x.toDouble(), j0y.toDouble(), j0z.toDouble(), _j0scaled)

                angleBuf.put(j0scaled.x.toFloat())
                angleBuf.put(j0scaled.y.toFloat())
                angleBuf.put(j0scaled.z.toFloat())

                val j1x = -j0x
                val j1y = -j0y
                val j1z = -j0z

                val j1scaled = bodyB.inverseInertia.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

                angleBuf.put(j1scaled.x.toFloat())
                angleBuf.put(j1scaled.y.toFloat())
                angleBuf.put(j1scaled.z.toFloat())

                val b = 0f
                angleBuf.put(b)

                val den =
                    j0scaled.dot(j0x.toDouble(), j0y.toDouble(), j0z.toDouble()).toFloat() +
                    j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat()
                angleBuf.put(den)

                angleBuf.put(Float.fromBits(bodyAIdx))
                angleBuf.put(Float.fromBits(bodyBIdx))

                angleBuf.put(0f)
            }

            toSolve += OFFSET_12_AA_DATA_SIZE
            toSolveAngular += OFFSET_6_AA_DATA_SIZE * 3
        }
    }

    fun solve() {
        var i = 0
        while (i < toSolve) {
            solveLinearConstraint(
                constraintSolver.flatBodyData,
                jointAxisData,
                i
            ) { existing, calculated -> 
                existing + calculated 
            }

            i += OFFSET_12_AA_DATA_SIZE
        }
    }

    fun solveFriction() {
        var i = 0
        while (i < toSolveAngular) {
            solveAngleConstraint(constraintSolver.flatBodyData, angleData, i) { existing, calculated ->
                val n = jointAxisData[i / OFFSET_6_AA_DATA_SIZE / 3 * OFFSET_12_AA_DATA_SIZE + OFFSET_12_AA_LAMBDA]
                min(max(-friction * n, existing + calculated), friction * n)
            }

            i += OFFSET_6_AA_DATA_SIZE
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

    val ima = constraintData[idx + OFFSET_12_AA_IMA]
    val imb = constraintData[idx + OFFSET_12_AA_IMB]

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

    val bias = constraintData[idx + OFFSET_12_AA_BIAS]
    val den = constraintData[idx + OFFSET_12_AA_DEN]

    val aIdx = constraintData[idx + OFFSET_12_AA_A_IDX].toRawBits() * BODY_DATA_FLOATS
    val bIdx = constraintData[idx + OFFSET_12_AA_B_IDX].toRawBits() * BODY_DATA_FLOATS

    val existing = constraintData[idx + OFFSET_12_AA_LAMBDA]

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

    constraintData[idx + OFFSET_12_AA_LAMBDA] = lambdaTransform(existing, lambda)
    val effective = constraintData[idx + OFFSET_12_AA_LAMBDA] - existing
    

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
