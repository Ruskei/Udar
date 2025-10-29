package com.ixume.udar.physics.angular

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.BODY_DATA_FLOATS
import com.ixume.udar.physics.constraint.LocalConstraintSolver
import com.ixume.udar.physics.constraint.O_OFFSET
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.max

/*
ball and socket constraint:
spherical joint, unlimited twist, limited x, z
 */
class LocalAngularConstraintSolver(val constraintSolver: LocalConstraintSolver) {
    private var timeStep = Udar.Companion.CONFIG.timeStep.toFloat()
    private var bias = Udar.Companion.CONFIG.angularConstraint.bias.toFloat()
    private var slop = Udar.Companion.CONFIG.angularConstraint.slop.toFloat()

    private val constraints = constraintSolver.physicsWorld.angularConstraints.constraints
    private var toSolve = 0

    private var jointData: FloatArray = FloatArray(1)

    private val _i = Vector3f()
    private val _j = Vector3f()
    private val _j0scaled = Vector3d()
    private val _j1scaled = Vector3d()

    fun setup() {
        timeStep = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.angularConstraint.bias.toFloat()
        slop = Udar.CONFIG.angularConstraint.slop.toFloat()

        constructFlatConstraintData()
    }

    private fun constructFlatConstraintData() {
        val numc = constraints.size()
        val relevantData = run {
            if (jointData.size < numc * OFFSET_6_AA_DATA_SIZE) {
                jointData = FloatArray(max(jointData.size * 2, numc * OFFSET_6_AA_DATA_SIZE))
            }

            jointData
        }

        val n = FloatBuffer.wrap(relevantData)

        toSolve = 0
        constraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, bodyAAxisX, bodyAAxisY, bodyAAxisZ, bodyBAxisX, bodyBAxisY, bodyBAxisZ ->
            val bodyA = constraintSolver.physicsWorld.activeBodies.fastGet(bodyAIdx)!!
            val bodyB = constraintSolver.physicsWorld.activeBodies.fastGet(bodyBIdx)!!

            val iia = bodyA.inverseInertia
            val iib = bodyB.inverseInertia

            val qA = bodyA.q
            val qB = bodyB.q

            val i = qA.transform(bodyAAxisX.toDouble(), bodyAAxisY.toDouble(), bodyAAxisZ.toDouble(), _i).normalize()
            val j = qB.transform(bodyBAxisX.toDouble(), bodyBAxisY.toDouble(), bodyBAxisZ.toDouble(), _j).normalize()

            val j0x = fma(i.y, j.z, -i.z * j.y)
            val j0y = fma(i.z, j.x, -i.x * j.z)
            val j0z = fma(i.x, j.y, -i.y * j.x)

            n.put(j0x)
            n.put(j0y)
            n.put(j0z)

            val j0scaled = iia.transform(j0x.toDouble(), j0y.toDouble(), j0z.toDouble(), _j0scaled)

            n.put(j0scaled.x.toFloat())
            n.put(j0scaled.y.toFloat())
            n.put(j0scaled.z.toFloat())

            val j1x = -j0x
            val j1y = -j0y
            val j1z = -j0z

            val j1scaled = iib.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

            n.put(j1scaled.x.toFloat())
            n.put(j1scaled.y.toFloat())
            n.put(j1scaled.z.toFloat())

            val b = -bias / timeStep * (1f - i.dot(j))
            n.put(b)

            val den =
                j0scaled.dot(j0x.toDouble(), j0y.toDouble(), j0z.toDouble()).toFloat() +
                j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat()
            if (den < 1e-11) return@forEach
            n.put(den)

            n.put(Float.fromBits(bodyAIdx))
            n.put(Float.fromBits(bodyBIdx))

            n.put(0f)

            toSolve += OFFSET_6_AA_DATA_SIZE

            /*
            an axis alignment constraint is really simple, just takes in g_1 and g_2
            
		    this means (q_a * g_1) . (q_b * g_2) = 1
		    derivative:
		    d/dt (Q * g) = o x (Q * g)
		    so:
		    (o_a x (q_a * g_1)) . (q_b * g_2) + (o_b x (q_b * g_2)) . (q_a * g_1) = 0
		    i = q_a * g_1, j = q_b * g_2
		    (o_a x i) . j + (o_b x j) . i = 0
		    (i x j) . o_a + (-i x j) . o_b = 0
		    J = [i x j, -i x j],
		    V = [o_a, o_b]
		    JV = 0
		    dV = l m^-1 J
		    J(V + dV) = 0
		    J(V + l m^-1 J) = 0
		    l = (-JV + b) / (J m^-1 J)
             */
        }
    }

    fun solve() {
        var i = 0
        while (i < toSolve) {
            solveAngleConstraint(constraintSolver.flatBodyData, jointData, i) { existing, calculated ->
                max(0f, existing + calculated)
            }

            i += OFFSET_6_AA_DATA_SIZE
        }
    }
}

private const val OFFSET_6_AA_DATA_SIZE = 14

private const val OFFSET_6_AA_BIAS = 9
private const val OFFSET_6_AA_DEN = 10
private const val OFFSET_6_AA_A_IDX = 11
private const val OFFSET_6_AA_B_IDX = 12
private const val OFFSET_6_AA_LAMBDA = 13

private inline fun solveAngleConstraint(
    bodyData: FloatArray,
    constraintData: FloatArray,
    idx: Int,
    lambdaTransform: (existing: Float, calculated: Float) -> Float,
) {
    val j0x = constraintData[idx]
    val j0y = constraintData[idx + 1]
    val j0z = constraintData[idx + 2]
    val j1x = -j0x
    val j1y = -j0y
    val j1z = -j0z

    val v0x = constraintData[idx + 3]
    val v0y = constraintData[idx + 4]
    val v0z = constraintData[idx + 5]
    val v1x = constraintData[idx + 6]
    val v1y = constraintData[idx + 7]
    val v1z = constraintData[idx + 8]

    val bias = constraintData[OFFSET_6_AA_BIAS]
    val den = constraintData[OFFSET_6_AA_DEN]

    val aIdx = constraintData[OFFSET_6_AA_A_IDX].toRawBits() * BODY_DATA_FLOATS
    val bIdx = constraintData[OFFSET_6_AA_B_IDX].toRawBits() * BODY_DATA_FLOATS

    val existing = constraintData[OFFSET_6_AA_LAMBDA]

    val oAx = bodyData[aIdx + O_OFFSET]
    val oAy = bodyData[aIdx + O_OFFSET + 1]
    val oAz = bodyData[aIdx + O_OFFSET + 2]

    val oBx = bodyData[bIdx + O_OFFSET]
    val oBy = bodyData[bIdx + O_OFFSET + 1]
    val oBz = bodyData[bIdx + O_OFFSET + 2]

    val lambda =
        -fma(
            j0x, oAx,
            fma(
                j0y, oAy,
                fma(
                    j0z, oAz,
                    fma(
                        j1x, oBx,
                        fma(
                            j1y, oBy,
                            fma(
                                j1z, oBz,
                                bias
                            )
                        )
                    )
                )
            )
        ) / den

    constraintData[OFFSET_6_AA_LAMBDA] = lambdaTransform(existing, lambda)
    val effective = constraintData[OFFSET_6_AA_LAMBDA] - existing

    bodyData[aIdx + 3] += v0x * effective
    bodyData[aIdx + 4] += v0y * effective
    bodyData[aIdx + 5] += v0z * effective

    bodyData[bIdx + 3] += v1x * effective
    bodyData[bIdx + 4] += v1y * effective
    bodyData[bIdx + 5] += v1z * effective
}
