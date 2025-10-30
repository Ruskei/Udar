package com.ixume.udar.physics.angular

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.BODY_DATA_FLOATS
import com.ixume.udar.physics.constraint.LocalConstraintSolver
import com.ixume.udar.physics.constraint.O_OFFSET
import org.joml.Matrix3d
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.*

class LocalAngularConstraintSolver(val constraintSolver: LocalConstraintSolver) {
    private var timeStep = Udar.Companion.CONFIG.timeStep.toFloat()
    private var bias = Udar.Companion.CONFIG.angularConstraint.bias.toFloat()
    private var slop = Udar.Companion.CONFIG.angularConstraint.slop.toFloat()

    private val axisConstraints = constraintSolver.physicsWorld.angularConstraints.constraints
    private var axisToSolve = 0
    private val angleConstraints = constraintSolver.physicsWorld.angularConstraints.limitedConstraints
    private var angleToSolve = 0
    private val ballConstraints = constraintSolver.physicsWorld.angularConstraints.ballConstraints
    private var ballToSolve = 0

    private var jointData: FloatArray = FloatArray(1)
    private var limitedJointData: FloatArray = FloatArray(1)
    private var revoluteJointData: FloatArray = FloatArray(1)

    private val _i = Vector3f()
    private val _j = Vector3f()
    private val _j0scaled = Vector3d()
    private val _j1scaled = Vector3d()

    private val `_jA'` = Vector3d()
    private val `_jB'` = Vector3d()
    private val _jN = Vector3d()
    private val `_gA'` = Vector3d()
    private val `_gB'` = Vector3d()
    private val _pA = Vector3d()
    private val `_pA'` = Vector3d()
    private val _pB = Vector3d()
    private val _temp1 = Vector3d()
    private val _temp2 = Vector3f()
    private val _temp3 = Vector3f()

    private val _J = Vector3d()

    fun setup() {
        timeStep = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.angularConstraint.bias.toFloat()
        slop = Udar.CONFIG.angularConstraint.slop.toFloat()

        constructAxisConstraintData()
        constructLimitedAngleConstraintData()
        constructBallConstraintData()
    }

    private fun constructAxisConstraintData() {
        val numc = axisConstraints.size()
        val relevantData = run {
            if (jointData.size < numc * OFFSET_6_AA_DATA_SIZE) {
                jointData = FloatArray(max(jointData.size * 2, numc * OFFSET_6_AA_DATA_SIZE))
            }

            jointData
        }

        val n = FloatBuffer.wrap(relevantData)

        axisToSolve = 0
        axisConstraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, bodyAAxisX, bodyAAxisY, bodyAAxisZ, bodyBAxisX, bodyBAxisY, bodyBAxisZ ->
            val bodyA = constraintSolver.physicsWorld.activeBodies.fastGet(bodyAIdx)!!
            val bodyB = constraintSolver.physicsWorld.activeBodies.fastGet(bodyBIdx)!!

            val qA = bodyA.q
            val qB = bodyB.q

            val i = qA.transform(bodyAAxisX.toDouble(), bodyAAxisY.toDouble(), bodyAAxisZ.toDouble(), _i).normalize()
            val j = qB.transform(bodyBAxisX.toDouble(), bodyBAxisY.toDouble(), bodyBAxisZ.toDouble(), _j).normalize()

            if (alignAxis(
                    bodyAIdx = bodyAIdx,
                    bodyBIdx = bodyBIdx,
                    iia = bodyA.inverseInertia,
                    iib = bodyB.inverseInertia,
                    buf = n,
                    i = i,
                    j = j,
                )
            ) {
                axisToSolve += OFFSET_6_AA_DATA_SIZE
            }

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

    private fun constructLimitedAngleConstraintData() {
        val numc = angleConstraints.size()
        val relevantData = run {
            if (limitedJointData.size < numc * OFFSET_6_AA_DATA_SIZE) {
                limitedJointData = FloatArray(max(limitedJointData.size * 2, numc * OFFSET_6_AA_DATA_SIZE))
            }

            limitedJointData
        }

        val n = FloatBuffer.wrap(relevantData)

        angleToSolve = 0
        angleConstraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, jAX, jAY, jAZ, jBX, jBY, jBZ, gAX, gAY, gAZ, gBX, gBY, gBZ, minAngle, maxAngle ->
            val bodyA = constraintSolver.physicsWorld.activeBodies.fastGet(bodyAIdx)!!
            val bodyB = constraintSolver.physicsWorld.activeBodies.fastGet(bodyBIdx)!!

            val qA = bodyA.q
            val qB = bodyB.q

            val `jA'` = qA.transform(jAX.toDouble(), jAY.toDouble(), jAZ.toDouble(), `_jA'`)
            val `jB'` = qB.transform(jBX.toDouble(), jBY.toDouble(), jBZ.toDouble(), `_jB'`)
            val j = `jA'`.add(`jB'`, _jN).normalize()

            val `gA'` = qA.transform(gAX.toDouble(), gAY.toDouble(), gAZ.toDouble(), `_gA'`)
            val `gB'` = qB.transform(gBX.toDouble(), gBY.toDouble(), gBZ.toDouble(), `_gB'`)

            val pA = run {
                val m = j.dot(`gA'`)
                _pA.set(`gA'`).sub(
                    j.x * m,
                    j.y * m,
                    j.z * m,
                )
            }.normalize()

            val pB = run {
                val m = j.dot(`gB'`)
                _pB.set(`gB'`).sub(
                    j.x * m,
                    j.y * m,
                    j.z * m,
                )
            }.normalize()

            val theta = atan2(_temp1.set(pA).cross(pB).dot(j), pA.dot(pB))
            if (!theta.isFinite()) return@forEach

            if (theta in minAngle..maxAngle) return@forEach


            val `theta'` = min(max(minAngle.toDouble(), theta), maxAngle.toDouble())
            val `pA'` = `_pA'`.set(pA).mul(cos(`theta'`))
                .add(_temp1.set(j).cross(pA).mul(sin(`theta'`)))
                .add(_temp1.set(j).mul(j.dot(pA) * (1.0 - cos(`theta'`))))
                .normalize()

            if (alignAxis(
                    bodyAIdx = bodyAIdx,
                    bodyBIdx = bodyBIdx,
                    iia = bodyA.inverseInertia,
                    iib = bodyB.inverseInertia,
                    buf = n,
                    i = _temp2.set(`pA'`),
                    j = _temp3.set(pB),
                )
            ) {
                angleToSolve += OFFSET_6_AA_DATA_SIZE
            }
            /*
            for an angle constraint with an angle limit, we take in the following constraint data:
            jA = axis of rotation on A
            jB = axis of rotation on B
            gA = reference axis on A
            gB = reference axis on B
            phi< = min angle
            phi> = max angle
            
            we also have:
            qA, qB = rotations
            oA, oB = omegas
            
            if a vector is ', then it was rotated from local to world
            
            first, the normalized rotation axis will be:
            j = (jA' + jB') / 2
            then we project gA' and gB' onto j plane
            pA = gA' - j(gA' . j)
            pB = gB' - j(gB' . j)
            
            theta = atan2(pA x pB . j, pA . pB)
            
            if theta < phi< or theta > phi>
                theta' = clamp(theta, phi<, phi>)
                pA' = rotate pA around j by theta'
                align: pA' to pB
            
            the output is 2 axes that we need to align
             */
        }
    }

    private fun constructBallConstraintData() {
        val numc = ballConstraints.size()
        val relevantData = run {
            if (revoluteJointData.size < numc * 2 * OFFSET_6_AA_DATA_SIZE) {
                revoluteJointData = FloatArray(max(revoluteJointData.size * 2, numc * 2 * OFFSET_6_AA_DATA_SIZE))
            }

            revoluteJointData
        }

        val n = FloatBuffer.wrap(relevantData)

        ballToSolve = 0
        ballConstraints.forEach { constraintIdx, bodyAIdx, bodyBIdx, jAX, jAY, jAZ, jBX, jBY, jBZ, gAX, gAY, gAZ, gBX, gBY, gBZ, swingAngle, minTwistAngle, maxTwistAngle ->
            val bodyA = constraintSolver.physicsWorld.activeBodies.fastGet(bodyAIdx)!!
            val bodyB = constraintSolver.physicsWorld.activeBodies.fastGet(bodyBIdx)!!

            val qA = bodyA.q
            val qB = bodyB.q

            val `jA'` = qA.transform(jAX.toDouble(), jAY.toDouble(), jAZ.toDouble(), `_jA'`).normalize()
            val `jB'` = qB.transform(jBX.toDouble(), jBY.toDouble(), jBZ.toDouble(), `_jB'`).normalize()

            run swing@{
                val J = _J.set(`jA'`).cross(`jB'`).normalize()

                val theta = atan2(_temp1.set(`jA'`).cross(`jB'`).dot(J), `jA'`.dot(`jB'`))
                if (!theta.isFinite()) return@swing
                if (theta < swingAngle) return@swing

                val `theta'` = min(theta, swingAngle.toDouble())
                val `jA''` = `_pA'`.set(`jA'`).mul(cos(`theta'`))
                    .add(_temp1.set(J).cross(`jA'`).mul(sin(`theta'`)))
                    .add(_temp1.set(J).mul(J.dot(`jA'`) * (1.0 - cos(`theta'`))))
                    .normalize()

                if (alignAxis(
                        bodyAIdx = bodyAIdx,
                        bodyBIdx = bodyBIdx,
                        iia = bodyA.inverseInertia,
                        iib = bodyB.inverseInertia,
                        buf = n,
                        i = _temp2.set(`jA''`),
                        j = _temp3.set(`jB'`),
                    )
                ) {
                    ballToSolve += OFFSET_6_AA_DATA_SIZE
                }
            }

//            run twist@{
//                println("  TWIST")
//                val j = _J.set(`jA'`).add(`jB'`).normalize()
//
//                val `gA'` = qA.transform(gAX.toDouble(), gAY.toDouble(), gAZ.toDouble(), `_gA'`)
//                val `gB'` = qB.transform(gBX.toDouble(), gBY.toDouble(), gBZ.toDouble(), `_gB'`)
//
//                val pA = _pA.set(`gA'`).reject(j).normalize()
//                val pB = _pB.set(`gB'`).reject(j).normalize()
//                val theta = atan2(_temp1.set(pA).cross(pB).dot(j), pA.dot(pB))
//                if (!theta.isFinite()) return@twist
//                if (theta in minTwistAngle..maxTwistAngle) return@twist
//                
//                println("    gA': ${`gA'`.x} ${`gA'`.y} ${`gA'`.z}")
//                println("    gB': ${`gB'`.x} ${`gB'`.y} ${`gB'`.z}")
//                println("    theta: $theta")
//
//                val `theta'` = min(max(minTwistAngle.toDouble(), theta), maxTwistAngle.toDouble())
//                val `pA'` = `_pA'`.set(pA).mul(cos(`theta'`))
//                    .add(_temp1.set(j).cross(pA).mul(sin(`theta'`)))
//                    .add(_temp1.set(j).mul(j.dot(pA) * (1.0 - cos(`theta'`))))
//                    .normalize()
//
//                if (alignAxis(
//                        bodyAIdx = bodyAIdx,
//                        bodyBIdx = bodyBIdx,
//                        iia = bodyA.inverseInertia,
//                        iib = bodyB.inverseInertia,
//                        buf = n,
//                        i = _temp2.set(`pA'`),
//                        j = _temp3.set(pB),
//                    )
//                ) {
//                    ballToSolve += OFFSET_6_AA_DATA_SIZE
//                }
//            }
            /*
            for an revolute constraint with an revolute limit, we take in the following constraint data:
            jA = axis of rotation on A
            jB = axis of rotation on B
            gA = reference axis on A
            gB = reference axis on B
            phi< = min revolute
            phi> = max revolute
            
            we also have:
            qA, qB = rotations
            oA, oB = omegas
            
            if a vector is ', then it was rotated from local to world
            
            first, the normalized rotation axis will be:
            j = (jA' + jB') / 2
            then we project gA' and gB' onto j plane
            pA = gA' - j(gA' . j)
            pB = gB' - j(gB' . j)
            
            theta = atan2(pA x pB . j, pA . pB)
            
            if theta < phi< or theta > phi>
                theta' = clamp(theta, phi<, phi>)
                pA' = rotate pA around j by theta'
                align: pA' to pB
            
            the output is 2 axes that we need to align
             */
        }
    }

    fun solve() {
        run axis@{
            var i = 0
            while (i < axisToSolve) {
                solveAngleConstraint(constraintSolver.flatBodyData, jointData, i) { existing, calculated ->
                    max(0f, existing + calculated)
                }

                i += OFFSET_6_AA_DATA_SIZE
            }
        }

        run angle@{
            var i = 0
            while (i < angleToSolve) {
                solveAngleConstraint(constraintSolver.flatBodyData, limitedJointData, i) { existing, calculated ->
                    max(0f, existing + calculated)
                }

                i += OFFSET_6_AA_DATA_SIZE
            }
        }

        run ball@{
            var i = 0
            while (i < ballToSolve) {
                println("    solving: $i")
                solveAngleConstraint(constraintSolver.flatBodyData, revoluteJointData, i) { existing, calculated ->
                    max(0f, existing + calculated)
                }

                i += OFFSET_6_AA_DATA_SIZE
            }
        }
    }

    private inline fun alignAxis(
        bodyAIdx: Int,
        bodyBIdx: Int,
        iia: Matrix3d,
        iib: Matrix3d,
        buf: FloatBuffer,
        i: Vector3f,
        j: Vector3f,
    ): Boolean {
        if (!i.isFinite) return false
        if (!j.isFinite) return false
        val j0x = fma(i.y, j.z, -i.z * j.y)
        val j0y = fma(i.z, j.x, -i.x * j.z)
        val j0z = fma(i.x, j.y, -i.y * j.x)

        buf.put(j0x)
        buf.put(j0y)
        buf.put(j0z)

        val j0scaled = iia.transform(j0x.toDouble(), j0y.toDouble(), j0z.toDouble(), _j0scaled)

        buf.put(j0scaled.x.toFloat())
        buf.put(j0scaled.y.toFloat())
        buf.put(j0scaled.z.toFloat())

        val j1x = -j0x
        val j1y = -j0y
        val j1z = -j0z

        val j1scaled = iib.transform(j1x.toDouble(), j1y.toDouble(), j1z.toDouble(), _j1scaled)

        buf.put(j1scaled.x.toFloat())
        buf.put(j1scaled.y.toFloat())
        buf.put(j1scaled.z.toFloat())

        val b = -bias / timeStep * (1f - i.dot(j))
        buf.put(b)

        val den =
            j0scaled.dot(j0x.toDouble(), j0y.toDouble(), j0z.toDouble()).toFloat() +
            j1scaled.dot(j1x.toDouble(), j1y.toDouble(), j1z.toDouble()).toFloat()
        if (den < 1e-11) return false
        buf.put(den)

        buf.put(Float.fromBits(bodyAIdx))
        buf.put(Float.fromBits(bodyBIdx))

        buf.put(0f)
        return true
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
    
    println("      raw: $lambda ($den)")

    constraintData[OFFSET_6_AA_LAMBDA] = lambdaTransform(existing, lambda)
    val effective = constraintData[OFFSET_6_AA_LAMBDA] - existing
    
    println("      eff: $effective")

    bodyData[aIdx + 3] += v0x * effective
    bodyData[aIdx + 4] += v0y * effective
    bodyData[aIdx + 5] += v0z * effective

    bodyData[bIdx + 3] += v1x * effective
    bodyData[bIdx + 4] += v1y * effective
    bodyData[bIdx + 5] += v1z * effective
}

inline fun Vector3d.reject(x: Double, y: Double, z: Double): Vector3d {
    val m = dot(x, y, z)
    return sub(x * m, y * m, z * m)
}

inline fun Vector3d.reject(other: Vector3d): Vector3d {
    return reject(other.x, other.y, other.z)
}
