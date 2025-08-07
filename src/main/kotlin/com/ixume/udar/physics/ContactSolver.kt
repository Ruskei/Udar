package com.ixume.udar.physics

import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import org.joml.Matrix3d
import org.joml.Quaterniond
import org.joml.Vector3d
import kotlin.math.abs

object ContactSolver {
    private const val NORMAL_ITERATIONS = 4
    private const val FRICTION_ITERATIONS = 4
    private const val SIGNIFICANT_LAMBDA = 1e-6

    private val vA = Vector3d()
    private val wA = Vector3d()
    private val vB = Vector3d()
    private val wB = Vector3d()

    private val iMA = Vector3d()
    private val iIA = Matrix3d()
    private val iMB = Vector3d()
    private val iIB = Matrix3d()

    private val n = Vector3d()
    private val nn = Vector3d()

    private val j0 = Vector3d()
    private val j1 = Vector3d()
    private val j2 = Vector3d()
    private val j3 = Vector3d()

    fun solve(contacts: List<IContact>) {
        var itr = 1
        while (itr <= NORMAL_ITERATIONS) {
            var itrSignificant = false
            for (contact in contacts) {
                val first = contact.first
                val second = contact.second

                val point = contact.result.point
                n.set(contact.result.norm)
//                                //V = [ vA, wA, vB, wB ]
//                                // vA and vB are given, wA and wB are js rotated w
                vA.set(first.velocity)
                wA.set(first.omega).rotate(first.q)
                vB.set(second.velocity)
                wB.set(second.omega).rotate(second.q)
//
//                                //M^(-1) = [ iMA, iIA, iMB, iAB ]
                iMA.set(first.inverseMass)
                iIA.set(first.inverseInertia)
                iMB.set(second.inverseMass)
                iIB.set(second.inverseInertia)

                //lambda = -(JV - b) / (JM^(-1)J^(T))

                //J = [ -n, (-rA x n), n, (rB x n) ]
                // n is given, rA is point - center
                nn.set(n).negate()
                j1.set(first.pos).sub(point).cross(n)
                j3.set(point).sub(second.pos).cross(n)

                val significant = deltaV(
                    contact = contact,
                    type = DeltaType.NORMAL,
                    j0 = nn, j1 = j1, j2 = n, j3 = j3,
                    iMA = iMA, iIA = iIA, iMB = iMB, iIB = iIB,
                    vA = vA, wA = wA, vB = vB, wB = wB,
                    first.velocity, first.omega, second.velocity, second.omega,
                )

                if (significant) itrSignificant = true


//                                println("ITR: $itr")
//                                println("  - FIRST IM: ${first.inverseMass}")
//                                println("  - FIRST V: ${first.velocity}")
//                                println("  - SECOND IM: ${second.inverseMass}")
//                                println("  - SECOND V: ${second.velocity}")
//                                println("-- lambda: $lambda")
//                                println("-- dVA: $dVA")
//                                println("-- dOA: $dOA")
//                                println("-- dVB: $dVB")
//                                println("-- dOB: $dOB")


                //friction is same as normal but find orthonormal basis from normal

            }

            if (!itrSignificant) {
                break
            }

            itr++
        }

//        println("DID $itr ITERATIONS")

        for (itr in 1..FRICTION_ITERATIONS) {
            var itrSignificant = false
            for (contact in contacts) {
                val first = contact.first
                val second = contact.second

                val point = contact.result.point
                n.set(contact.result.norm)
//                                //V = [ vA, wA, vB, wB ]
//                                // vA and vB are given, wA and wB are js rotated w
                vA.set(first.velocity)
                wA.set(first.omega).rotate(first.q)
                vB.set(second.velocity)
                wB.set(second.omega).rotate(second.q)
//
//                                //M^(-1) = [ iMA, iIA, iMB, iAB ]
                iMA.set(first.inverseMass)
                iIA.set(first.inverseInertia)
                iMB.set(second.inverseMass)
                iIB.set(second.inverseInertia)

                run t1@{
                    j0.set(contact.t1).negate()
                    j1.set(first.pos).sub(point).cross(contact.t1)
                    j2.set(contact.t1)
                    j3.set(point).sub(second.pos).cross(contact.t1)

                    val significant = deltaV(
                        contact = contact,
                        type = DeltaType.T1,
                        j0 = j0, j1 = j1, j2 = j2, j3 = j3,
                        iMA = iMA, iIA = iIA, iMB = iMB, iIB = iIB,
                        vA = vA, wA = wA, vB = vB, wB = wB,
                        first.velocity, first.omega, second.velocity, second.omega,
                    )

                    if (significant) itrSignificant = true
                }

                run t2@{
                    j0.set(contact.t2).negate()
                    j1.set(first.pos).sub(point).cross(contact.t2)
                    j2.set(contact.t2)
                    j3.set(point).sub(second.pos).cross(contact.t2)

                    val significant = deltaV(
                        contact = contact,
                        type = DeltaType.T2,
                        j0 = j0, j1 = j1, j2 = j2, j3 = j3,
                        iMA = iMA, iIA = iIA, iMB = iMB, iIB = iIB,
                        vA = vA, wA = wA, vB = vB, wB = wB,
                        first.velocity, first.omega, second.velocity, second.omega,
                    )

                    if (significant) itrSignificant = true
                }
            }

            if (!itrSignificant) {
                break
            }
        }
    }

    class DeltaV(
        val dVA: Vector3d,
        val dOA: Vector3d,
        val dVB: Vector3d,
        val dOB: Vector3d,
        val significant: Boolean,
    )

    private enum class DeltaType {
        NORMAL, T1, T2
    }

    private val tempIMA = Vector3d()
    private val tempIMB = Vector3d()

    private val j1Temp = Vector3d()
    private val j2Temp = Vector3d()
    private val j3Temp = Vector3d()
    private val j0Temp = Vector3d()

    private val firstQ = Quaterniond()
    private val secondQ = Quaterniond()

    private fun deltaV(
        contact: IContact, type: DeltaType,
        j0: Vector3d, j1: Vector3d, j2: Vector3d, j3: Vector3d,
        iMA: Vector3d, iIA: Matrix3d, iMB: Vector3d, iIB: Matrix3d,
        vA: Vector3d, wA: Vector3d, vB: Vector3d, wB: Vector3d,
        firstV: Vector3d, firstO: Vector3d, secondV: Vector3d, secondO: Vector3d
    ): Boolean {
        val depth = contact.result.depth

        val slop =
            if (contact.first is ActiveBody && contact.second is ActiveBody) Udar.CONFIG.collision.activeSlop else Udar.CONFIG.collision.passiveSlop

        val bias = if (type == DeltaType.NORMAL) Udar.CONFIG.collision.bias / TIME_STEP * (abs(depth) - slop).coerceAtLeast(0.0) else 0.0

        j0Temp.set(j0)
        j1Temp.set(j1)
        j2Temp.set(j2)
        j3Temp.set(j3)

        val den =
            j0Temp.mul(j0).dot(iMA) + j1Temp.mul(iIA).dot(j1) + j2Temp.mul(j2)
                .dot(iMB) + j3Temp.mul(iIB).dot(j3)

        var lambda = (j0.dot(vA) + j1.dot(wA) + j2.dot(vB) + j3.dot(wB) + bias) / den

        when (type) {
            DeltaType.NORMAL -> {
                val curLambdaSum = contact.lambdaSum
                contact.lambdaSum = (curLambdaSum + lambda).coerceAtLeast(0.0)
                lambda = contact.lambdaSum - curLambdaSum
            }

            DeltaType.T1 -> {
                val curT1Sum = contact.t1Sum
                contact.t1Sum =
                    (curT1Sum + lambda).coerceIn(-Udar.CONFIG.collision.friction * contact.lambdaSum, Udar.CONFIG.collision.friction * contact.lambdaSum)
                lambda = contact.t1Sum - curT1Sum
            }

            DeltaType.T2 -> {
                val curT2Sum = contact.t2Sum
                contact.t2Sum =
                    (curT2Sum + lambda).coerceIn(-Udar.CONFIG.collision.friction * contact.lambdaSum, Udar.CONFIG.collision.friction * contact.lambdaSum)
                lambda = contact.t2Sum - curT2Sum
            }
        }

        //delta-V = M^(-1)J^T * lambda
        val dVA = tempIMA.set(iMA).mul(j0).mul(lambda)
        val dOA = j1Temp.set(j1).mul(iIA).mul(lambda).rotate(firstQ.set(contact.first.q).conjugate())
        val dVB = tempIMB.set(iMB).mul(j2).mul(lambda)
        val dOB = j3Temp.set(j3).mul(iIB).mul(lambda).rotate(secondQ.set(contact.second.q).conjugate())

        firstV.sub(dVA)
        firstO.sub(dOA)
        secondV.sub(dVB)
        secondO.sub(dOB)

        return lambda > SIGNIFICANT_LAMBDA
    }
}