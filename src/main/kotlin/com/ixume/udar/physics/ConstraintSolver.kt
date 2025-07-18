package com.ixume.udar.physics

import com.ixume.udar.Udar
import com.ixume.udar.body.ActiveBody
import com.ixume.udar.body.ActiveBody.Companion.TIME_STEP
import org.joml.Quaterniond
import org.joml.Vector3d
import kotlin.math.abs

object ContactsSolver {
    private const val NORMAL_ITERATIONS = 4
    private const val FRICTION_ITERATIONS = 4
    private const val SIGNIFICANT_LAMBDA = 1e-6

    fun solve(contacts: List<Contact>) {
        var itr = 1
        while (itr <= NORMAL_ITERATIONS) {
            var itrSignificant = false
            for (contact in contacts) {
                val first = contact.first
                val second = contact.second

                val point = contact.result.point
                val n = contact.result.norm

//                                //V = [ vA, wA, vB, wB ]
//                                // vA and vB are given, wA and wB are js rotated w
                val vA = first.velocity
                val wA = Vector3d(first.omega).rotate(first.q)
                val vB = second.velocity
                val wB = Vector3d(second.omega).rotate(second.q)
//
//                                //M^(-1) = [ iMA, iIA, iMB, iAB ]
                val iMA = Vector3d(first.inverseMass)
                val iIA = first.inverseInertia
                val iMB = Vector3d(second.inverseMass)
                val iIB = second.inverseInertia

                val rA = Vector3d(point).sub(first.pos)
                val rB = Vector3d(point).sub(second.pos)

                //lambda = -(JV - b) / (JM^(-1)J^(T))

                //J = [ -n, (-rA x n), n, (rB x n) ]
                // n is given, rA is point - center
                val nn = Vector3d(n).mul(-1.0)
                val j1 = Vector3d(rA).negate().cross(n)
                val j3 = Vector3d(rB).cross(n)

                val (dVA, dOA, dVB, dOB, significant) = deltaV(
                    contact = contact,
                    type = DeltaType.NORMAL,
                    j0 = nn, j1 = j1, j2 = n, j3 = j3,
                    iMA = iMA, iIA = iIA, iMB = iMB, iIB = iIB,
                    vA = vA, wA = wA, vB = vB, wB = wB,
                )

                if (significant) itrSignificant = true

                first.velocity.sub(dVA)
                first.omega.sub(dOA)
                second.velocity.sub(dVB)
                second.omega.sub(dOB)

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
                val n = contact.result.norm

//                                //V = [ vA, wA, vB, wB ]
//                                // vA and vB are given, wA and wB are js rotated w
                val vA = first.velocity
                val wA = Vector3d(first.omega).rotate(first.q)
                val vB = second.velocity
                val wB = Vector3d(second.omega).rotate(second.q)
//
//                                //M^(-1) = [ iMA, iIA, iMB, iAB ]
                val iMA = Vector3d(first.inverseMass)
                val iIA = first.inverseInertia
                val iMB = Vector3d(second.inverseMass)
                val iIB = second.inverseInertia

                val rA = Vector3d(point).sub(first.pos)
                val rB = Vector3d(point).sub(second.pos)

                run t1@{
                    val j0 = Vector3d(contact.t1).negate()
                    val j1 = Vector3d(rA).negate().cross(contact.t1)
                    val j2 = Vector3d(contact.t1)
                    val j3 = Vector3d(rB).cross(contact.t1)

                    val (dVA, dOA, dVB, dOB, significant) = deltaV(
                        contact = contact,
                        type = DeltaType.T1,
                        j0 = j0, j1 = j1, j2 = j2, j3 = j3,
                        iMA = iMA, iIA = iIA, iMB = iMB, iIB = iIB,
                        vA = vA, wA = wA, vB = vB, wB = wB,
                    )

                    first.velocity.sub(dVA)
                    first.omega.sub(dOA)
                    second.velocity.sub(dVB)
                    second.omega.sub(dOB)

                    if (significant) itrSignificant = true
                }

                run t2@{
                    val j0 = Vector3d(contact.t2).negate()
                    val j1 = Vector3d(rA).negate().cross(contact.t2)
                    val j2 = Vector3d(contact.t2)
                    val j3 = Vector3d(rB).cross(contact.t2)

                    val (dVA, dOA, dVB, dOB, significant) = deltaV(
                        contact = contact,
                        type = DeltaType.T2,
                        j0 = j0, j1 = j1, j2 = j2, j3 = j3,
                        iMA = iMA, iIA = iIA, iMB = iMB, iIB = iIB,
                        vA = vA, wA = wA, vB = vB, wB = wB,
                    )

                    first.velocity.sub(dVA)
                    first.omega.sub(dOA)
                    second.velocity.sub(dVB)
                    second.omega.sub(dOB)

                    if (significant) itrSignificant = true
                }
            }

            if (!itrSignificant) {
                break
            }
        }
    }

    data class DeltaV(
        val dVA: Vector3d,
        val dOA: Vector3d,
        val dVB: Vector3d,
        val dOB: Vector3d,
        val significant: Boolean,
    )

    private enum class DeltaType {
        NORMAL, T1, T2
    }

    private fun deltaV(
        contact: Contact, type: DeltaType,
        j0: Vector3d, j1: Vector3d, j2: Vector3d, j3: Vector3d,
        iMA: Vector3d, iIA: Vector3d, iMB: Vector3d, iIB: Vector3d,
        vA: Vector3d, wA: Vector3d, vB: Vector3d, wB: Vector3d,
    ): DeltaV {
        val depth = contact.result.depth

        val slop =
            if (contact.first is ActiveBody && contact.second is ActiveBody) Udar.CONFIG.collision.activeSlop else Udar.CONFIG.collision.passiveSlop

        val bias = if (type == DeltaType.NORMAL) Udar.CONFIG.collision.bias / TIME_STEP * (abs(depth) - slop).coerceAtLeast(0.0) else 0.0

        val den =
            Vector3d(j0).mul(j0).dot(iMA) + Vector3d(j1).mul(j1).dot(iIA) + Vector3d(j2).mul(j2)
                .dot(iMB) + Vector3d(j3).mul(j3).dot(iIB)

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
        val dVA = Vector3d(iMA).mul(j0).mul(lambda)
        val dOA = Vector3d(iIA).mul(j1).mul(lambda).rotate(Quaterniond(contact.first.q).conjugate())
        val dVB = Vector3d(iMB).mul(j2).mul(lambda)
        val dOB = Vector3d(iIB).mul(j3).mul(lambda).rotate(Quaterniond(contact.second.q).conjugate())

        return DeltaV(
            dVA,
            dOA,
            dVB,
            dOB,
            lambda > SIGNIFICANT_LAMBDA,
        )
    }
}