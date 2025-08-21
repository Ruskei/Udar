package com.ixume.udar.physics.constraint

import com.ixume.udar.physics.Contact
import org.joml.Quaterniond
import org.joml.Quaternionf
import org.joml.Vector3f
import java.nio.FloatBuffer
import kotlin.math.abs
import kotlin.math.max

class EnvConstraintSolver {
    private val j1 = Vector3f()

    private var allData = FloatArray(1)
    private lateinit var relevantContacts: List<Contact>

    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    fun setup(envConstraints: List<Contact>) {
        val total = envConstraints.size

        relevantContacts = envConstraints

        if (allData.size < total * A_CONTACT_FLOATS) {
            allData = FloatArray(max(total * A_CONTACT_FLOATS, allData.size * 2))
        }

        val buf = FloatBuffer.wrap(allData)

        //iterate through the colorset for this, anytime the count % PROCESSORS == proc, we add that
        //this is guaranteed to work since all workloads will read this data the same way
        for (contact in envConstraints) {
            buf.putVector3f(_vec3.set(contact.result.norm)) //n

            buf.putVector3f(_vec3.set(contact.first.velocity)) //first v
            buf.putVector3f(_quat.set(contact.first.q).transform(_vec3.set(contact.first.omega))) //first o

            buf.putVector3f(
                j1.set(contact.first.pos).sub(_vec3.set(contact.result.point)).cross(_vec3.set(contact.result.norm))
            ) //j1

            val timestep = 0.005f
            val slop = 0.0001f

            buf.put(0.1f / timestep * (abs(contact.result.depth.toFloat()) - slop).coerceAtLeast(0f)) // bias

            buf.put(
                j0Temp.set(
                    contact.result.norm.x * contact.result.norm.x,
                    contact.result.norm.y * contact.result.norm.y,
                    contact.result.norm.z * contact.result.norm.z
                )
                    .dot(
                        contact.first.inverseMass.toFloat(),
                        contact.first.inverseMass.toFloat(),
                        contact.first.inverseMass.toFloat(),
                    ) + j1Temp.set(j1).mul(contact.first.inverseInertia).dot(j1)
            ) // den

            buf.put(contact.lambdaSum.toFloat()) //lambda

            buf.putVector3f(
                tempIMA.set(contact.first.inverseMass)
                    .mul(
                        -contact.result.norm.x.toFloat(),
                        -contact.result.norm.y.toFloat(),
                        -contact.result.norm.z.toFloat()
                    )
            ) // DVA
            buf.putVector3f(j1Temp.set(j1).mul(contact.first.inverseInertia)) // DOA
        }
    }

    fun solve() {
        var i = 0
        while (i < relevantContacts.size) {
            solveNormal(i * A_CONTACT_FLOATS)

            i++
        }
    }

    fun write() {
        var i = 0
        while (i < relevantContacts.size) {
            val contact = relevantContacts[i]

            val first = contact.first

            first.velocity.from(i * A_CONTACT_FLOATS + A_FIRST_V_OFFSET, allData)

            first.omega.from(i * A_CONTACT_FLOATS + A_FIRST_O_OFFSET, allData).rotate(_quatd.set(first.q).conjugate())

            contact.lambdaSum = allData[i * A_CONTACT_FLOATS + A_LAMBDA_OFFSET].toDouble()

            i++
        }
    }

    private fun solveNormal(start: Int) {
        val nx = allData[start + A_NORMAL_OFFSET]
        val ny = allData[start + A_NORMAL_OFFSET + 1]
        val nz = allData[start + A_NORMAL_OFFSET + 2]

        //set nn, j1, and j3
        j0Temp.set(-nx, -ny, -nz)
        j1.from(start + A_J1_OFFSET, allData)

        val bias = allData[start + A_BIAS_OFFSET]

        val den = allData[start + A_DENOMINATOR_OFFSET]

        var lambda =
            Math.fma(
                -nx,
                allData[start + A_FIRST_V_OFFSET],
                Math.fma(
                    -ny,
                    allData[start + A_FIRST_V_OFFSET + 1],
                    Math.fma(
                        -nz,
                        allData[start + A_FIRST_V_OFFSET + 2],
                        Math.fma(
                            allData[start + A_J1_OFFSET],
                            allData[start + A_FIRST_O_OFFSET],
                            Math.fma(
                                allData[start + A_J1_OFFSET + 1],
                                allData[start + A_FIRST_O_OFFSET + 1],
                                Math.fma(
                                    allData[start + A_J1_OFFSET + 2],
                                    allData[start + A_FIRST_O_OFFSET + 2],
                                    bias
                                )
                            )
                        )
                    )
                )
            ) / den

        val l = allData[start + A_LAMBDA_OFFSET]

        val curLambdaSum = l
        allData[start + A_LAMBDA_OFFSET] = max(0f, curLambdaSum + lambda)
        lambda = allData[start + A_LAMBDA_OFFSET] - curLambdaSum
        //firstV.sub(dVA)
        allData[start + A_BASE_OFFSET] -= (allData[start + A_DELTA_BASE_OFFSET] * lambda)
        allData[start + A_BASE_OFFSET + 1] -= (allData[start + A_DELTA_BASE_OFFSET + 1] * lambda)
        allData[start + A_BASE_OFFSET + 2] -= (allData[start + A_DELTA_BASE_OFFSET + 2] * lambda)
        //firstO.sub(dOA)
        allData[start + A_BASE_OFFSET + 3] -= (allData[start + A_DELTA_BASE_OFFSET + 3] * lambda)
        allData[start + A_BASE_OFFSET + 4] -= (allData[start + A_DELTA_BASE_OFFSET + 4] * lambda)
        allData[start + A_BASE_OFFSET + 5] -= (allData[start + A_DELTA_BASE_OFFSET + 5] * lambda)
    }

    private val tempIMA = Vector3f()

    private val j1Temp = Vector3f()
    private val j0Temp = Vector3f()

    private val _quatd = Quaterniond()
}

private const val A_NORMAL_OFFSET = 0 // constant

private const val A_BASE_OFFSET = 3
private const val A_FIRST_V_OFFSET = 3
private const val A_FIRST_O_OFFSET = 6

private const val A_J1_OFFSET = 9 // constant

private const val A_BIAS_OFFSET = 12 // constant
private const val A_DENOMINATOR_OFFSET = 13 // constant
private const val A_LAMBDA_OFFSET = 14 // constant

private const val A_DELTA_BASE_OFFSET = 15 // constant, 2 vec3s

private const val A_CONTACT_FLOATS = 21