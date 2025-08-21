package com.ixume.udar.physics.constraint

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.Contact
import org.joml.Quaterniond
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.nio.FloatBuffer
import kotlin.math.max

class LocalConstraintSolver {
    private val j1 = Vector3f()
    private val j3 = Vector3f()

    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    private lateinit var idMap: Array<ActiveBody>
    private var flatBodyData: FloatArray = FloatArray(1)
    private var bodyCount = 0

    private lateinit var constraintMap: Array<Contact>
    private lateinit var flatContactData: FloatArray

    private lateinit var envConstraintMap: Array<Contact>
    private lateinit var envContactData: FloatArray

    /**
     * @param map The id -> ActiveBody map
     * @param flatData Flattened body data structs
     */
    fun setup(
        map: Array<ActiveBody>,

        flatData: FloatArray,
        constraintMap: Array<Contact>,

        flatEnvData: FloatArray,
        envConstraintMap: Array<Contact>,
    ) {
        this.flatContactData = flatData
        bodyCount = map.size

        this.idMap = map
        this.constraintMap = constraintMap
        this.envContactData = flatEnvData
        this.envConstraintMap = envConstraintMap

        buildFlatBodyData(map)
    }

    private fun buildFlatBodyData(arr: Array<ActiveBody>) {
        val n = arr.size
        if (flatBodyData.size < n * BASE_SIZE) { // resizing is fine, since all the valid bodies should set their data anyway
            flatBodyData = FloatArray(max(flatBodyData.size * 2, n * BASE_SIZE))
        }

        val buf = FloatBuffer.wrap(flatBodyData)

        var i = 0
        while (i < bodyCount) {
            val b = arr[i]

            buf.putVector3f(_vec3.set(b.velocity))
            buf.putVector3f(_vec3.set(b.omega).rotate(_quat.set(b.q)))

            i++
        }
    }

    private val j0Temp = Vector3f()
    private val j2Temp = Vector3f()

    private val _quatd = Quaterniond()

    fun solveNormal() {
        var i = 0
        val n = flatContactData.size

        while (i < n) {
            solveA2AContact(i)

            i += A2A_CONTACT_DATA_FLOATS
        }

        var j = 0
        val e = envContactData.size

        while (j < e) {
            solveA2SContact(j)

            j += A2S_CONTACT_DATA_FLOATS
        }
    }

    fun solveFriction() {
        var i = 0
        val n = flatContactData.size

        while (i < n) {
            solveA2AContact(i)

            i += A2A_CONTACT_DATA_FLOATS
        }

        var j = 0
        val e = envContactData.size

        while (j < e) {
            solveA2SContact(j)

            j += A2S_CONTACT_DATA_FLOATS
        }
    }

    /**
     * For a contact between active objects
     */
    private fun solveA2AContact(contactIdx: Int) {
        val nx = flatContactData[contactIdx + A2A_NORMAL_OFFSET]
        val ny = flatContactData[contactIdx + A2A_NORMAL_OFFSET + 1]
        val nz = flatContactData[contactIdx + A2A_NORMAL_OFFSET + 2]

        //set nn, j1, and j3
        j0Temp.set(-nx, -ny, -nz)
        j1.from(contactIdx + A2A_J1_OFFSET, flatContactData)
        j2Temp.set(nx, ny, nz)
        j3.from(contactIdx + A2A_J3_OFFSET, flatContactData)

        val bias = flatContactData[contactIdx + A2A_BIAS_OFFSET]

        val den = flatContactData[contactIdx + A2A_DEN_OFFSET]

        val myIdx = flatContactData[contactIdx + A2A_MY_IDX_OFFSET].toBits() * BASE_SIZE

        val vAx = flatBodyData[myIdx + V_OFFSET]
        val vAy = flatBodyData[myIdx + V_OFFSET + 1]
        val vAz = flatBodyData[myIdx + V_OFFSET + 2]

        val oAx = flatBodyData[myIdx + O_OFFSET]
        val oAy = flatBodyData[myIdx + O_OFFSET + 1]
        val oAz = flatBodyData[myIdx + O_OFFSET + 2]

        val otherIdx = flatContactData[contactIdx + A2A_OTHER_IDX_OFFSET].toBits() * BASE_SIZE

        val vBx = flatBodyData[otherIdx + V_OFFSET]
        val vBy = flatBodyData[otherIdx + V_OFFSET + 1]
        val vBz = flatBodyData[otherIdx + V_OFFSET + 2]

        val oBx = flatBodyData[otherIdx + O_OFFSET]
        val oBy = flatBodyData[otherIdx + O_OFFSET + 1]
        val oBz = flatBodyData[otherIdx + O_OFFSET + 2]

        var lambda =
            Math.fma(
                -nx,
                vAx,
                Math.fma(
                    -ny,
                    vAy,
                    Math.fma(
                        -nz,
                        vAz,
                        Math.fma(
                            flatContactData[contactIdx + A2A_J1_OFFSET],
                            oAx,
                            Math.fma(
                                flatContactData[contactIdx + A2A_J1_OFFSET + 1],
                                oAy,
                                Math.fma(
                                    flatContactData[contactIdx + A2A_J1_OFFSET + 2],
                                    oAz,
                                    Math.fma(
                                        nx,
                                        vBx,
                                        Math.fma(
                                            ny,
                                            vBy,
                                            Math.fma(
                                                nz,
                                                vBz,
                                                Math.fma(
                                                    flatContactData[contactIdx + A2A_J3_OFFSET],
                                                    oBx,
                                                    Math.fma(
                                                        flatContactData[contactIdx + A2A_J3_OFFSET + 1],
                                                        oBy,
                                                        Math.fma(
                                                            flatContactData[contactIdx + A2A_J3_OFFSET + 2],
                                                            oBz,
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

        val l = flatContactData[contactIdx + A2A_LAMBDA_OFFSET]

        val cls = l
        flatContactData[contactIdx + A2A_LAMBDA_OFFSET] = max(0f, cls + lambda)
        lambda = flatContactData[contactIdx + A2A_LAMBDA_OFFSET] - cls

        flatBodyData[myIdx] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET] * lambda)
        flatBodyData[myIdx + 1] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 1] * lambda)
        flatBodyData[myIdx + 2] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 2] * lambda)

        flatBodyData[myIdx + 3] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 3] * lambda)
        flatBodyData[myIdx + 4] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 4] * lambda)
        flatBodyData[myIdx + 5] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 5] * lambda)

        flatBodyData[otherIdx] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 6] * lambda)
        flatBodyData[otherIdx + 1] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 7] * lambda)
        flatBodyData[otherIdx + 2] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 8] * lambda)

        flatBodyData[otherIdx + 3] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 9] * lambda)
        flatBodyData[otherIdx + 4] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 10] * lambda)
        flatBodyData[otherIdx + 5] -= (flatContactData[contactIdx + A2A_DELTA_OFFSET + 11] * lambda)
    }

    /**
     * For a contact between an active body and a static object
     */
    private fun solveA2SContact(contactIdx: Int) {
        val nx = envContactData[contactIdx + A2S_NORMAL_OFFSET]
        val ny = envContactData[contactIdx + A2S_NORMAL_OFFSET + 1]
        val nz = envContactData[contactIdx + A2S_NORMAL_OFFSET + 2]

        //set nn, j1, and j3
        j0Temp.set(-nx, -ny, -nz)
        j1.from(contactIdx + A2S_J1_OFFSET, envContactData)

        val bias = envContactData[contactIdx + A2S_BIAS_OFFSET]

        val den = envContactData[contactIdx + A2S_DEN_OFFSET]

        val myIdx = envContactData[contactIdx + A2S_MY_IDX_OFFSET].toBits() * BASE_SIZE

        val vAx = flatBodyData[myIdx + V_OFFSET]
        val vAy = flatBodyData[myIdx + V_OFFSET + 1]
        val vAz = flatBodyData[myIdx + V_OFFSET + 2]

        val oAx = flatBodyData[myIdx + O_OFFSET]
        val oAy = flatBodyData[myIdx + O_OFFSET + 1]
        val oAz = flatBodyData[myIdx + O_OFFSET + 2]

        var lambda =
            Math.fma(
                -nx,
                vAx,
                Math.fma(
                    -ny,
                    vAy,
                    Math.fma(
                        -nz,
                        vAz,
                        Math.fma(
                            envContactData[contactIdx + A2S_J1_OFFSET],
                            oAx,
                            Math.fma(
                                envContactData[contactIdx + A2S_J1_OFFSET + 1],
                                oAy,
                                Math.fma(
                                    envContactData[contactIdx + A2S_J1_OFFSET + 2],
                                    oAz,
                                    bias
                                )
                            )
                        )
                    )
                )
            ) / den

        val l = envContactData[contactIdx + A2S_LAMBDA_OFFSET]

        val cls = l
        envContactData[contactIdx + A2S_LAMBDA_OFFSET] = max(0f, cls + lambda)
        lambda = envContactData[contactIdx + A2S_LAMBDA_OFFSET] - cls

        flatBodyData[myIdx] -= (envContactData[contactIdx + A2S_DELTA_OFFSET] * lambda)
        flatBodyData[myIdx + 1] -= (envContactData[contactIdx + A2S_DELTA_OFFSET + 1] * lambda)
        flatBodyData[myIdx + 2] -= (envContactData[contactIdx + A2S_DELTA_OFFSET + 2] * lambda)

        flatBodyData[myIdx + 3] -= (envContactData[contactIdx + A2S_DELTA_OFFSET + 3] * lambda)
        flatBodyData[myIdx + 4] -= (envContactData[contactIdx + A2S_DELTA_OFFSET + 4] * lambda)
        flatBodyData[myIdx + 5] -= (envContactData[contactIdx + A2S_DELTA_OFFSET + 5] * lambda)
    }

    fun write() {
        var i = 0
        val n = bodyCount * BASE_SIZE
        while (i < n) {
            val body = idMap[i / BASE_SIZE]

            body.velocity.from(i + V_OFFSET, flatBodyData)
            body.omega.from(i + O_OFFSET, flatBodyData).rotate(_quatd.set(body.q).conjugate())

            i += BASE_SIZE
        }

        var j = 0
        while (j < flatContactData.size) {
            val contact = constraintMap[j / A2A_CONTACT_DATA_FLOATS]

            contact.lambdaSum = flatContactData[j + A2A_LAMBDA_OFFSET].toDouble()

            j += A2A_CONTACT_DATA_FLOATS
        }
    }
}

fun FloatBuffer.putVector3f(v: Vector3f): FloatBuffer {
    put(v.x)
    put(v.y)
    put(v.z)

    return this
}

inline fun Vector3f.from(idx: Int, arr: FloatArray): Vector3f {
    x = arr[idx]
    y = arr[idx + 1]
    z = arr[idx + 2]

    return this
}

inline fun Vector3d.from(idx: Int, arr: FloatArray): Vector3d {
    x = arr[idx].toDouble()
    y = arr[idx + 1].toDouble()
    z = arr[idx + 2].toDouble()

    return this
}
