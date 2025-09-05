package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.contact.A2AContactBuffer
import com.ixume.udar.physics.contact.A2SContactBuffer
import org.joml.Matrix3d
import org.joml.Matrix3f
import org.joml.Quaterniond
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.Math
import java.nio.FloatBuffer
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class LocalConstraintSolver(
    val physicsWorld: PhysicsWorld,
) {
    private val timeStep = Udar.CONFIG.timeStep.toFloat()
    private val friction = Udar.CONFIG.collision.friction.toFloat()
    private val bias = Udar.CONFIG.collision.bias.toFloat()
    private val slop = Udar.CONFIG.collision.passiveSlop.toFloat()

    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    private var idMap: Array<ActiveBody>
    private var bodyCount: Int
    @Volatile
    private var flatBodyData: FloatArray = FloatArray(1)

    private var any = false

    init {
        val snapshot = physicsWorld.bodiesSnapshot()
        bodyCount = snapshot.size
        idMap = physicsWorld.bodiesSnapshot().let {
            Array(it.size) { i ->
                val b = it[i]
                b.id = i
                b
            }
        }
    }

    private lateinit var contacts: A2AContactBuffer
    private lateinit var envContacts: A2SContactBuffer
    private var contactNormalData: FloatArray = FloatArray(1)

    private var contactT1Data: FloatArray = FloatArray(1)
    private var contactT2Data: FloatArray = FloatArray(1)

    private var envContactNormalData: FloatArray = FloatArray(1)

    private var envContactT1Data: FloatArray = FloatArray(1)
    private var envContactT2Data: FloatArray = FloatArray(1)
    
    fun prepare() {
        updateIDMap()
        bodyCount = idMap.size
    }

    fun setup() {
        any = true
        if (bodyCount == 0) {
            any = false
            return
        }

        contacts = physicsWorld.contactBuffer
        envContacts = physicsWorld.envContactBuffer

        if (contacts.isEmpty() && envContacts.isEmpty()) {
            any = false
            return
        }

        constructFlatConstraintData(ContactComponent.NORMAL)
        constructFlatEnvConstraintData(ContactComponent.NORMAL)

        constructFlatConstraintData(ContactComponent.T1)
        constructFlatEnvConstraintData(ContactComponent.T1)

        constructFlatConstraintData(ContactComponent.T2)
        constructFlatEnvConstraintData(ContactComponent.T2)

        buildFlatBodyData()
    }

    fun updateIDMap() {
        val ls = physicsWorld.bodiesSnapshot()
        idMap = Array(ls.size) {
            val b = ls[it]
            b.id = it
            b
        }
    }

    private val _c_vec3 = Vector3f()
    private val _c_j0 = Vector3f()
    private val _c_j1 = Vector3f()
    private val _c_j1Temp = Vector3f()
    private val _c_j2 = Vector3f()
    private val _c_j3 = Vector3f()
    private val _c_j3Temp = Vector3f()
    private val _c_im = Vector3f()
    private val _norm = Vector3f()
    private val _c_tempMatrix3f1 = Matrix3f()
    private val _c_tempMatrix3f2 = Matrix3f()

    private fun constructFlatConstraintData(component: ContactComponent) {
        val numc = contacts.size()
        val relevantData = when (component) {
            ContactComponent.NORMAL -> {
                if (contactNormalData.size < numc * A2A_N_CONTACT_DATA_FLOATS) {
                    contactNormalData = FloatArray(max(contactNormalData.size * 2, numc * A2A_N_CONTACT_DATA_FLOATS))
                }

                contactNormalData
            }

            ContactComponent.T1 -> {
                if (contactT1Data.size < numc * A2A_N_CONTACT_DATA_FLOATS) {
                    contactT1Data = FloatArray(max(contactT1Data.size * 2, numc * A2A_N_CONTACT_DATA_FLOATS))
                }

                contactT1Data
            }

            ContactComponent.T2 -> {
                if (contactT2Data.size < numc * A2A_N_CONTACT_DATA_FLOATS) {
                    contactT2Data = FloatArray(max(contactT2Data.size * 2, numc * A2A_N_CONTACT_DATA_FLOATS))
                }

                contactT2Data
            }
        }

        val n = FloatBuffer.wrap(relevantData)
        var i = 0
        while (i < numc) {
            when (component) {
                ContactComponent.NORMAL -> _norm.set(contacts.normX(i), contacts.normY(i), contacts.normZ(i))
                ContactComponent.T1 -> _norm.set(contacts.t1X(i), contacts.t1Y(i), contacts.t1Z(i))
                ContactComponent.T2 -> _norm.set(contacts.t2X(i), contacts.t2Y(i), contacts.t2Z(i))
            }

            n.putVector3f(_norm)

            n.putVector3f(
                _c_j1
                    .set(contacts.bodyAPosX(i), contacts.bodyAPosY(i), contacts.bodyAPosZ(i))
                    .sub(contacts.pointAX(i), contacts.pointAY(i), contacts.pointAZ(i))
                    .cross(_norm)
            ) //j1

            n.putVector3f(
                _c_j3
                    .set(contacts.pointBX(i), contacts.pointBY(i), contacts.pointBZ(i))
                    .sub(contacts.bodyBPosX(i), contacts.bodyBPosY(i), contacts.bodyBPosZ(i))
                    .cross(_norm)
            ) //j3
            
            val depth = contacts.depth(i)

            if (component == ContactComponent.NORMAL) {
                val tb = bias / timeStep * max(0f, abs(depth) - slop)
                n.put(tb) // bias
            } else {
                n.put(0f)
            }

            val bodyAInverseMass = contacts.bodyAInverseMass(i)
            val bodyBInverseMass = contacts.bodyBInverseMass(i)
            val bodyAInverseInertia = contacts.bodyAInverseInertia(i, _c_tempMatrix3f1)
            val bodyBInverseInertia = contacts.bodyBInverseInertia(i, _c_tempMatrix3f2)
            
            val den =
                _c_j0.set(
                    _norm.x * _norm.x,
                    _norm.y * _norm.y,
                    _norm.z * _norm.z
                ).dot(
                    bodyAInverseMass,
                    bodyAInverseMass,
                    bodyAInverseMass,
                ) +
                _c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia).dot(_c_j1) +
                _c_j2.set(
                    _norm.x * _norm.x,
                    _norm.y * _norm.y,
                    _norm.z * _norm.z
                ).dot(
                    bodyBInverseMass,
                    bodyBInverseMass,
                    bodyBInverseMass,
                ) +
                _c_j3Temp.set(_c_j3)._mul(bodyBInverseInertia).dot(_c_j3)
            

            n.put(den) // den

            n.put(
                when (component) {
                    ContactComponent.NORMAL -> contacts.normalLambda(i)
                    ContactComponent.T1 -> contacts.t1Lambda(i)
                    ContactComponent.T2 -> contacts.t2Lambda(i)
                }
            ) // lambda
            
            val dva = _c_im.set(bodyAInverseMass).mul(_c_vec3.set(_norm).negate())
            n.putVector3f(dva) // DVA
            n.putVector3f(_c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia)) // DOA
            val dvb = _c_im.set(bodyBInverseMass).mul(_norm)
            n.putVector3f(dvb) // DVB
            n.putVector3f(_c_j3Temp.set(_c_j3)._mul(bodyBInverseInertia)) // DOB
            
            n.put(Float.fromBits(contacts.bodyAIdx(i))) // my id
            n.put(Float.fromBits(contacts.bodyBIdx(i))) // other id
            
            check(n.position() == (i + 1) * A2A_N_CONTACT_DATA_FLOATS)

            i++
        }
    }

    private fun constructFlatEnvConstraintData(component: ContactComponent) {
        val numc = envContacts.size()
        val relevantData = when (component) {
            ContactComponent.NORMAL -> {
                if (envContactNormalData.size < numc * A2S_N_CONTACT_DATA_FLOATS) {
                    envContactNormalData =
                        FloatArray(max(envContactNormalData.size * 2, numc * A2S_N_CONTACT_DATA_FLOATS))
                }
                envContactNormalData
            }

            ContactComponent.T1 -> {
                if (envContactT1Data.size < numc * A2S_N_CONTACT_DATA_FLOATS) {
                    envContactT1Data = FloatArray(max(envContactT1Data.size * 2, numc * A2S_N_CONTACT_DATA_FLOATS))
                }
                envContactT1Data
            }

            ContactComponent.T2 -> {
                if (envContactT2Data.size < numc * A2S_N_CONTACT_DATA_FLOATS) {
                    envContactT2Data = FloatArray(max(envContactT2Data.size * 2, numc * A2S_N_CONTACT_DATA_FLOATS))
                }
                envContactT2Data
            }
        }
        val n = FloatBuffer.wrap(relevantData)
        var i = 0
        while (i < numc) {
            when (component) {
                ContactComponent.NORMAL -> _norm.set(envContacts.normX(i), envContacts.normY(i), envContacts.normZ(i))
                ContactComponent.T1 -> _norm.set(envContacts.t1X(i), envContacts.t1Y(i), envContacts.t1Z(i))
                ContactComponent.T2 -> _norm.set(envContacts.t2X(i), envContacts.t2Y(i), envContacts.t2Z(i))
            }

            n.putVector3f(_norm) // norm

            n.putVector3f(
                _c_j1
                    .set(envContacts.bodyAPosX(i), envContacts.bodyAPosY(i), envContacts.bodyAPosZ(i))
                    .sub(envContacts.pointAX(i), envContacts.pointAY(i), envContacts.pointAZ(i))
                    .cross(_norm)
            ) //j1

            if (component == ContactComponent.NORMAL) {
                n.put(bias / timeStep * max(0f, abs(envContacts.depth(i)) - slop)) // bias
            } else {
                n.put(0f)
            }

            val bodyAInverseMass = envContacts.bodyAInverseMass(i)
            val bodyAInverseInertia = envContacts.bodyAInverseInertia(i, _c_tempMatrix3f1)

            n.put(
                _c_j0.set(
                    _norm.x * _norm.x,
                    _norm.y * _norm.y,
                    _norm.z * _norm.z
                ).dot(
                    bodyAInverseMass,
                    bodyAInverseMass,
                    bodyAInverseMass,
                ) + _c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia).dot(_c_j1)
            ) // den

            n.put(
                when (component) {
                    ContactComponent.NORMAL -> envContacts.normalLambda(i)
                    ContactComponent.T1 -> envContacts.t1Lambda(i)
                    ContactComponent.T2 -> envContacts.t2Lambda(i)
                }
            ) // lambda

            n.putVector3f(_c_im.set(bodyAInverseMass).mul(_c_vec3.set(_norm).negate())) // DVA
            n.putVector3f(_c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia)) // DOA

            n.put(Float.fromBits(envContacts.bodyAIdx(i))) // my id

            i++
        }
    }

    private fun buildFlatBodyData() {
        val n = idMap.size
        if (flatBodyData.size < n * BODY_DATA_FLOATS) { // resizing is fine, since all the valid bodies should set their data anyway
            flatBodyData = FloatArray(max(flatBodyData.size * 2, n * BODY_DATA_FLOATS))
        }

        val buf = FloatBuffer.wrap(flatBodyData)

        var i = 0
        while (i < bodyCount) {
            val b = idMap[i]

            buf.putVector3f(_vec3.set(b.velocity))
            buf.putVector3f(_vec3.set(b.omega).rotate(_quat.set(b.q)))

            i++
        }
    }

    private val _quatd = Quaterniond()

    fun solveNormal() {
        if (!any) return

        var i = 0
        val n = contacts.size() * A2A_N_CONTACT_DATA_FLOATS

        while (i < n) {
            solveA2AContact(contactNormalData, i, ContactComponent.NORMAL)

            i += A2A_N_CONTACT_DATA_FLOATS
        }

        var j = 0
        val e = envContacts.size() * A2S_N_CONTACT_DATA_FLOATS

        while (j < e) {
            solveA2SContact(envContactNormalData, j, ContactComponent.NORMAL)

            j += A2S_N_CONTACT_DATA_FLOATS
        }
    }

    fun solveFriction() {
        if (!any) return

        var i = 0
        val n = contacts.size() * A2A_N_CONTACT_DATA_FLOATS

        while (i < n) {
            solveA2AContact(contactT1Data, i, ContactComponent.T1)
            solveA2AContact(contactT2Data, i, ContactComponent.T2)

            i += A2A_N_CONTACT_DATA_FLOATS
        }

        var j = 0
        val e = envContacts.size() * A2S_N_CONTACT_DATA_FLOATS

        while (j < e) {
            solveA2SContact(envContactT1Data, j, ContactComponent.T1)
            solveA2SContact(envContactT2Data, j, ContactComponent.T2)

            j += A2S_N_CONTACT_DATA_FLOATS
        }
    }

    /**
     * For a contact between active objects
     */
    private fun solveA2AContact(data: FloatArray, contactIdx: Int, component: ContactComponent) {
        if (component != ContactComponent.NORMAL) {
            val n = contactNormalData[contactIdx + A2A_N_LAMBDA_OFFSET]
            if (n < FRICTION_LAMBDA_EPSILON) {
                return
            }
        }

        val nx = data[contactIdx + A2A_N_NORMAL_OFFSET]
        val ny = data[contactIdx + A2A_N_NORMAL_OFFSET + 1]
        val nz = data[contactIdx + A2A_N_NORMAL_OFFSET + 2]

        //set nn, j1, and j3
        val bias = data[contactIdx + A2A_N_BIAS_OFFSET]
        
        val den = data[contactIdx + A2A_N_DEN_OFFSET]

        val myIdx = data[contactIdx + A2A_N_MY_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

        val vAx = flatBodyData[myIdx + V_OFFSET]
        val vAy = flatBodyData[myIdx + V_OFFSET + 1]
        val vAz = flatBodyData[myIdx + V_OFFSET + 2]

        val oAx = flatBodyData[myIdx + O_OFFSET]
        val oAy = flatBodyData[myIdx + O_OFFSET + 1]
        val oAz = flatBodyData[myIdx + O_OFFSET + 2]

        val otherIdx = data[contactIdx + A2A_N_OTHER_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

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
                            data[contactIdx + A2A_N_J1_OFFSET],
                            oAx,
                            Math.fma(
                                data[contactIdx + A2A_N_J1_OFFSET + 1],
                                oAy,
                                Math.fma(
                                    data[contactIdx + A2A_N_J1_OFFSET + 2],
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
                                                    data[contactIdx + A2A_N_J3_OFFSET],
                                                    oBx,
                                                    Math.fma(
                                                        data[contactIdx + A2A_N_J3_OFFSET + 1],
                                                        oBy,
                                                        Math.fma(
                                                            data[contactIdx + A2A_N_J3_OFFSET + 2],
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

        val l = data[contactIdx + A2A_N_LAMBDA_OFFSET]
        if (component == ContactComponent.NORMAL) {
            data[contactIdx + A2A_N_LAMBDA_OFFSET] = max(0f, l + lambda)
        } else {
            val n = contactNormalData[contactIdx + A2A_N_LAMBDA_OFFSET]
            data[contactIdx + A2A_N_LAMBDA_OFFSET] = min(max(-friction * n, l + lambda), friction * n)
        }

        lambda = data[contactIdx + A2A_N_LAMBDA_OFFSET] - l

        if (component == ContactComponent.NORMAL) {
            println("effective lambda: $lambda, den: $den")
            println("| norm: ($nx, $ny, $nz)")
            println("| dva: (${data[contactIdx + A2A_N_DELTA_OFFSET]}, ${data[contactIdx + A2A_N_DELTA_OFFSET + 1]}, ${data[contactIdx + A2A_N_DELTA_OFFSET + 2]})")
            println("| dvb: (${data[contactIdx + A2A_N_DELTA_OFFSET + 6]}, ${data[contactIdx + A2A_N_DELTA_OFFSET + 7]}, ${data[contactIdx + A2A_N_DELTA_OFFSET + 8]})")
        }

        flatBodyData[myIdx] -= (data[contactIdx + A2A_N_DELTA_OFFSET] * lambda)
        flatBodyData[myIdx + 1] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 1] * lambda)
        flatBodyData[myIdx + 2] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 2] * lambda)

        flatBodyData[myIdx + 3] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 3] * lambda)
        flatBodyData[myIdx + 4] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 4] * lambda)
        flatBodyData[myIdx + 5] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 5] * lambda)

        flatBodyData[otherIdx] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 6] * lambda)
        flatBodyData[otherIdx + 1] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 7] * lambda)
        flatBodyData[otherIdx + 2] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 8] * lambda)

        flatBodyData[otherIdx + 3] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 9] * lambda)
        flatBodyData[otherIdx + 4] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 10] * lambda)
        flatBodyData[otherIdx + 5] -= (data[contactIdx + A2A_N_DELTA_OFFSET + 11] * lambda)
    }

    /**
     * For a contact between an active body and a static object
     */
    private fun solveA2SContact(data: FloatArray, contactIdx: Int, component: ContactComponent) {
        val nx = data[contactIdx + A2S_N_NORMAL_OFFSET]
        val ny = data[contactIdx + A2S_N_NORMAL_OFFSET + 1]
        val nz = data[contactIdx + A2S_N_NORMAL_OFFSET + 2]

        val bias = data[contactIdx + A2S_N_BIAS_OFFSET]
        if (component != ContactComponent.NORMAL) {
            check(bias == 0f)
        }

        val den = data[contactIdx + A2S_N_DEN_OFFSET]

        val myIdx = data[contactIdx + A2S_N_MY_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

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
                            data[contactIdx + A2S_N_J1_OFFSET],
                            oAx,
                            Math.fma(
                                data[contactIdx + A2S_N_J1_OFFSET + 1],
                                oAy,
                                Math.fma(
                                    data[contactIdx + A2S_N_J1_OFFSET + 2],
                                    oAz,
                                    bias
                                )
                            )
                        )
                    )
                )
            ) / den

        val l = data[contactIdx + A2S_N_LAMBDA_OFFSET]

        if (component == ContactComponent.NORMAL) {
            data[contactIdx + A2S_N_LAMBDA_OFFSET] = max(0f, l + lambda)
        } else {
            val n = envContactNormalData[contactIdx + A2S_N_LAMBDA_OFFSET]
            data[contactIdx + A2S_N_LAMBDA_OFFSET] = min(max(-friction * n, l + lambda), friction * n)
        }

        lambda = data[contactIdx + A2S_N_LAMBDA_OFFSET] - l

        if (abs(lambda) < FRICTION_LAMBDA_EPSILON && component != ContactComponent.NORMAL) {
            return
        }

        flatBodyData[myIdx] -= (data[contactIdx + A2S_N_DELTA_OFFSET] * lambda)
        flatBodyData[myIdx + 1] -= (data[contactIdx + A2S_N_DELTA_OFFSET + 1] * lambda)
        flatBodyData[myIdx + 2] -= (data[contactIdx + A2S_N_DELTA_OFFSET + 2] * lambda)

        flatBodyData[myIdx + 3] -= (data[contactIdx + A2S_N_DELTA_OFFSET + 3] * lambda)
        flatBodyData[myIdx + 4] -= (data[contactIdx + A2S_N_DELTA_OFFSET + 4] * lambda)
        flatBodyData[myIdx + 5] -= (data[contactIdx + A2S_N_DELTA_OFFSET + 5] * lambda)
    }

    fun write() {
        if (!any) return

        var i = 0
        val n = bodyCount * BODY_DATA_FLOATS
        while (i < n) {
            val body = idMap[i / BODY_DATA_FLOATS]

            body.velocity.from(i + V_OFFSET, flatBodyData)
            println("body.velocity: ${body.velocity}")
            check(body.velocity.isFinite)
            body.omega.from(i + O_OFFSET, flatBodyData).rotate(_quatd.set(body.q).conjugate())
            check(body.omega.isFinite)

            i += BODY_DATA_FLOATS
        }

        var j = 0
        while (j < contacts.size()) {
            val lambda = contactNormalData[j * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET]
            contacts.setNormalLambda(j, lambda)
            contacts.setT1Lambda(j, contactT1Data[j * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET])
            contacts.setT2Lambda(j, contactT2Data[j * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET])

            j++
        }

        var k = 0
        while (k < envContacts.size()) {
            envContacts.setNormalLambda(k, envContactNormalData[k * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET])
            envContacts.setT1Lambda(k, envContactT1Data[k * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET])
            envContacts.setT2Lambda(k, envContactT2Data[k * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET])

            k++
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

enum class ContactComponent {
    NORMAL, T1, T2
}

private const val FRICTION_LAMBDA_EPSILON = 0.0

private inline fun Vector3f._mul(mat: Matrix3d): Vector3f {
    val lx: Float = x
    val ly: Float = y
    val lz: Float = z
    this.x = Math.fma(mat.m00.toFloat(), lx, Math.fma(mat.m10.toFloat(), ly, mat.m20.toFloat() * lz))
    this.y = Math.fma(mat.m01.toFloat(), lx, Math.fma(mat.m11.toFloat(), ly, mat.m21.toFloat() * lz))
    this.z = Math.fma(mat.m02.toFloat(), lx, Math.fma(mat.m12.toFloat(), ly, mat.m22.toFloat() * lz))

    return this
}

private inline fun Vector3f._mul(mat: Matrix3f): Vector3f {
    val lx: Float = x
    val ly: Float = y
    val lz: Float = z
    this.x = Math.fma(mat.m00, lx, Math.fma(mat.m10, ly, mat.m20 * lz))
    this.y = Math.fma(mat.m01, lx, Math.fma(mat.m11, ly, mat.m21 * lz))
    this.z = Math.fma(mat.m02, lx, Math.fma(mat.m12, ly, mat.m22 * lz))

    return this
}
