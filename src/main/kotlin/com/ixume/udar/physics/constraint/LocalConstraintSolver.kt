package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.contact.A2AManifoldBuffer
import com.ixume.udar.physics.contact.A2APrevContactDataBuffer
import com.ixume.udar.physics.contact.A2SManifoldBuffer
import com.ixume.udar.physics.contact.A2SPrevContactDataBuffer
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
    private var timeStep = Udar.CONFIG.timeStep.toFloat()
    private var friction = Udar.CONFIG.collision.friction.toFloat()
    private var bias = Udar.CONFIG.collision.bias.toFloat()
    private var slop = Udar.CONFIG.collision.passiveSlop.toFloat()

    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    private var idMap: Array<ActiveBody>
    private var bodyCount: Int

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

    private lateinit var manifolds: A2AManifoldBuffer
    private lateinit var envManifolds: A2SManifoldBuffer
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

        timeStep = Udar.CONFIG.timeStep.toFloat()
        friction = Udar.CONFIG.collision.friction.toFloat()
        bias = Udar.CONFIG.collision.bias.toFloat()
        slop = Udar.CONFIG.collision.passiveSlop.toFloat()

        manifolds = physicsWorld.manifoldBuffer
        envManifolds = physicsWorld.envManifoldBuffer

        if (manifolds.isEmpty() && envManifolds.isEmpty()) {
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

//        runningMaxLambda = FloatArray(Udar.CONFIG.collision.normalIterations)
        itr = 0
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
        val numc = manifolds.numContacts.get()
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
        while (i < manifolds.size()) {
            val num = manifolds.numContacts(i)
            var m = 0
            while (m < num) {
                when (component) {
                    ContactComponent.NORMAL -> _norm.set(
                        manifolds.normX(i, m),
                        manifolds.normY(i, m),
                        manifolds.normZ(i, m)
                    )

                    ContactComponent.T1 -> _norm.set(manifolds.t1X(i, m), manifolds.t1Y(i, m), manifolds.t1Z(i, m))
                    ContactComponent.T2 -> _norm.set(manifolds.t2X(i, m), manifolds.t2Y(i, m), manifolds.t2Z(i, m))
                }

                n.putVector3f(_norm)

                n.putVector3f(
                    _c_j1
                        .set(manifolds.bodyAX(i), manifolds.bodyAY(i), manifolds.bodyAZ(i))
                        .sub(manifolds.pointAX(i, m), manifolds.pointAY(i, m), manifolds.pointAZ(i, m))
                        .cross(_norm)
                ) //j1

                n.putVector3f(
                    _c_j3
                        .set(manifolds.pointBX(i, m), manifolds.pointBY(i, m), manifolds.pointBZ(i, m))
                        .sub(manifolds.bodyBX(i), manifolds.bodyBY(i), manifolds.bodyBZ(i))
                        .cross(_norm)
                ) //j3

                val depth = manifolds.depth(i, m)

                if (component == ContactComponent.NORMAL) {
                    val tb = bias / timeStep * max(0f, abs(depth) - slop)
                    n.put(tb) // bias
                } else {
                    n.put(0f)
                }

                val bodyAInverseMass = manifolds.bodyAIM(i)
                val bodyBInverseMass = manifolds.bodyBIM(i)
                val bodyAInverseInertia = manifolds.bodyAII(i, _c_tempMatrix3f1)
                val bodyBInverseInertia = manifolds.bodyBII(i, _c_tempMatrix3f2)

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
                        ContactComponent.NORMAL -> manifolds.normalLambda(i, m)
                        ContactComponent.T1 -> manifolds.t1Lambda(i, m)
                        ContactComponent.T2 -> manifolds.t2Lambda(i, m)
                    }
                ) // lambda

                val dva = _c_im.set(bodyAInverseMass).mul(_c_vec3.set(_norm).negate())
                n.putVector3f(dva) // DVA
                n.putVector3f(_c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia)) // DOA
                val dvb = _c_im.set(bodyBInverseMass).mul(_norm)
                n.putVector3f(dvb) // DVB
                n.putVector3f(_c_j3Temp.set(_c_j3)._mul(bodyBInverseInertia)) // DOB

                n.put(Float.fromBits(manifolds.bodyAIdx(i))) // my id
                n.put(Float.fromBits(manifolds.bodyBIdx(i))) // other id

                m++
            }

            i++
        }
    }

    private fun constructFlatEnvConstraintData(component: ContactComponent) {
        val numc = envManifolds.numContacts.get()
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

        var count = 0
        var ns = 0
        while (ns < envManifolds.size()) {
            val num = envManifolds.numContacts(ns)
            var m = 0
            while (m < num) {
                when (component) {
                    ContactComponent.NORMAL -> _norm.set(
                        envManifolds.normX(ns, m),
                        envManifolds.normY(ns, m),
                        envManifolds.normZ(ns, m)
                    )

                    ContactComponent.T1 -> _norm.set(
                        envManifolds.t1X(ns, m),
                        envManifolds.t1Y(ns, m),
                        envManifolds.t1Z(ns, m)
                    )

                    ContactComponent.T2 -> _norm.set(
                        envManifolds.t2X(ns, m),
                        envManifolds.t2Y(ns, m),
                        envManifolds.t2Z(ns, m)
                    )
                }

                n.putVector3f(_norm) // norm

                val j1 = _c_j1
                    .set(envManifolds.bodyX(ns), envManifolds.bodyY(ns), envManifolds.bodyZ(ns))
                    .sub(envManifolds.pointAX(ns, m), envManifolds.pointAY(ns, m), envManifolds.pointAZ(ns, m))
                    .cross(_norm)
                n.putVector3f(j1)

                if (component == ContactComponent.NORMAL) {
                    n.put(bias / timeStep * max(0f, abs(envManifolds.depth(ns, m)) - slop)) // bias
                } else {
                    n.put(0f)
                }

                val bodyAInverseMass = envManifolds.bodyIM(ns)
                val bodyAInverseInertia = envManifolds.bodyII(ns, _c_tempMatrix3f1)

                val den = _c_j0.set(
                    _norm.x * _norm.x,
                    _norm.y * _norm.y,
                    _norm.z * _norm.z
                ).dot(
                    bodyAInverseMass,
                    bodyAInverseMass,
                    bodyAInverseMass,
                ) + _c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia).dot(_c_j1)
                n.put(den)

                n.put(
                    when (component) {
                        ContactComponent.NORMAL -> envManifolds.normalLambda(ns, m)
                        ContactComponent.T1 -> envManifolds.t1Lambda(ns, m)
                        ContactComponent.T2 -> envManifolds.t2Lambda(ns, m)
                    }
                ) // lambda

                n.putVector3f(_c_im.set(bodyAInverseMass).mul(_c_vec3.set(_norm).negate())) // DVA
                n.putVector3f(_c_j1Temp.set(_c_j1)._mul(bodyAInverseInertia)) // DOA

                n.put(Float.fromBits(envManifolds.bodyIdx(ns))) // my id
                val cID = envManifolds.manifoldID(ns)
                n.put(cID.toFloat())
                n.put((cID ushr 32).toFloat())

                count++
                m++
            }

            ns++
        }

        check(numc == count)
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

//    private var maxLambda = -Float.MAX_VALUE
//    private var runningMaxLambda = FloatArray(0)

    fun warmStart() {
        if (!any) return

        val n = manifolds.numContacts.get() * A2A_N_CONTACT_DATA_FLOATS
        val e = envManifolds.numContacts.get() * A2S_N_CONTACT_DATA_FLOATS

        var i = 0
        while (i < n) {
            warmA2AContact(contactNormalData, i, ContactComponent.NORMAL)

            i += A2A_N_CONTACT_DATA_FLOATS
        }

        var j = 0
        while (j < e) {
            warmA2SContact(envContactNormalData, j, ContactComponent.NORMAL)

            j += A2S_N_CONTACT_DATA_FLOATS
        }

        var k = 0
        while (k < n) {
            warmA2AContact(contactT1Data, k, ContactComponent.T1)
            warmA2AContact(contactT2Data, k, ContactComponent.T2)

            k += A2A_N_CONTACT_DATA_FLOATS
        }

        var l = 0
        while (l < e) {
            warmA2SContact(envContactT1Data, l, ContactComponent.T1)
            warmA2SContact(envContactT2Data, l, ContactComponent.T2)

            l += A2S_N_CONTACT_DATA_FLOATS
        }
    }

    private var itr = 0
    private var t = 0
    fun solveNormal() {
        itr++
        t++

        if (!any) return

//        maxLambda = -Float.MAX_VALUE
        val n = manifolds.numContacts.get() * A2A_N_CONTACT_DATA_FLOATS

        var i = 0
        while (i < n) {
            solveA2AContact(contactNormalData, i, ContactComponent.NORMAL)

            i += A2A_N_CONTACT_DATA_FLOATS
        }

//        runningMaxLambda[itr - 1] += abs(maxLambda)
//
//        if (t % (DATA_OUTPUT_INTERVAL * Udar.CONFIG.collision.normalIterations) == 0) {
//            println("AVERAGE LAMBDAS")
//            var p = 0
//            while (p < runningMaxLambda.size) {
//                println("$p: ${runningMaxLambda[p]}")
//                runningMaxLambda[p] = 0f
//                p++
//            }
//        }

        val e = envManifolds.numContacts.get() * A2S_N_CONTACT_DATA_FLOATS

        var j = 0
        while (j < e) {
            solveA2SContact(envContactNormalData, j, ContactComponent.NORMAL)

            j += A2S_N_CONTACT_DATA_FLOATS
        }
    }

    fun solveFriction() {
        if (!any) return

        var j = 0
        val e = envManifolds.numContacts.get() * A2S_N_CONTACT_DATA_FLOATS

        while (j < e) {
            solveA2SContact(envContactT1Data, j, ContactComponent.T1)
            solveA2SContact(envContactT2Data, j, ContactComponent.T2)

            j += A2S_N_CONTACT_DATA_FLOATS
        }

        var i = 0
        val n = manifolds.numContacts.get() * A2A_N_CONTACT_DATA_FLOATS

        while (i < n) {
            solveA2AContact(contactT1Data, i, ContactComponent.T1)
            solveA2AContact(contactT2Data, i, ContactComponent.T2)

            i += A2A_N_CONTACT_DATA_FLOATS
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

//        if (maxLambda == -Float.MAX_VALUE || abs(lambda) > abs(maxLambda)) {
//            maxLambda = lambda
//        }
//
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

    private fun warmA2AContact(data: FloatArray, contactIdx: Int, component: ContactComponent) {
        val lambda = data[contactIdx + A2A_N_LAMBDA_OFFSET]
        if (component == ContactComponent.NORMAL && lambda != 0f) {
//            println("warmed contact with $lambda!") CORRECT!
        }

        if (component != ContactComponent.NORMAL) {
            val n = contactNormalData[contactIdx + A2A_N_LAMBDA_OFFSET]
            if (n < FRICTION_LAMBDA_EPSILON) {
                return
            }
        }

        val myIdx = data[contactIdx + A2A_N_MY_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS
        val otherIdx = data[contactIdx + A2A_N_OTHER_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

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

//        if (maxLambda == -Float.MAX_VALUE || abs(lambda) > abs(maxLambda)) {
//            maxLambda = lambda
//        }
//
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

    private fun warmA2SContact(data: FloatArray, contactIdx: Int, component: ContactComponent) {
        val lambda = data[contactIdx + A2S_N_LAMBDA_OFFSET]
        if (lambda == 0f) {
            return
        }

        val myIdx = data[contactIdx + A2S_N_MY_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

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

    private val _a2aContactDataBuffer = A2APrevContactDataBuffer()
    private val _a2sContactDataBuffer = A2SPrevContactDataBuffer()

    fun write() {
        physicsWorld.prevContactMap.clear()
        physicsWorld.prevContactData.clear()

        physicsWorld.prevEnvContactMap.clear()
        physicsWorld.prevEnvContactData.clear()

        if (!any) return

        var i = 0
        val n = bodyCount * BODY_DATA_FLOATS
        while (i < n) {
            val body = idMap[i / BODY_DATA_FLOATS]

            body.velocity.from(i + V_OFFSET, flatBodyData)
            check(body.velocity.isFinite)
            body.omega.from(i + O_OFFSET, flatBodyData).rotate(_quatd.set(body.q).conjugate())
            check(body.omega.isFinite)

            i += BODY_DATA_FLOATS
        }

        heatUp()
    }

    private fun heatUp() {
        var count = 0
        val numManifolds = manifolds.size()

        var j = 0
        while (j < numManifolds) {
            _a2aContactDataBuffer.clear()
            val numContacts = manifolds.numContacts(j)
            val manifoldID = manifolds.manifoldID(j)

            if (physicsWorld.prevContactMap.containsKey(manifoldID)) {
                j++
                continue
            }

            var l = 0
            while (l < numContacts) {
                val nl = contactNormalData[count * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET]
                _a2aContactDataBuffer.add(
                    ax = manifolds.pointAX(j, l),
                    ay = manifolds.pointAY(j, l),
                    az = manifolds.pointAZ(j, l),

                    bx = manifolds.pointBX(j, l),
                    by = manifolds.pointBY(j, l),
                    bz = manifolds.pointBZ(j, l),

                    normalLambda = nl * Udar.CONFIG.collision.lambdaCarryover,
                    t1Lambda = contactT1Data[count * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET] * Udar.CONFIG.collision.lambdaCarryover,
                    t2Lambda = contactT2Data[count * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET] * Udar.CONFIG.collision.lambdaCarryover,
                )

                count++
                l++
            }

            val idx = physicsWorld.prevContactData.add(numContacts, _a2aContactDataBuffer)
            physicsWorld.prevContactMap.put(manifoldID, idx)

            j++
        }

        var envCount = 0
        val numEnvManifolds = envManifolds.size()

        var k = 0
        while (k < numEnvManifolds) {
            _a2sContactDataBuffer.clear()
            val numContacts = envManifolds.numContacts(k)
            val manifoldID = envManifolds.manifoldID(k)
            if (physicsWorld.prevEnvContactMap.containsKey(manifoldID)) {
                k++
                continue
            }

            var l = 0
            while (l < numContacts) {
                val nl = envContactNormalData[envCount * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET]
                _a2sContactDataBuffer.add(
                    x = envManifolds.pointAX(k, l),
                    y = envManifolds.pointAY(k, l),
                    z = envManifolds.pointAZ(k, l),

                    normalLambda = nl * Udar.CONFIG.collision.lambdaCarryover,
                    t1Lambda = envContactT1Data[envCount * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET] * Udar.CONFIG.collision.lambdaCarryover,
                    t2Lambda = envContactT2Data[envCount * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET] * Udar.CONFIG.collision.lambdaCarryover,
                )

                envCount++
                l++
            }

            val idx = physicsWorld.prevEnvContactData.add(numContacts, _a2sContactDataBuffer)
            physicsWorld.prevEnvContactMap.put(manifoldID, idx)

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


private const val DATA_OUTPUT_INTERVAL = 250