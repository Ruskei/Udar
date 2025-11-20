package com.ixume.udar.physics.contact

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.BODY_DATA_FLOATS
import com.ixume.udar.physics.constraint.LocalConstraintData
import com.ixume.udar.physics.constraint.O_OFFSET
import com.ixume.udar.physics.constraint.V_OFFSET
import com.ixume.udar.physics.constraint._mul
import com.ixume.udar.physics.constraint.putVector3f
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldBuffer
import com.ixume.udar.physics.contact.a2a.A2APrevContactDataBuffer
import com.ixume.udar.physics.contact.a2s.A2SPrevContactDataBuffer
import org.joml.Matrix3f
import org.joml.Vector3f
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class LocalContactSolver(val constraintSolver: LocalConstraintData) {
    private val physicsWorld = constraintSolver.physicsWorld

    private var timeStep = Udar.CONFIG.timeStep.toFloat()
    private var friction = Udar.CONFIG.collision.friction.toFloat()
    private var bias = Udar.CONFIG.collision.bias.toFloat()
    private var slop = Udar.CONFIG.collision.passiveSlop.toFloat()

    private lateinit var manifolds: A2AManifoldArray
    private lateinit var envManifolds: A2SManifoldBuffer
    private var contactNormalData: FloatArray = FloatArray(1)

    private var contactT1Data: FloatArray = FloatArray(1)
    private var contactT2Data: FloatArray = FloatArray(1)

    private var envContactNormalData: FloatArray = FloatArray(1)

    private var envContactT1Data: FloatArray = FloatArray(1)
    private var envContactT2Data: FloatArray = FloatArray(1)

    private var any = false

    private var itr = 0
    private var t = 0

    private val _c_j1 = Vector3f()
    private val _c_j1Temp = Vector3f()
    private val _c_j3Temp = Vector3f()
    private val _norm = Vector3f()
    private val _c_tempMatrix3f1 = Matrix3f()
    private val _c_tempMatrix3f2 = Matrix3f()

    fun setup() {
        timeStep = Udar.CONFIG.timeStep.toFloat()
        friction = Udar.CONFIG.collision.friction.toFloat()
        bias = Udar.CONFIG.collision.bias.toFloat()
        slop = Udar.CONFIG.collision.passiveSlop.toFloat()

        manifolds = physicsWorld.manifoldBuffer
        envManifolds = physicsWorld.envManifoldBuffer

        any = true
        if (manifolds.isEmpty() && envManifolds.isEmpty()) {
            any = false
            return
        }

        constructFlatConstraintData(Component.AXIS)
        constructFlatEnvConstraintData(Component.AXIS)

        constructFlatConstraintData(Component.T1)
        constructFlatEnvConstraintData(Component.T1)

        constructFlatConstraintData(Component.T2)
        constructFlatEnvConstraintData(Component.T2)

        itr = 0

        warmStart()
    }

    private fun constructFlatConstraintData(component: Component) {
        val numc = manifolds.numContacts
        val relevantData = when (component) {
            Component.AXIS -> {
                if (contactNormalData.size < numc * A2A_N_CONTACT_DATA_FLOATS) {
                    contactNormalData = FloatArray(max(contactNormalData.size * 2, numc * A2A_N_CONTACT_DATA_FLOATS))
                }

                contactNormalData
            }

            Component.T1 -> {
                if (contactT1Data.size < numc * A2A_N_CONTACT_DATA_FLOATS) {
                    contactT1Data = FloatArray(max(contactT1Data.size * 2, numc * A2A_N_CONTACT_DATA_FLOATS))
                }

                contactT1Data
            }

            Component.T2 -> {
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
                val nx: Float
                val ny: Float
                val nz: Float
                when (component) {
                    Component.AXIS -> {
                        nx = manifolds.normX(i, m)
                        ny = manifolds.normY(i, m)
                        nz = manifolds.normZ(i, m)

                        _norm.set(nx, ny, nz)
                    }

                    Component.T1 -> {
                        nx = manifolds.t1X(i, m)
                        ny = manifolds.t1Y(i, m)
                        nz = manifolds.t1Z(i, m)
                        _norm.set(nx, ny, nz)
                    }

                    Component.T2 -> {
                        nx = manifolds.t2X(i, m)
                        ny = manifolds.t2Y(i, m)
                        nz = manifolds.t2Z(i, m)
                        _norm.set(nx, ny, nz)
                    }
                }

                n.putVector3f(_norm)

                val rax = manifolds.bodyAX(i) - manifolds.pointAX(i, m)
                val ray = manifolds.bodyAY(i) - manifolds.pointAY(i, m)
                val raz = manifolds.bodyAZ(i) - manifolds.pointAZ(i, m)

                val rbx = manifolds.pointBX(i, m) - manifolds.bodyBX(i)
                val rby = manifolds.pointBY(i, m) - manifolds.bodyBY(i)
                val rbz = manifolds.pointBZ(i, m) - manifolds.bodyBZ(i)

                val j1x = fma(ray, nz, -raz * ny)
                val j1y = fma(raz, nx, -rax * nz)
                val j1z = fma(rax, ny, -ray * nx)

                val j3x = fma(rby, nz, -rbz * ny)
                val j3y = fma(rbz, nx, -rbx * nz)
                val j3z = fma(rbx, ny, -rby * nx)

                n.put(j1x)
                n.put(j1y)
                n.put(j1z)

//                n.putVector3f(
//                    _c_j1
//                        .set(manifolds.bodyAX(i), manifolds.bodyAY(i), manifolds.bodyAZ(i))
//                        .sub(manifolds.pointAX(i, m), manifolds.pointAY(i, m), manifolds.pointAZ(i, m))
//                        .cross(_norm)
//                ) //j1

                n.put(j3x)
                n.put(j3y)
                n.put(j3z)

//                n.putVector3f(
//                    _c_j3
//                        .set(manifolds.pointBX(i, m), manifolds.pointBY(i, m), manifolds.pointBZ(i, m))
//                        .sub(manifolds.bodyBX(i), manifolds.bodyBY(i), manifolds.bodyBZ(i))
//                        .cross(_norm)
//                ) //j3

                val depth = manifolds.depth(i, m)

                if (component == Component.AXIS) {
                    val tb = bias / timeStep * max(0f, abs(depth) - slop)
                    n.put(tb) // bias
                } else {
                    n.put(0f)
                }

                val ima = manifolds.bodyAIM(i)
                val imb = manifolds.bodyBIM(i)
                val iia = manifolds.bodyAII(i, _c_tempMatrix3f1)
                val iib = manifolds.bodyBII(i, _c_tempMatrix3f2)

                val den =
                    ima +
                    iia.transform(j1x, j1y, j1z, _c_j1Temp).dot(j1x, j1y, j1z) +
                    imb +
                    iib.transform(j3x, j3y, j3z, _c_j3Temp).dot(j3x, j3y, j3z)

                n.put(den) // den

                n.put(
                    when (component) {
                        Component.AXIS -> manifolds.normalLambda(i, m)
                        Component.T1 -> manifolds.t1Lambda(i, m)
                        Component.T2 -> manifolds.t2Lambda(i, m)
                    }
                ) // lambda

                n.put(-_norm.x * ima)
                n.put(-_norm.y * ima)
                n.put(-_norm.z * ima)
                iia.transform(j1x, j1y, j1z, _c_j1Temp)
                n.put(_c_j1Temp.x)
                n.put(_c_j1Temp.y)
                n.put(_c_j1Temp.z)
                n.put(_norm.x * imb)
                n.put(_norm.y * imb)
                n.put(_norm.z * imb)
                iib.transform(j3x, j3y, j3z, _c_j3Temp)
                n.put(_c_j3Temp.x)
                n.put(_c_j3Temp.y)
                n.put(_c_j3Temp.z)

                n.put(Float.fromBits(manifolds.bodyAIdx(i))) // my id
                n.put(Float.fromBits(manifolds.bodyBIdx(i))) // other id

                m++
            }

            i++
        }
    }

    private fun constructFlatEnvConstraintData(component: Component) {
        val numc = envManifolds.numContacts.get()
        val relevantData = when (component) {
            Component.AXIS -> {
                if (envContactNormalData.size < numc * A2S_N_CONTACT_DATA_FLOATS) {
                    envContactNormalData =
                        FloatArray(max(envContactNormalData.size * 2, numc * A2S_N_CONTACT_DATA_FLOATS))
                }
                envContactNormalData
            }

            Component.T1 -> {
                if (envContactT1Data.size < numc * A2S_N_CONTACT_DATA_FLOATS) {
                    envContactT1Data = FloatArray(max(envContactT1Data.size * 2, numc * A2S_N_CONTACT_DATA_FLOATS))
                }
                envContactT1Data
            }

            Component.T2 -> {
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
                    Component.AXIS -> _norm.set(
                        envManifolds.normX(ns, m),
                        envManifolds.normY(ns, m),
                        envManifolds.normZ(ns, m)
                    )

                    Component.T1 -> _norm.set(
                        envManifolds.t1X(ns, m),
                        envManifolds.t1Y(ns, m),
                        envManifolds.t1Z(ns, m)
                    )

                    Component.T2 -> _norm.set(
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

                if (component == Component.AXIS) {
                    n.put(bias / timeStep * max(0f, abs(envManifolds.depth(ns, m)) - slop)) // bias
                } else {
                    n.put(0f)
                }

                val ima = envManifolds.bodyIM(ns)
                val iia = envManifolds.bodyII(ns, _c_tempMatrix3f1)

                val den = ima + _c_j1Temp.set(_c_j1)._mul(iia).dot(_c_j1)
                n.put(den)

                n.put(
                    when (component) {
                        Component.AXIS -> envManifolds.normalLambda(ns, m)
                        Component.T1 -> envManifolds.t1Lambda(ns, m)
                        Component.T2 -> envManifolds.t2Lambda(ns, m)
                    }
                ) // lambda

                n.put(-_norm.x * ima)
                n.put(-_norm.y * ima)
                n.put(-_norm.z * ima)
                n.putVector3f(_c_j1Temp.set(_c_j1)._mul(iia)) // DOA

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



//    private var maxLambda = -Float.MAX_VALUE
//    private var runningMaxLambda = FloatArray(0)

    private fun warmStart() {
        if (!any) return

        val n = manifolds.numContacts * A2A_N_CONTACT_DATA_FLOATS
        val e = envManifolds.numContacts.get() * A2S_N_CONTACT_DATA_FLOATS

        var i = 0
        while (i < n) {
            warmA2AContact(contactNormalData, i, Component.AXIS)

            i += A2A_N_CONTACT_DATA_FLOATS
        }

        var j = 0
        while (j < e) {
            warmA2SContact(envContactNormalData, j, Component.AXIS)

            j += A2S_N_CONTACT_DATA_FLOATS
        }

        var k = 0
        while (k < n) {
            warmA2AContact(contactT1Data, k, Component.T1)
            warmA2AContact(contactT2Data, k, Component.T2)

            k += A2A_N_CONTACT_DATA_FLOATS
        }

        var l = 0
        while (l < e) {
            warmA2SContact(envContactT1Data, l, Component.T1)
            warmA2SContact(envContactT2Data, l, Component.T2)

            l += A2S_N_CONTACT_DATA_FLOATS
        }
    }

    private fun warmA2AContact(data: FloatArray, contactIdx: Int, component: Component) {
        val flatBodyData = constraintSolver.flatBodyData
        val lambda = data[contactIdx + A2A_N_LAMBDA_OFFSET]
//        if (component == ContactComponent.NORMAL && lambda != 0f) {
//            println("warmed contact with $lambda!") CORRECT!
//        }

        if (component != Component.AXIS) {
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

    private fun warmA2SContact(data: FloatArray, contactIdx: Int, component: Component) {
        val flatBodyData = constraintSolver.flatBodyData
        val lambda = data[contactIdx + A2S_N_LAMBDA_OFFSET]
        if (lambda == 0f) {
            return
        }

        val myIdx = data[contactIdx + A2S_N_MY_IDX_OFFSET].toRawBits() * BODY_DATA_FLOATS

        if (abs(lambda) < FRICTION_LAMBDA_EPSILON && component != Component.AXIS) {
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

    private fun heatUp() {
        var count = 0
        val numManifolds = manifolds.size()

        var j = 0
        while (j < numManifolds) {
            _a2aContactDataBuffer.clear()
            val numContacts = manifolds.numContacts(j)
            val manifoldID = manifolds.manifoldID(j)
            val bodyA = physicsWorld.activeBodies.fastGet(manifolds.bodyAIdx(j))!!
            val bodyB = physicsWorld.activeBodies.fastGet(manifolds.bodyBIdx(j))!!

            if (physicsWorld.prevContactMap.containsKey(manifoldID)) {
                println("A2A ID COLLISION! $manifoldID")
                j++
                continue
            }

            var l = 0
            while (l < numContacts) {
                val nl = contactNormalData[count * A2A_N_CONTACT_DATA_FLOATS + A2A_N_LAMBDA_OFFSET]
                val ax = manifolds.pointAX(j, l)
                val ay = manifolds.pointAY(j, l)
                val az = manifolds.pointAZ(j, l)
                bodyA.hookManager.onCollision(ax.toDouble(), ay.toDouble(), az.toDouble(), nl.toDouble())
                val bx = manifolds.pointBX(j, l)
                val by = manifolds.pointBY(j, l)
                val bz = manifolds.pointBZ(j, l)
                bodyB.hookManager.onCollision(bx.toDouble(), by.toDouble(), bz.toDouble(), nl.toDouble())
                _a2aContactDataBuffer.add(
                    ax = ax,
                    ay = ay,
                    az = az,

                    bx = bx,
                    by = by,
                    bz = bz,

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
            val body = physicsWorld.activeBodies.fastGet(envManifolds.bodyIdx(k))!!
            if (physicsWorld.prevEnvContactMap.containsKey(manifoldID)) {
                println("A2S ID COLLISION: $manifoldID!")

                k++
                continue
            }

            var l = 0
            while (l < numContacts) {
                val nl = envContactNormalData[envCount * A2S_N_CONTACT_DATA_FLOATS + A2S_N_LAMBDA_OFFSET]
                val x = envManifolds.pointAX(k, l)
                val y = envManifolds.pointAY(k, l)
                val z = envManifolds.pointAZ(k, l)
                body.hookManager.onCollision(x.toDouble(), y.toDouble(), z.toDouble(), nl.toDouble())
                _a2sContactDataBuffer.add(
                    x = x,
                    y = y,
                    z = z,

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

    fun solveNormal() {
        itr++
        t++

        if (!any) return

//        maxLambda = -Float.MAX_VALUE
        val n = manifolds.numContacts * A2A_N_CONTACT_DATA_FLOATS

        var i = 0
        while (i < n) {
            this.solveA2AContact(contactNormalData, i, Component.AXIS)

            i += A2A_N_CONTACT_DATA_FLOATS
        }


        val e = envManifolds.numContacts.get() * A2S_N_CONTACT_DATA_FLOATS

        var j = 0
        while (j < e) {
            solveA2SContact(envContactNormalData, j, Component.AXIS)

            j += A2S_N_CONTACT_DATA_FLOATS
        }
    }
    
    fun solveFriction() {
        if (!any) return

        var j = 0
        val e = envManifolds.numContacts.get() * A2S_N_CONTACT_DATA_FLOATS

        while (j < e) {
            solveA2SContact(envContactT1Data, j, Component.T1)
            solveA2SContact(envContactT2Data, j, Component.T2)

            j += A2S_N_CONTACT_DATA_FLOATS
        }

        var i = 0
        val n = manifolds.numContacts * A2A_N_CONTACT_DATA_FLOATS

        while (i < n) {
            solveA2AContact(contactT1Data, i, Component.T1)
            solveA2AContact(contactT2Data, i, Component.T2)

            i += A2A_N_CONTACT_DATA_FLOATS
        }
    }

    /**
     * For a contact between active objects
     */
    private fun solveA2AContact(data: FloatArray, contactIdx: Int, component: Component) {
        val flatBodyData = constraintSolver.flatBodyData
        if (component != Component.AXIS) {
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
            fma(
                -nx,
                vAx,
                fma(
                    -ny,
                    vAy,
                    fma(
                        -nz,
                        vAz,
                        fma(
                            data[contactIdx + A2A_N_J1_OFFSET],
                            oAx,
                            fma(
                                data[contactIdx + A2A_N_J1_OFFSET + 1],
                                oAy,
                                fma(
                                    data[contactIdx + A2A_N_J1_OFFSET + 2],
                                    oAz,
                                    fma(
                                        nx,
                                        vBx,
                                        fma(
                                            ny,
                                            vBy,
                                            fma(
                                                nz,
                                                vBz,
                                                fma(
                                                    data[contactIdx + A2A_N_J3_OFFSET],
                                                    oBx,
                                                    fma(
                                                        data[contactIdx + A2A_N_J3_OFFSET + 1],
                                                        oBy,
                                                        fma(
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

        if (component == Component.AXIS) {
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


    /**
     * For a contact between an active body and a static object
     */
    private fun solveA2SContact(data: FloatArray, contactIdx: Int, component: Component) {
        val flatBodyData = constraintSolver.flatBodyData
        val nx = data[contactIdx + A2S_N_NORMAL_OFFSET]
        val ny = data[contactIdx + A2S_N_NORMAL_OFFSET + 1]
        val nz = data[contactIdx + A2S_N_NORMAL_OFFSET + 2]

        val bias = data[contactIdx + A2S_N_BIAS_OFFSET]
        if (component != Component.AXIS) {
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
            fma(
                -nx,
                vAx,
                fma(
                    -ny,
                    vAy,
                    fma(
                        -nz,
                        vAz,
                        fma(
                            data[contactIdx + A2S_N_J1_OFFSET],
                            oAx,
                            fma(
                                data[contactIdx + A2S_N_J1_OFFSET + 1],
                                oAy,
                                fma(
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

        if (component == Component.AXIS) {
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
        if (abs(lambda) < FRICTION_LAMBDA_EPSILON && component != Component.AXIS) {
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
        heatUp()
    }
}

enum class Component {
    AXIS, T1, T2
}

private const val FRICTION_LAMBDA_EPSILON = 0.0

const val A2A_N_CONTACT_DATA_FLOATS = 26

const val A2A_N_NORMAL_OFFSET = 0
const val A2A_N_J1_OFFSET = 3
const val A2A_N_J3_OFFSET = 6
const val A2A_N_BIAS_OFFSET = 9
const val A2A_N_DEN_OFFSET = 10
const val A2A_N_LAMBDA_OFFSET = 11
const val A2A_N_DELTA_OFFSET = 12
const val A2A_N_MY_IDX_OFFSET = 24
const val A2A_N_OTHER_IDX_OFFSET = 25

const val A2S_N_CONTACT_DATA_FLOATS = 18

const val A2S_N_NORMAL_OFFSET = 0
const val A2S_N_J1_OFFSET = 3
const val A2S_N_BIAS_OFFSET = 6
const val A2S_N_DEN_OFFSET = 7
const val A2S_N_LAMBDA_OFFSET = 8
const val A2S_N_DELTA_OFFSET = 9
const val A2S_N_MY_IDX_OFFSET = 15
const val A2S_MANIFOLD_ID_OFFSET = 16
