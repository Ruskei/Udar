package com.ixume.udar.physics.contact.v2

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.ConstraintSolver
import java.lang.Math.fma
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import kotlin.math.*

class ContactSolver(val parent: ConstraintSolver) {
    private val world = parent.physicsWorld

    private var dt = Udar.CONFIG.timeStep.toFloat()
    private var friction = Udar.CONFIG.collision.friction.toFloat()
    private var bias = Udar.CONFIG.collision.bias.toFloat()
    private var slop = Udar.CONFIG.collision.passiveSlop.toFloat()
    private var carryover = Udar.CONFIG.collision.lambdaCarryover
    private var relaxation = Udar.CONFIG.collision.relaxation

    private var a2aNumManifolds = 0
    private var a2aNormalManifoldData = A2AManifoldData(0)
    private var a2aT1ManifoldData = A2AManifoldData(0)
    private var a2aT2ManifoldData = A2AManifoldData(0)

    private var a2sNumManifolds = 0
    private var a2sNormalManifoldData = A2SManifoldData(0)
    private var a2sT1ManifoldData = A2SManifoldData(0)
    private var a2sT2ManifoldData = A2SManifoldData(0)

    private val setupExecutor = Executors.newFixedThreadPool(3)

    private var normalDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)

    fun setup() {
        dt = Udar.CONFIG.timeStep.toFloat()
        friction = Udar.CONFIG.collision.friction.toFloat()
        bias = Udar.CONFIG.collision.bias.toFloat()
        slop = Udar.CONFIG.collision.passiveSlop.toFloat()
        carryover = Udar.CONFIG.collision.lambdaCarryover
        relaxation = Udar.CONFIG.collision.relaxation

        val a2aManifolds = world.manifoldBuffer
        val a2sManifolds = world.envManifoldBuffer
        a2aNumManifolds = a2aManifolds.size()
        a2sNumManifolds = a2sManifolds.size()

        if (normalDeltaLambdas.size != Udar.CONFIG.collision.normalIterations) {
            normalDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)
        } else {
            normalDeltaLambdas.fill(0f)
        }

        if (a2aNormalManifoldData.sizeFor(
                a2aNumManifolds,
                a2aManifolds.numContacts
            ) != a2aNormalManifoldData.value.size
        ) {
            val s = a2aNormalManifoldData.sizeFor(a2aNumManifolds, a2aManifolds.numContacts)
            a2aNormalManifoldData = A2AManifoldData(s)
            a2aT1ManifoldData = A2AManifoldData(s)
            a2aT2ManifoldData = A2AManifoldData(s)
        }

        if (a2sNormalManifoldData.sizeFor(
                a2sNumManifolds,
                a2sManifolds.numContacts.get()
            ) != a2sNormalManifoldData.value.size
        ) {
            val s = a2sNormalManifoldData.sizeFor(a2sNumManifolds, a2sManifolds.numContacts.get())
            a2sNormalManifoldData = A2SManifoldData(s)
            a2sT1ManifoldData = A2SManifoldData(s)
            a2sT2ManifoldData = A2SManifoldData(s)
        }

        val latch = CountDownLatch(3)

        setupExecutor.execute {
            var a2aCursor = 0

            for (manifoldIdx in 0..<a2aManifolds.size()) {
                val b1Idx = a2aManifolds.bodyAIdx(manifoldIdx)
                val b2Idx = a2aManifolds.bodyBIdx(manifoldIdx)
                val b1 = world.activeBodies.fastGet(b1Idx)!!
                val b2 = world.activeBodies.fastGet(b2Idx)!!

                val im1 = b1.inverseMass.toFloat()
                val im2 = b2.inverseMass.toFloat()

                val numContacts = a2aManifolds.numContacts(manifoldIdx)

                a2aCursor = a2aNormalManifoldData.add(
                    cursor = a2aCursor,
                    body1Idx = b1Idx,
                    body2Idx = b2Idx,

                    im1 = im1,
                    im2 = im2,

                    numConstraints = numContacts,
                ) { baseIdx, constraintIdx ->
                    val nx = a2aManifolds.normX(manifoldIdx, constraintIdx)
                    val ny = a2aManifolds.normY(manifoldIdx, constraintIdx)
                    val nz = a2aManifolds.normZ(manifoldIdx, constraintIdx)

                    val depth = a2aManifolds.depth(manifoldIdx, constraintIdx)
                    val bias = -bias * max(0f, depth - slop) / dt
                    val lambda = a2aManifolds.normalLambda(manifoldIdx, constraintIdx)

                    a2aNormalManifoldData.addData(
                        baseIdx = baseIdx,
                        manifoldIdx = manifoldIdx,
                        constraintIdx = constraintIdx,
                        b1 = b1,
                        b2 = b2,

                        nx = nx,
                        ny = ny,
                        nz = nz,

                        bias = bias,
                        lambda = lambda,
                        manifolds = a2aManifolds
                    )
                }
            }

            var a2sCursor = 0

            for (manifoldIdx in 0..<a2sManifolds.size()) {
                val b1Idx = a2sManifolds.bodyIdx(manifoldIdx)
                val b1 = world.activeBodies.fastGet(b1Idx)!!

                val im1 = b1.inverseMass.toFloat()

                val numContacts = a2sManifolds.numContacts(manifoldIdx)

                a2sCursor = a2sNormalManifoldData.add(
                    cursor = a2sCursor,
                    body1Idx = b1Idx,

                    im1 = im1,

                    numConstraints = numContacts,
                ) { baseIdx, constraintIdx ->
                    val nx = a2sManifolds.normX(manifoldIdx, constraintIdx)
                    val ny = a2sManifolds.normY(manifoldIdx, constraintIdx)
                    val nz = a2sManifolds.normZ(manifoldIdx, constraintIdx)

                    val depth = a2sManifolds.depth(manifoldIdx, constraintIdx)
                    val bias = -bias * max(0f, depth - slop) / dt
                    val lambda = a2sManifolds.normalLambda(manifoldIdx, constraintIdx)

                    a2sNormalManifoldData.addData(
                        baseIdx = baseIdx,
                        manifoldIdx = manifoldIdx,
                        constraintIdx = constraintIdx,
                        b1 = b1,

                        nx = nx,
                        ny = ny,
                        nz = nz,

                        bias = bias,
                        lambda = lambda,
                        manifolds = a2sManifolds
                    )
                }
            }

            latch.countDown()
        }

        setupExecutor.execute {
            var a2aCursor = 0

            for (manifoldIdx in 0..<a2aManifolds.size()) {
                val b1Idx = a2aManifolds.bodyAIdx(manifoldIdx)
                val b2Idx = a2aManifolds.bodyBIdx(manifoldIdx)
                val b1 = world.activeBodies.fastGet(b1Idx)!!
                val b2 = world.activeBodies.fastGet(b2Idx)!!

                val im1 = b1.inverseMass.toFloat()
                val im2 = b2.inverseMass.toFloat()

                val numContacts = a2aManifolds.numContacts(manifoldIdx)

                a2aCursor = a2aT1ManifoldData.add(
                    cursor = a2aCursor,
                    body1Idx = b1Idx,
                    body2Idx = b2Idx,

                    im1 = im1,
                    im2 = im2,

                    numConstraints = numContacts,
                ) { baseIdx, constraintIdx ->
                    val nx = a2aManifolds.normX(manifoldIdx, constraintIdx)
                    val ny = a2aManifolds.normY(manifoldIdx, constraintIdx)
                    val nz = a2aManifolds.normZ(manifoldIdx, constraintIdx)

                    val absNx = abs(nx)
                    val absNy = abs(ny)
                    val absNz = abs(nz)
                    var perpX: Float
                    var perpY: Float
                    var perpZ: Float
                    if (absNx <= absNy && absNx <= absNz) {
                        perpX = 0f
                        perpY = -nz
                        perpZ = ny
                    } else if (absNy <= absNx && absNy <= absNz) {
                        perpX = nz
                        perpY = 0f
                        perpZ = -nx
                    } else {
                        perpX = -ny
                        perpY = nx
                        perpZ = 0f
                    }
                    val len = sqrt((perpX * perpX + perpY * perpY + perpZ * perpZ).toDouble()).toFloat()
                    val t1x = if (len > 1e-6f) perpX / len else 0f
                    val t1y = if (len > 1e-6f) perpY / len else 1f
                    val t1z = if (len > 1e-6f) perpZ / len else 0f

                    val lambda = a2aManifolds.t1Lambda(manifoldIdx, constraintIdx)

                    a2aT1ManifoldData.addData(
                        baseIdx = baseIdx,
                        manifoldIdx = manifoldIdx,
                        constraintIdx = constraintIdx,
                        b1 = b1,
                        b2 = b2,

                        nx = t1x,
                        ny = t1y,
                        nz = t1z,

                        bias = 0f,
                        lambda = lambda,
                        manifolds = a2aManifolds
                    )
                }
            }

            var a2sCursor = 0

            for (manifoldIdx in 0..<a2sManifolds.size()) {
                val b1Idx = a2sManifolds.bodyIdx(manifoldIdx)
                val b1 = world.activeBodies.fastGet(b1Idx)!!

                val im1 = b1.inverseMass.toFloat()

                val numContacts = a2sManifolds.numContacts(manifoldIdx)

                a2sCursor = a2sT1ManifoldData.add(
                    cursor = a2sCursor,
                    body1Idx = b1Idx,

                    im1 = im1,

                    numConstraints = numContacts,
                ) { baseIdx, constraintIdx ->
                    val nx = a2sManifolds.normX(manifoldIdx, constraintIdx)
                    val ny = a2sManifolds.normY(manifoldIdx, constraintIdx)
                    val nz = a2sManifolds.normZ(manifoldIdx, constraintIdx)

                    val absNx = abs(nx)
                    val absNy = abs(ny)
                    val absNz = abs(nz)
                    var perpX: Float
                    var perpY: Float
                    var perpZ: Float
                    if (absNx <= absNy && absNx <= absNz) {
                        perpX = 0f
                        perpY = -nz
                        perpZ = ny
                    } else if (absNy <= absNx && absNy <= absNz) {
                        perpX = nz
                        perpY = 0f
                        perpZ = -nx
                    } else {
                        perpX = -ny
                        perpY = nx
                        perpZ = 0f
                    }
                    val len = sqrt((perpX * perpX + perpY * perpY + perpZ * perpZ).toDouble()).toFloat()
                    val t1x = if (len > 1e-6f) perpX / len else 0f
                    val t1y = if (len > 1e-6f) perpY / len else 1f
                    val t1z = if (len > 1e-6f) perpZ / len else 0f

                    val lambda = a2sManifolds.t1Lambda(manifoldIdx, constraintIdx)

                    a2sT1ManifoldData.addData(
                        baseIdx = baseIdx,
                        manifoldIdx = manifoldIdx,
                        constraintIdx = constraintIdx,
                        b1 = b1,

                        nx = t1x,
                        ny = t1y,
                        nz = t1z,

                        bias = 0f,
                        lambda = lambda,
                        manifolds = a2sManifolds
                    )
                }
            }

            latch.countDown()
        }

        setupExecutor.execute {
            var a2aCursor = 0

            for (manifoldIdx in 0..<a2aManifolds.size()) {
                val b1Idx = a2aManifolds.bodyAIdx(manifoldIdx)
                val b2Idx = a2aManifolds.bodyBIdx(manifoldIdx)
                val b1 = world.activeBodies.fastGet(b1Idx)!!
                val b2 = world.activeBodies.fastGet(b2Idx)!!

                val im1 = b1.inverseMass.toFloat()
                val im2 = b2.inverseMass.toFloat()

                val numContacts = a2aManifolds.numContacts(manifoldIdx)

                a2aCursor = a2aT2ManifoldData.add(
                    cursor = a2aCursor,
                    body1Idx = b1Idx,
                    body2Idx = b2Idx,

                    im1 = im1,
                    im2 = im2,

                    numConstraints = numContacts,
                ) { baseIdx, constraintIdx ->
                    val nx = a2aManifolds.normX(manifoldIdx, constraintIdx)
                    val ny = a2aManifolds.normY(manifoldIdx, constraintIdx)
                    val nz = a2aManifolds.normZ(manifoldIdx, constraintIdx)

                    val absNx = abs(nx)
                    val absNy = abs(ny)
                    val absNz = abs(nz)
                    var perpX: Float
                    var perpY: Float
                    var perpZ: Float
                    if (absNx <= absNy && absNx <= absNz) {
                        perpX = 0f
                        perpY = -nz
                        perpZ = ny
                    } else if (absNy <= absNx && absNy <= absNz) {
                        perpX = nz
                        perpY = 0f
                        perpZ = -nx
                    } else {
                        perpX = -ny
                        perpY = nx
                        perpZ = 0f
                    }
                    val len = sqrt((perpX * perpX + perpY * perpY + perpZ * perpZ).toDouble()).toFloat()
                    val t1x = if (len > 1e-6f) perpX / len else 0f
                    val t1y = if (len > 1e-6f) perpY / len else 1f
                    val t1z = if (len > 1e-6f) perpZ / len else 0f

                    val t2x = ny * t1z - nz * t1y
                    val t2y = nz * t1x - nx * t1z
                    val t2z = nx * t1y - ny * t1x

                    val lambda = a2aManifolds.t1Lambda(manifoldIdx, constraintIdx)

                    a2aT2ManifoldData.addData(
                        baseIdx = baseIdx,
                        manifoldIdx = manifoldIdx,
                        constraintIdx = constraintIdx,
                        b1 = b1,
                        b2 = b2,

                        nx = t2x,
                        ny = t2y,
                        nz = t2z,

                        bias = 0f,
                        lambda = lambda,
                        manifolds = a2aManifolds
                    )
                }
            }

            var a2sCursor = 0

            for (manifoldIdx in 0..<a2sManifolds.size()) {
                val b1Idx = a2sManifolds.bodyIdx(manifoldIdx)
                val b1 = world.activeBodies.fastGet(b1Idx)!!

                val im1 = b1.inverseMass.toFloat()

                val numContacts = a2sManifolds.numContacts(manifoldIdx)

                a2sCursor = a2sT2ManifoldData.add(
                    cursor = a2sCursor,
                    body1Idx = b1Idx,

                    im1 = im1,

                    numConstraints = numContacts,
                ) { baseIdx, constraintIdx ->
                    val nx = a2sManifolds.normX(manifoldIdx, constraintIdx)
                    val ny = a2sManifolds.normY(manifoldIdx, constraintIdx)
                    val nz = a2sManifolds.normZ(manifoldIdx, constraintIdx)

                    val absNx = abs(nx)
                    val absNy = abs(ny)
                    val absNz = abs(nz)
                    var perpX: Float
                    var perpY: Float
                    var perpZ: Float
                    if (absNx <= absNy && absNx <= absNz) {
                        perpX = 0f
                        perpY = -nz
                        perpZ = ny
                    } else if (absNy <= absNx && absNy <= absNz) {
                        perpX = nz
                        perpY = 0f
                        perpZ = -nx
                    } else {
                        perpX = -ny
                        perpY = nx
                        perpZ = 0f
                    }
                    val len = sqrt((perpX * perpX + perpY * perpY + perpZ * perpZ).toDouble()).toFloat()
                    val t1x = if (len > 1e-6f) perpX / len else 0f
                    val t1y = if (len > 1e-6f) perpY / len else 1f
                    val t1z = if (len > 1e-6f) perpZ / len else 0f

                    val t2x = ny * t1z - nz * t1y
                    val t2y = nz * t1x - nx * t1z
                    val t2z = nx * t1y - ny * t1x

                    val lambda = a2sManifolds.t1Lambda(manifoldIdx, constraintIdx)

                    a2sT2ManifoldData.addData(
                        baseIdx = baseIdx,
                        manifoldIdx = manifoldIdx,
                        constraintIdx = constraintIdx,
                        b1 = b1,

                        nx = t2x,
                        ny = t2y,
                        nz = t2z,

                        bias = 0f,
                        lambda = lambda,
                        manifolds = a2sManifolds
                    )
                }
            }

            latch.countDown()
        }

        latch.await()

        if (carryover > 0f) warm()
    }

    private fun warm() {
        warm(a2aNormalManifoldData, a2aNumManifolds)
        warm(a2sNormalManifoldData, a2sNumManifolds)

        warm(a2aT1ManifoldData, a2aNumManifolds)
        warm(a2sT1ManifoldData, a2sNumManifolds)

        warm(a2aT2ManifoldData, a2aNumManifolds)
        warm(a2sT2ManifoldData, a2sNumManifolds)
    }

    private fun warm(
        manifolds: A2AManifoldData,
        num: Int,
    ) {
        val bodyData = parent.flatBodyData
        manifolds.forEachConstraint(
            numManifolds = num,
        ) { b1Idx, b2Idx, im1, im2, rawIdx ->
            val t = manifolds[rawIdx + A2A_LAMBDA_OFFSET]

            val j10 = manifolds[rawIdx + 0]
            val j11 = manifolds[rawIdx + 1]
            val j12 = manifolds[rawIdx + 2]

            val j20 = -j10
            val j21 = -j11
            val j22 = -j12

            val ej10 = j10 * im1
            val ej11 = j11 * im1
            val ej12 = j12 * im1
            val ej13 = manifolds[rawIdx + 9]
            val ej14 = manifolds[rawIdx + 10]
            val ej15 = manifolds[rawIdx + 11]

            val ej20 = j20 * im2
            val ej21 = j21 * im2
            val ej22 = j22 * im2
            val ej23 = manifolds[rawIdx + 12]
            val ej24 = manifolds[rawIdx + 13]
            val ej25 = manifolds[rawIdx + 14]

            bodyData[b1Idx * 6 + 0] += ej10 * t * relaxation
            bodyData[b1Idx * 6 + 1] += ej11 * t * relaxation
            bodyData[b1Idx * 6 + 2] += ej12 * t * relaxation
            bodyData[b1Idx * 6 + 3] += ej13 * t * relaxation
            bodyData[b1Idx * 6 + 4] += ej14 * t * relaxation
            bodyData[b1Idx * 6 + 5] += ej15 * t * relaxation

            bodyData[b2Idx * 6 + 0] += ej20 * t * relaxation
            bodyData[b2Idx * 6 + 1] += ej21 * t * relaxation
            bodyData[b2Idx * 6 + 2] += ej22 * t * relaxation
            bodyData[b2Idx * 6 + 3] += ej23 * t * relaxation
            bodyData[b2Idx * 6 + 4] += ej24 * t * relaxation
            bodyData[b2Idx * 6 + 5] += ej25 * t * relaxation
        }
    }

    private fun warm(
        manifolds: A2SManifoldData,
        num: Int,
    ) {
        val bodyData = parent.flatBodyData
        manifolds.forEachConstraint(
            numManifolds = num,
        ) { b1Idx, im1, rawIdx ->
            val t = manifolds[rawIdx + A2S_LAMBDA_OFFSET]

            val j10 = manifolds[rawIdx + 0]
            val j11 = manifolds[rawIdx + 1]
            val j12 = manifolds[rawIdx + 2]

            val ej10 = j10 * im1
            val ej11 = j11 * im1
            val ej12 = j12 * im1
            val ej13 = manifolds[rawIdx + 6]
            val ej14 = manifolds[rawIdx + 7]
            val ej15 = manifolds[rawIdx + 8]

            bodyData[b1Idx * 6 + 0] += ej10 * t * relaxation
            bodyData[b1Idx * 6 + 1] += ej11 * t * relaxation
            bodyData[b1Idx * 6 + 2] += ej12 * t * relaxation
            bodyData[b1Idx * 6 + 3] += ej13 * t * relaxation
            bodyData[b1Idx * 6 + 4] += ej14 * t * relaxation
            bodyData[b1Idx * 6 + 5] += ej15 * t * relaxation
        }
    }

    private inline fun solve(
        manifolds: A2AManifoldData,
        num: Int,

        lambdaTransform: (l: Float, lIdx: Int) -> Float,
    ) {
        val bodyData = parent.flatBodyData
        manifolds.forEachConstraint(
            numManifolds = num,
        ) { b1Idx, b2Idx, im1, im2, rawIdx ->
            val lambdaIdx = rawIdx + A2A_LAMBDA_OFFSET
            val t = manifolds[lambdaIdx]

            val v10 = bodyData[b1Idx * 6 + 0]
            val v11 = bodyData[b1Idx * 6 + 1]
            val v12 = bodyData[b1Idx * 6 + 2]
            val v13 = bodyData[b1Idx * 6 + 3]
            val v14 = bodyData[b1Idx * 6 + 4]
            val v15 = bodyData[b1Idx * 6 + 5]

            val v20 = bodyData[b2Idx * 6 + 0]
            val v21 = bodyData[b2Idx * 6 + 1]
            val v22 = bodyData[b2Idx * 6 + 2]
            val v23 = bodyData[b2Idx * 6 + 3]
            val v24 = bodyData[b2Idx * 6 + 4]
            val v25 = bodyData[b2Idx * 6 + 5]

            val bias = manifolds[rawIdx + A2A_BIAS_OFFSET]
            val iden = manifolds[rawIdx + A2A_IDEN_OFFSET]

            val j10 = manifolds[rawIdx + 0]
            val j11 = manifolds[rawIdx + 1]
            val j12 = manifolds[rawIdx + 2]
            val j13 = manifolds[rawIdx + 3]
            val j14 = manifolds[rawIdx + 4]
            val j15 = manifolds[rawIdx + 5]

            val j20 = -j10
            val j21 = -j11
            val j22 = -j12
            val j23 = manifolds[rawIdx + 6]
            val j24 = manifolds[rawIdx + 7]
            val j25 = manifolds[rawIdx + 8]

            val ej10 = j10 * im1
            val ej11 = j11 * im1
            val ej12 = j12 * im1
            val ej13 = manifolds[rawIdx + 9]
            val ej14 = manifolds[rawIdx + 10]
            val ej15 = manifolds[rawIdx + 11]

            val ej20 = j20 * im2
            val ej21 = j21 * im2
            val ej22 = j22 * im2
            val ej23 = manifolds[rawIdx + 12]
            val ej24 = manifolds[rawIdx + 13]
            val ej25 = manifolds[rawIdx + 14]

            val l = lambdaTransform(
                t - iden * fma(
                    j10, v10,
                    fma(
                        j11, v11,
                        fma(
                            j12, v12,
                            fma(
                                j13, v13,
                                fma(
                                    j14, v14,
                                    fma(
                                        j15, v15,
                                        fma(
                                            j20, v20,
                                            fma(
                                                j21, v21,
                                                fma(
                                                    j22, v22,
                                                    fma(
                                                        j23, v23,
                                                        fma(
                                                            j24, v24,
                                                            fma(
                                                                j25, v25,
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
                ),
                lambdaIdx
            )

            manifolds[lambdaIdx] = l

            bodyData[b1Idx * 6 + 0] += ej10 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 1] += ej11 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 2] += ej12 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 3] += ej13 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 4] += ej14 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 5] += ej15 * (l - t) * relaxation

            bodyData[b2Idx * 6 + 0] += ej20 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 1] += ej21 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 2] += ej22 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 3] += ej23 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 4] += ej24 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 5] += ej25 * (l - t) * relaxation
        }
    }

    private inline fun solve(
        blocks: A2SManifoldData,
        num: Int,

        lambdaTransform: (l: Float, lIdx: Int) -> Float,
    ) {
        val bodyData = parent.flatBodyData
        blocks.forEachConstraint(
            numManifolds = num,
        ) { b1Idx, im1, rawIdx ->
            val lambdaIdx = rawIdx + A2S_LAMBDA_OFFSET
            val t = blocks[lambdaIdx]

            val v10 = bodyData[b1Idx * 6 + 0]
            val v11 = bodyData[b1Idx * 6 + 1]
            val v12 = bodyData[b1Idx * 6 + 2]
            val v13 = bodyData[b1Idx * 6 + 3]
            val v14 = bodyData[b1Idx * 6 + 4]
            val v15 = bodyData[b1Idx * 6 + 5]

            val bias = blocks[rawIdx + A2S_BIAS_OFFSET]
            val iden = blocks[rawIdx + A2S_IDEN_OFFSET]

            val j10 = blocks[rawIdx + 0]
            val j11 = blocks[rawIdx + 1]
            val j12 = blocks[rawIdx + 2]
            val j13 = blocks[rawIdx + 3]
            val j14 = blocks[rawIdx + 4]
            val j15 = blocks[rawIdx + 5]

            val ej10 = j10 * im1
            val ej11 = j11 * im1
            val ej12 = j12 * im1
            val ej13 = blocks[rawIdx + 6]
            val ej14 = blocks[rawIdx + 7]
            val ej15 = blocks[rawIdx + 8]

            val l = lambdaTransform(
                t - iden * fma(
                    j10, v10,
                    fma(
                        j11, v11,
                        fma(
                            j12, v12,
                            fma(
                                j13, v13,
                                fma(
                                    j14, v14,
                                    fma(
                                        j15, v15,
                                        bias
                                    )
                                )
                            )
                        )
                    )
                ),
                lambdaIdx
            )

            blocks[lambdaIdx] = l

            bodyData[b1Idx * 6 + 0] += ej10 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 1] += ej11 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 2] += ej12 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 3] += ej13 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 4] += ej14 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 5] += ej15 * (l - t) * relaxation
        }
    }

    fun solveNormals(iteration: Int) {
        solve(a2aNormalManifoldData, a2aNumManifolds) { l, lIdx ->
            val r = max(0f, l)
            normalDeltaLambdas[iteration - 1] += abs(r - a2aNormalManifoldData[lIdx])
            r
        }

        solve(a2sNormalManifoldData, a2sNumManifolds) { l, lIdx ->
            val r = max(0f, l)
            normalDeltaLambdas[iteration - 1] += abs(r - a2sNormalManifoldData[lIdx])
            r
        }
    }

    fun solveFrictions() {
        solve(a2aT1ManifoldData, a2aNumManifolds) { l, lIdx ->
            val nl = a2aNormalManifoldData[lIdx]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }

        solve(a2aT2ManifoldData, a2aNumManifolds) { l, lIdx ->
            val nl = a2aNormalManifoldData[lIdx]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }

        solve(a2sT1ManifoldData, a2sNumManifolds) { l, lIdx ->
            val nl = a2sNormalManifoldData[lIdx]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }

        solve(a2sT2ManifoldData, a2sNumManifolds) { l, lIdx ->
            val nl = a2sNormalManifoldData[lIdx]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }
    }

    fun heatUp() {
        if (carryover <= 0f) return
        val a2aNumManifolds = world.manifoldBuffer.size()
        val a2aManifolds = world.manifoldBuffer
        val a2aPrevData = world.prevContactData
        a2aPrevData.ensureCapacity(a2aNumManifolds)
        val a2aPrevMap = world.prevContactMap

        val a2sNumManifolds = world.envManifoldBuffer.size()
        val a2sManifolds = world.envManifoldBuffer
        val a2sPrevData = world.prevEnvContactData
        a2sPrevData.ensureCapacity(a2sNumManifolds)
        val a2sPrevMap = world.prevEnvContactMap

        val stay = Udar.CONFIG.collision.stay

        a2aNormalManifoldData.forEachManifold(
            numManifolds = a2aNumManifolds,
        ) { b1Idx, b2Idx, _, _, numConstraints, i, cursor ->
            val manifoldID = a2aManifolds.manifoldID(i)
            val existingIdx = a2aPrevMap.get(manifoldID)
            var c: Int
            if (existingIdx == -1) {
                c = a2aPrevData.popFree()
                a2aPrevMap.put(manifoldID, c)
                c = a2aPrevData.start(c, numConstraints, stay, manifoldID)
            } else {
                c = a2aPrevData.start(existingIdx, numConstraints, stay, manifoldID)
                a2aPrevData.stay(existingIdx, stay)
            }

            var j = 0
            while (j < numConstraints) {
                val nl =
                    a2aNormalManifoldData[cursor + A2A_MANIFOLD_DATA_SIZE + j * A2A_CONSTRAINT_DATA_SIZE + A2A_LAMBDA_OFFSET] * carryover
                val t1l =
                    a2aT1ManifoldData[cursor + A2A_MANIFOLD_DATA_SIZE + j * A2A_CONSTRAINT_DATA_SIZE + A2A_LAMBDA_OFFSET] * carryover
                val t2l =
                    a2aT2ManifoldData[cursor + A2A_MANIFOLD_DATA_SIZE + j * A2A_CONSTRAINT_DATA_SIZE + A2A_LAMBDA_OFFSET] * carryover

                val x1 = a2aManifolds.pointAX(i, j)
                val y1 = a2aManifolds.pointAY(i, j)
                val z1 = a2aManifolds.pointAZ(i, j)

                val x2 = a2aManifolds.pointBX(i, j)
                val y2 = a2aManifolds.pointBY(i, j)
                val z2 = a2aManifolds.pointBZ(i, j)

                c = a2aPrevData.add(
                    cursor = c,
                    x1 = x1,
                    y1 = y1,
                    z1 = z1,
                    x2 = x2,
                    y2 = y2,
                    z2 = z2,
                    nl = nl,
                    t1l = t1l,
                    t2l = t2l,
                )

                j++
            }
        }

        a2sNormalManifoldData.forEachManifold(
            numManifolds = a2sNumManifolds,
        ) { b1Idx, _, numConstraints, i, cursor ->
            val manifoldID = a2sManifolds.manifoldID(i)
            val existingIdx = a2sPrevMap.get(manifoldID)
            var c: Int
            if (existingIdx == -1) {
                c = a2sPrevData.popFree()
                a2sPrevMap.put(manifoldID, c)
                c = a2sPrevData.start(c, numConstraints, stay, manifoldID)
            } else {
                c = a2sPrevData.start(existingIdx, numConstraints, stay, manifoldID)
                a2sPrevData.stay(existingIdx, stay)
            }

            var j = 0
            while (j < numConstraints) {
                val nl =
                    a2sNormalManifoldData[cursor + A2S_MANIFOLD_DATA_SIZE + j * A2S_CONSTRAINT_DATA_SIZE + A2S_LAMBDA_OFFSET] * carryover
                val t1l =
                    a2sT1ManifoldData[cursor + A2S_MANIFOLD_DATA_SIZE + j * A2S_CONSTRAINT_DATA_SIZE + A2S_LAMBDA_OFFSET] * carryover
                val t2l =
                    a2sT2ManifoldData[cursor + A2S_MANIFOLD_DATA_SIZE + j * A2S_CONSTRAINT_DATA_SIZE + A2S_LAMBDA_OFFSET] * carryover

                val x1 = a2sManifolds.pointAX(i, j)
                val y1 = a2sManifolds.pointAY(i, j)
                val z1 = a2sManifolds.pointAZ(i, j)

                c = a2sPrevData.add(
                    cursor = c,
                    x = x1,
                    y = y1,
                    z = z1,
                    nl = nl,
                    t1l = t1l,
                    t2l = t2l,
                )

                j++
            }
        }
    }

    fun reportLambdas() {
        val iterations = Udar.CONFIG.collision.normalIterations
        if (iterations <= 0) return
        val n =
            world.manifoldBuffer.numContacts + world.envManifoldBuffer.numContacts.get()
        if (n == 0) return
        for (i in 0..<normalDeltaLambdas.size) {
            normalDeltaLambdas[i] /= n
        }

        val first = normalDeltaLambdas[0]
        println("Δlambdas:")
        println("  %4d: %8.2e ".format(1, first) + "*".repeat(10))
        for (i in 2..normalDeltaLambdas.size) {
            val v = normalDeltaLambdas[i - 1]
            if (first == 0f) {
                println("  %4d: %8.2e ".format(i, v))
            } else {
                println("  %4d: %8.2e ".format(i, v) + "*".repeat(abs(v / first * 10.0).roundToInt()))
            }
        }
    }
}