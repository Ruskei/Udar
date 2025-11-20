package com.ixume.udar.physics.splitting

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.*
import java.lang.Math.fma
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import kotlin.math.max

class MassSplittingConstraintSolver(val constraintData: LocalConstraintData) {
    var timeStep = Udar.CONFIG.timeStep
    var bias = Udar.CONFIG.massSplittingConfig.bias
    private var threads = Udar.CONFIG.massSplittingConfig.threads
    private var runnables = Array(threads) { SolverRunnable(this) }

    private var temp = ConstraintData()
    private var numPairsPerBody = IntArray(0)
    private var bodyAverages = FloatArray(0)

    private var executor = Executors.newFixedThreadPool(threads)

    fun setup() {
        timeStep = Udar.CONFIG.timeStep

        if (bodyAverages.size < constraintData.physicsWorld.activeBodies.size() * 6) {
            bodyAverages =
                FloatArray(max(constraintData.physicsWorld.activeBodies.size() * 6, bodyAverages.size * 3 / 2))
        }

        if (threads != Udar.CONFIG.massSplittingConfig.threads) {
            threads = Udar.CONFIG.massSplittingConfig.threads
            runnables = Array(threads) { SolverRunnable(this) }
            executor.shutdown()
            executor = Executors.newFixedThreadPool(threads)
        }

        if (numPairsPerBody.size < constraintData.physicsWorld.activeBodies.size()) {
            numPairsPerBody =
                IntArray(max(numPairsPerBody.size * 3 / 2, constraintData.physicsWorld.activeBodies.size()))
        } else {
            numPairsPerBody.fill(0)
        }

        val manifolds = constraintData.physicsWorld.manifoldBuffer
        val envManifolds = constraintData.physicsWorld.envManifoldBuffer

        for (i in 0..<manifolds.size()) {
            numPairsPerBody[manifolds.bodyAIdx(i)]++
            numPairsPerBody[manifolds.bodyBIdx(i)]++
        }

        for (i in 0..<envManifolds.size()) {
            numPairsPerBody[envManifolds.bodyIdx(i)]++
        }

        val envManifoldNumContacts = envManifolds.numContacts.get()
        for (runnable in runnables) {
            runnable.cursor = 0
            runnable.numBlocks = 0
            runnable.flatBodyData = constraintData.flatBodyData
            runnable.ensureSize(manifolds.size() + envManifolds.size(), manifolds.numContacts + envManifoldNumContacts)
        }

        for (i in 0..<manifolds.size()) {
            val thread = i % threads
            val runnable = runnables[thread]

            runnable.numBlocks++

            val b1Idx = manifolds.bodyAIdx(i)
            val b2Idx = manifolds.bodyBIdx(i)
            val b1 = constraintData.physicsWorld.activeBodies.fastGet(b1Idx)!!
            val b2 = constraintData.physicsWorld.activeBodies.fastGet(b2Idx)!!

            val n1 = numPairsPerBody[b1Idx]
            val n2 = numPairsPerBody[b2Idx]

            val im1 = b1.inverseMass.toFloat()
            val im2 = b2.inverseMass.toFloat()

            val ii1 = b1.inverseInertia
            val ii2 = b2.inverseInertia

            val numContacts = manifolds.numContacts(i)

            runnable.cursor = runnable.constraintBlocks.add(
                cursor = runnable.cursor,
                temp = temp,
                body1Idx = b1Idx,
                body2Idx = b2Idx,

                numContacts = numContacts,
            ) { data, idx ->
                /*
                (P_A - P_B).n = 0
                (V_A + O_A x R_A - V_B - O_B x R_B).n = 0
                V_A.n + O_A x R_A.n - V_B.n - O_B x R_B.n = 0
                V_A.n + O_A.(R_A x n) - V_B.n - O_B.(R_B x n) = 0
                J = (n, R_A x n, -n, -R_B x n)
                V = (V_A, O_A, V_B, O_B)
                JV = 0
                 */
                val nx = manifolds.normX(i, idx)
                val ny = manifolds.normY(i, idx)
                val nz = manifolds.normZ(i, idx)

                val rax = manifolds.pointAX(i, idx) - manifolds.bodyAX(i)
                val ray = manifolds.pointAY(i, idx) - manifolds.bodyAY(i)
                val raz = manifolds.pointAZ(i, idx) - manifolds.bodyAZ(i)

                val rbx = manifolds.pointBX(i, idx) - manifolds.bodyBX(i)
                val rby = manifolds.pointBY(i, idx) - manifolds.bodyBY(i)
                val rbz = manifolds.pointBZ(i, idx) - manifolds.bodyBZ(i)

                data.j10 = nx
                data.j11 = ny
                data.j12 = nz
                data.j13 = fma(ray, nz, -raz * ny)
                data.j14 = fma(raz, nx, -rax * nz)
                data.j15 = fma(rax, ny, -ray * nx)

                data.j20 = -nx
                data.j21 = -ny
                data.j22 = -nz
                data.j23 = fma(-rby, nz, rbz * ny)
                data.j24 = fma(-rbz, nx, rbx * nz)
                data.j25 = fma(-rbx, ny, rby * nx)

                data.ej10 = data.j10 * im1
                data.ej11 = data.j11 * im1
                data.ej12 = data.j12 * im1
                data.ej13 =
                    fma(ii1.m00.toFloat(), data.j13, fma(ii1.m10.toFloat(), data.j14, ii1.m20.toFloat() * data.j15))
                data.ej14 =
                    fma(ii1.m01.toFloat(), data.j13, fma(ii1.m11.toFloat(), data.j14, ii1.m21.toFloat() * data.j15))
                data.ej15 =
                    fma(ii1.m02.toFloat(), data.j13, fma(ii1.m12.toFloat(), data.j14, ii1.m22.toFloat() * data.j15))

                data.ej20 = data.j20 * im2
                data.ej21 = data.j21 * im2
                data.ej22 = data.j22 * im2
                data.ej23 =
                    fma(ii2.m00.toFloat(), data.j23, fma(ii2.m10.toFloat(), data.j24, ii2.m20.toFloat() * data.j25))
                data.ej24 =
                    fma(ii2.m01.toFloat(), data.j23, fma(ii2.m11.toFloat(), data.j24, ii2.m21.toFloat() * data.j25))
                data.ej25 =
                    fma(ii2.m02.toFloat(), data.j23, fma(ii2.m12.toFloat(), data.j24, ii2.m22.toFloat() * data.j25))

                data.lambda = 0f // no warm starting yet
                data.den =
                    1f / (
                            (data.ej10 * data.j10 + data.ej11 * data.j11 + data.ej12 * data.j12 +
                             data.j13 * data.ej13 + data.j14 * data.ej14 + data.j15 * data.ej15) * n1 +
                            (data.ej20 * data.j20 + data.ej21 * data.j21 + data.ej22 * data.j22 +
                             data.j23 * data.ej23 + data.j24 * data.ej24 + data.j25 * data.ej25) * n2
                         )

                data.error = manifolds.depth(i, idx)
            }
        }

        for (i in 0..<envManifolds.size()) {
            val thread = i % threads
            val runnable = runnables[thread]

            runnable.numBlocks++

            val bIdx = envManifolds.bodyIdx(i)
            val b = constraintData.physicsWorld.activeBodies.fastGet(bIdx)!!

            val n1 = numPairsPerBody[bIdx]
            val im = b.inverseMass.toFloat()
            val ii = b.inverseInertia

            val numContacts = envManifolds.numContacts(i)

            runnable.cursor = runnable.constraintBlocks.add(
                cursor = runnable.cursor,
                temp = temp,
                body1Idx = bIdx,
                body2Idx = -1,

                numContacts = numContacts,
            ) { data, idx ->
                val nx = envManifolds.normX(i, idx)
                val ny = envManifolds.normY(i, idx)
                val nz = envManifolds.normZ(i, idx)

                val rax = envManifolds.pointAX(i, idx) - envManifolds.bodyX(i)
                val ray = envManifolds.pointAY(i, idx) - envManifolds.bodyY(i)
                val raz = envManifolds.pointAZ(i, idx) - envManifolds.bodyZ(i)

                data.j10 = nx
                data.j11 = ny
                data.j12 = nz
                data.j13 = fma(ray, nz, -raz * ny)
                data.j14 = fma(raz, nx, -rax * nz)
                data.j15 = fma(rax, ny, -ray * nx)

                data.j20 = 0f
                data.j21 = 0f
                data.j22 = 0f
                data.j23 = 0f
                data.j24 = 0f
                data.j25 = 0f

                data.ej10 = data.j10 * im
                data.ej11 = data.j11 * im
                data.ej12 = data.j12 * im
                data.ej13 =
                    fma(ii.m00.toFloat(), data.j13, fma(ii.m10.toFloat(), data.j14, ii.m20.toFloat() * data.j15))
                data.ej14 =
                    fma(ii.m01.toFloat(), data.j13, fma(ii.m11.toFloat(), data.j14, ii.m21.toFloat() * data.j15))
                data.ej15 =
                    fma(ii.m02.toFloat(), data.j13, fma(ii.m12.toFloat(), data.j14, ii.m22.toFloat() * data.j15))

                data.ej20 = 0f
                data.ej21 = 0f
                data.ej22 = 0f
                data.ej23 = 0f
                data.ej24 = 0f
                data.ej25 = 0f

                data.lambda = 0f // no warm starting yet
                data.den =
                    1f / (
                            (data.ej10 * data.j10 + data.ej11 * data.j11 + data.ej12 * data.j12 +
                             data.j13 * data.ej13 + data.j14 * data.ej14 + data.j15 * data.ej15) * n1
                         )

                data.error = envManifolds.depth(i, idx)
            }
        }
    }

    fun solve() {
        val latch = CountDownLatch(threads)
        for (thread in 0..<threads) {
            executor.execute {
                runnables[thread].run()

                latch.countDown()
            }
        }

        latch.await()

        average()
    }

    private fun average() {
        bodyAverages.fill(0f)

        for (runnable in runnables) {
            runnable.constraintBlocks.forEachBlock(runnable.numBlocks) { b1Idx, b2Idx, v10, v11, v12, v13, v14, v15, v20, v21, v22, v23, v24, v25 ->
                if (b1Idx != -1) {
                    val n1 = numPairsPerBody[b1Idx]

                    bodyAverages[b1Idx * 6 + 0] += v10 / n1
                    bodyAverages[b1Idx * 6 + 1] += v11 / n1
                    bodyAverages[b1Idx * 6 + 2] += v12 / n1
                    bodyAverages[b1Idx * 6 + 3] += v13 / n1
                    bodyAverages[b1Idx * 6 + 4] += v14 / n1
                    bodyAverages[b1Idx * 6 + 5] += v15 / n1
                }

                if (b2Idx != -1) {
                    val n2 = numPairsPerBody[b2Idx]

                    bodyAverages[b2Idx * 6 + 0] += v20 / n2
                    bodyAverages[b2Idx * 6 + 1] += v21 / n2
                    bodyAverages[b2Idx * 6 + 2] += v22 / n2
                    bodyAverages[b2Idx * 6 + 3] += v23 / n2
                    bodyAverages[b2Idx * 6 + 4] += v24 / n2
                    bodyAverages[b2Idx * 6 + 5] += v25 / n2
                }
            }
        }

        for (i in 0..<constraintData.physicsWorld.activeBodies.size()) {
            val n = numPairsPerBody[i]
            if (n == 0) continue
            bodyAverages.copyInto(
                destination = constraintData.flatBodyData,
                destinationOffset = i * 6,
                startIndex = i * 6,
                endIndex = i * 6 + 6
            )
        }
    }
}


private const val BASE_CONSTRAINT_BLOCK_SIZE = 15
private const val CONSTRAINT_DATA_SIZE = 27

/*
for contact block solving we need:
  beta_1 (idx) (int)
  beta_2 (idx) (int)
  D_alpha_1 (velocity of body 1) (1x6)
  D_alpha_2 (velocity of body 2) (1x6)
  
  numContacts (int)
    J_alpha_beta_1 (jacobian) (1x6)
    J_alpha_beta_2 (jacobian) (1x6)
    M_1^-1*J_alpha_beta_1 (jacobian) (1x6)
    M_2^-1*J_alpha_beta_2 (jacobian) (1x6)
    lambda (float)
    den (float)
    error (float)
 
since we're iterating this in sequence, it's okay that size differs per element
we need to pass a list of contact blocks to every thread
 */

typealias ConstraintBlocks = FloatArray

private inline fun ConstraintBlocks.forEachBlock(
    numBlocks: Int,
    block: (
        b1Idx: Int,
        b2Idx: Int,

        v10: Float,
        v11: Float,
        v12: Float,
        v13: Float,
        v14: Float,
        v15: Float,

        v20: Float,
        v21: Float,
        v22: Float,
        v23: Float,
        v24: Float,
        v25: Float,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()

        val v10 = this[p++]
        val v11 = this[p++]
        val v12 = this[p++]
        val v13 = this[p++]
        val v14 = this[p++]
        val v15 = this[p++]

        val v20 = this[p++]
        val v21 = this[p++]
        val v22 = this[p++]
        val v23 = this[p++]
        val v24 = this[p++]
        val v25 = this[p++]

        block(
            b1Idx, b2Idx,

            v10,
            v11,
            v12,
            v13,
            v14,
            v15,

            v20,
            v21,
            v22,
            v23,
            v24,
            v25,
        )

        val numConstraints = this[p++].toRawBits()
        p += numConstraints * CONSTRAINT_DATA_SIZE
        i++
    }
}

private inline fun ConstraintBlocks.forEachConstraint(
    numBlocks: Int,
    flatBodyData: FloatArray,
    temp: ConstraintData,
    block: (
        b1Idx: Int,
        b2Idx: Int,

        v1Idx: Int,
        v2Idx: Int,

        data: ConstraintData,
        lIdx: Int,

        numConstraints: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()

        val v1Idx = p
        if (b1Idx == -1) {
            fill(0f, p, p + 6)
            p += 6
        } else {
            this[p++] = flatBodyData.vx(b1Idx)
            this[p++] = flatBodyData.vy(b1Idx)
            this[p++] = flatBodyData.vz(b1Idx)
            this[p++] = flatBodyData.ox(b1Idx)
            this[p++] = flatBodyData.oy(b1Idx)
            this[p++] = flatBodyData.oz(b1Idx)
        }

        val v2Idx = p
        if (b2Idx == -1) {
            fill(0f, p, p + 6)
            p += 6
        } else {
            this[p++] = flatBodyData.vx(b2Idx)
            this[p++] = flatBodyData.vy(b2Idx)
            this[p++] = flatBodyData.vz(b2Idx)
            this[p++] = flatBodyData.ox(b2Idx)
            this[p++] = flatBodyData.oy(b2Idx)
            this[p++] = flatBodyData.oz(b2Idx)
        }

        val numConstraints = this[p++].toRawBits()
        repeat(numConstraints) {
            temp.j10 = this[p++]
            temp.j11 = this[p++]
            temp.j12 = this[p++]
            temp.j13 = this[p++]
            temp.j14 = this[p++]
            temp.j15 = this[p++]

            temp.j20 = this[p++]
            temp.j21 = this[p++]
            temp.j22 = this[p++]
            temp.j23 = this[p++]
            temp.j24 = this[p++]
            temp.j25 = this[p++]

            temp.ej10 = this[p++]
            temp.ej11 = this[p++]
            temp.ej12 = this[p++]
            temp.ej13 = this[p++]
            temp.ej14 = this[p++]
            temp.ej15 = this[p++]

            temp.ej20 = this[p++]
            temp.ej21 = this[p++]
            temp.ej22 = this[p++]
            temp.ej23 = this[p++]
            temp.ej24 = this[p++]
            temp.ej25 = this[p++]

            val lIdx = p++
            temp.lambda = this[lIdx]
            temp.den = this[p++]
            temp.error = this[p++]

            block(
                b1Idx,
                b2Idx,
                v1Idx,
                v2Idx,

                temp,
                lIdx,

                numConstraints,
            )
        }

        i++
    }
}

/**
 * Returns new cursor position
 */
private fun ConstraintBlocks.add(
    cursor: Int,
    temp: ConstraintData,

    body1Idx: Int,
    body2Idx: Int,

    numContacts: Int,
    constraintDataSupplier: (data: ConstraintData, constraintIdx: Int) -> Unit,
): Int {
    var c = cursor

    this[c++] = Float.fromBits(body1Idx)
    this[c++] = Float.fromBits(body2Idx)

    //space for velocities
    repeat(12) {
        this[c++] = 0f
    }

    this[c++] = Float.fromBits(numContacts)

    for (i in 0..<numContacts) {
        constraintDataSupplier(temp, i)

        this[c++] = temp.j10
        this[c++] = temp.j11
        this[c++] = temp.j12
        this[c++] = temp.j13
        this[c++] = temp.j14
        this[c++] = temp.j15

        this[c++] = temp.j20
        this[c++] = temp.j21
        this[c++] = temp.j22
        this[c++] = temp.j23
        this[c++] = temp.j24
        this[c++] = temp.j25

        this[c++] = temp.ej10
        this[c++] = temp.ej11
        this[c++] = temp.ej12
        this[c++] = temp.ej13
        this[c++] = temp.ej14
        this[c++] = temp.ej15

        this[c++] = temp.ej20
        this[c++] = temp.ej21
        this[c++] = temp.ej22
        this[c++] = temp.ej23
        this[c++] = temp.ej24
        this[c++] = temp.ej25

        this[c++] = temp.lambda
        this[c++] = temp.den
        this[c++] = temp.error
    }

    return c
}

private class ConstraintData(
    var j10: Float = 0f,
    var j11: Float = 0f,
    var j12: Float = 0f,
    var j13: Float = 0f,
    var j14: Float = 0f,
    var j15: Float = 0f,

    var j20: Float = 0f,
    var j21: Float = 0f,
    var j22: Float = 0f,
    var j23: Float = 0f,
    var j24: Float = 0f,
    var j25: Float = 0f,

    var ej10: Float = 0f,
    var ej11: Float = 0f,
    var ej12: Float = 0f,
    var ej13: Float = 0f,
    var ej14: Float = 0f,
    var ej15: Float = 0f,

    var ej20: Float = 0f,
    var ej21: Float = 0f,
    var ej22: Float = 0f,
    var ej23: Float = 0f,
    var ej24: Float = 0f,
    var ej25: Float = 0f,

    var lambda: Float = 0f,
    var den: Float = 0f,
    var error: Float = 0f,
)

private class SolverRunnable(val solver: MassSplittingConstraintSolver) : Runnable {
    var cursor = 0
    var numBlocks = 0
    var constraintBlocks: ConstraintBlocks = ConstraintBlocks(0)
    var flatBodyData: FloatArray = FloatArray(0)

    private val temp = ConstraintData()

    fun ensureSize(numManifolds: Int, numContacts: Int) {
        if (constraintBlocks.size < numManifolds * BASE_CONSTRAINT_BLOCK_SIZE + numContacts * CONSTRAINT_DATA_SIZE) {
            constraintBlocks = ConstraintBlocks(
                max(
                    numManifolds * BASE_CONSTRAINT_BLOCK_SIZE + numContacts * CONSTRAINT_DATA_SIZE,
                    constraintBlocks.size * 3 / 2
                )
            )
        }
    }

    override fun run() {
        constraintBlocks.forEachConstraint(
            flatBodyData = flatBodyData,
            temp = temp,
            numBlocks = numBlocks,
        ) { b1Idx, b2Idx, v1Idx, v2Idx, data, lIdx, numConstraints ->
            val t = constraintBlocks[lIdx]

            val v10 = constraintBlocks[v1Idx + 0]
            val v11 = constraintBlocks[v1Idx + 1]
            val v12 = constraintBlocks[v1Idx + 2]
            val v13 = constraintBlocks[v1Idx + 3]
            val v14 = constraintBlocks[v1Idx + 4]
            val v15 = constraintBlocks[v1Idx + 5]

            val v20 = constraintBlocks[v2Idx + 0]
            val v21 = constraintBlocks[v2Idx + 1]
            val v22 = constraintBlocks[v2Idx + 2]
            val v23 = constraintBlocks[v2Idx + 3]
            val v24 = constraintBlocks[v2Idx + 4]
            val v25 = constraintBlocks[v2Idx + 5]

            val bias = (-solver.bias * temp.error / solver.timeStep).toFloat()

            val l = max(
                0f, t - temp.den * fma(
                    temp.j10, v10,
                    fma(
                        temp.j11, v11,
                        fma(
                            temp.j12, v12,
                            fma(
                                temp.j13, v13,
                                fma(
                                    temp.j14, v14,
                                    fma(
                                        temp.j15, v15,
                                        fma(
                                            temp.j20, v20,
                                            fma(
                                                temp.j21, v21,
                                                fma(
                                                    temp.j22, v22,
                                                    fma(
                                                        temp.j23, v23,
                                                        fma(
                                                            temp.j24, v24,
                                                            fma(
                                                                temp.j25, v25,
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
                )
            )

            constraintBlocks[lIdx] = l

            constraintBlocks[v1Idx + 0] += temp.ej10 * (l - t)
            constraintBlocks[v1Idx + 1] += temp.ej11 * (l - t)
            constraintBlocks[v1Idx + 2] += temp.ej12 * (l - t)
            constraintBlocks[v1Idx + 3] += temp.ej13 * (l - t)
            constraintBlocks[v1Idx + 4] += temp.ej14 * (l - t)
            constraintBlocks[v1Idx + 5] += temp.ej15 * (l - t)

            constraintBlocks[v2Idx + 0] += temp.ej20 * (l - t)
            constraintBlocks[v2Idx + 1] += temp.ej21 * (l - t)
            constraintBlocks[v2Idx + 2] += temp.ej22 * (l - t)
            constraintBlocks[v2Idx + 3] += temp.ej23 * (l - t)
            constraintBlocks[v2Idx + 4] += temp.ej24 * (l - t)
            constraintBlocks[v2Idx + 5] += temp.ej25 * (l - t)
        }
    }
}