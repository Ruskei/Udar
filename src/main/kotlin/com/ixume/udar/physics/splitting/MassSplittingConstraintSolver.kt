package com.ixume.udar.physics.splitting

import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.constraint.*
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldBuffer
import org.joml.Quaterniond
import org.joml.Vector3d
import java.lang.Math.fma
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import kotlin.math.*

class MassSplittingConstraintSolver(val constraintData: LocalConstraintData) {
    var dt = Udar.CONFIG.timeStep
    var bias = Udar.CONFIG.massSplittingConfig.bias
    var slop = Udar.CONFIG.massSplittingConfig.slop
    var friction = Udar.CONFIG.massSplittingConfig.friction
    var erp = Udar.CONFIG.massSplittingConfig.erp
    var posIterations = Udar.CONFIG.massSplittingConfig.posIterations
    fun effectiveERP(): Float {
        return 1f - (1.0 - erp.toDouble()).pow(1.0 / posIterations.toDouble()).toFloat()
    }

    var effectiveERP = effectiveERP()

    private var threads = Udar.CONFIG.massSplittingConfig.threads
    private var runnables = Array(threads) { SolverRunnable(this) }

    private var temp = ConstraintData()
    private var numPairsPerBody = IntArray(0)
    private var bodyAverages = FloatArray(0)


    private var executor = Executors.newFixedThreadPool(threads)

    private val _tempQ = Quaterniond()
    private val _tempV = Vector3d()

    fun setup() {
        dt = Udar.CONFIG.timeStep
        bias = Udar.CONFIG.massSplittingConfig.bias
        slop = Udar.CONFIG.massSplittingConfig.slop
        friction = Udar.CONFIG.massSplittingConfig.friction
        erp = Udar.CONFIG.massSplittingConfig.erp
        posIterations = Udar.CONFIG.massSplittingConfig.posIterations
        effectiveERP = effectiveERP()

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
            runnable.a2aCursor = 0
            runnable.a2sCursor = 0
            runnable.a2aNumBlocks = 0
            runnable.a2sNumBlocks = 0
            runnable.flatBodyData = constraintData.flatBodyData
            runnable.ensureSize(manifolds.size(), manifolds.numContacts, envManifolds.size(), envManifoldNumContacts)
        }

        for (i in 0..<manifolds.size()) {
            val thread = i % threads
            val runnable = runnables[thread]

            runnable.a2aNumBlocks++

            val b1Idx = manifolds.bodyAIdx(i)
            val b2Idx = manifolds.bodyBIdx(i)
            val b1 = constraintData.physicsWorld.activeBodies.fastGet(b1Idx)!!
            val b2 = constraintData.physicsWorld.activeBodies.fastGet(b2Idx)!!

            val numContacts = manifolds.numContacts(i)

            val c = runnable.a2aCursor

            runnable.a2aCursor = runnable.a2aNormalBlocks.add(
                cursor = c,
                temp = temp,
                body1Idx = b1Idx,
                body2Idx = b2Idx,

                manifoldIdx = i,
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

                val depth = manifolds.depth(i, idx)
                val lambda = manifolds.normalLambda(i, idx)

                addA2AContact(
                    b1Idx = b1Idx,
                    b1 = b1,

                    b2Idx = b1Idx,
                    b2 = b2,

                    contactIdx = i,

                    nx = nx,
                    ny = ny,
                    nz = nz,

                    error = depth,
                    lambda = lambda,

                    data = data,
                    idx = idx,
                    manifolds = manifolds,
                    numPairsPerBody = numPairsPerBody
                )
            }

            runnable.a2aT1Blocks.add(
                cursor = c,
                temp = temp,
                body1Idx = b1Idx,
                body2Idx = b2Idx,

                manifoldIdx = i,
                numContacts = numContacts,
            ) { data, idx ->
                val nx = manifolds.normX(i, idx)
                val ny = manifolds.normY(i, idx)
                val nz = manifolds.normZ(i, idx)

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

                val depth = manifolds.depth(i, idx)
                val lambda = manifolds.t1Lambda(i, idx)

                addA2AContact(
                    b1Idx = b1Idx,
                    b1 = b1,

                    b2Idx = b1Idx,
                    b2 = b2,

                    contactIdx = i,

                    nx = t1x,
                    ny = t1y,
                    nz = t1z,

                    error = depth,
                    lambda = lambda,

                    data = data,
                    idx = idx,
                    manifolds = manifolds,
                    numPairsPerBody = numPairsPerBody
                )
            }

            runnable.a2aT2Blocks.add(
                cursor = c,
                temp = temp,
                body1Idx = b1Idx,
                body2Idx = b2Idx,

                manifoldIdx = i,
                numContacts = numContacts,
            ) { data, idx ->
                val nx = manifolds.normX(i, idx)
                val ny = manifolds.normY(i, idx)
                val nz = manifolds.normZ(i, idx)

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

                val depth = manifolds.depth(i, idx)
                val lambda = manifolds.t2Lambda(i, idx)

                addA2AContact(
                    b1Idx = b1Idx,
                    b1 = b1,

                    b2Idx = b1Idx,
                    b2 = b2,

                    contactIdx = i,

                    nx = t2x,
                    ny = t2y,
                    nz = t2z,

                    error = depth,
                    lambda = lambda,

                    data = data,
                    idx = idx,
                    manifolds = manifolds,
                    numPairsPerBody = numPairsPerBody
                )
            }
        }

        for (i in 0..<envManifolds.size()) {
            val thread = i % threads
            val runnable = runnables[thread]

            runnable.a2sNumBlocks++

            val bIdx = envManifolds.bodyIdx(i)
            val b = constraintData.physicsWorld.activeBodies.fastGet(bIdx)!!

            val numContacts = envManifolds.numContacts(i)

            val c = runnable.a2sCursor

            runnable.a2sCursor = runnable.a2sNormalBlocks.add(
                cursor = c,
                temp = temp,
                body1Idx = bIdx,
                body2Idx = -1,

                manifoldIdx = i,
                numContacts = numContacts,
            ) { data, idx ->
                val nx = envManifolds.normX(i, idx)
                val ny = envManifolds.normY(i, idx)
                val nz = envManifolds.normZ(i, idx)

                val depth = envManifolds.depth(i, idx)
                val lambda = envManifolds.normalLambda(i, idx)

                addA2SContact(
                    bIdx = bIdx,
                    b = b,
                    contactIdx = i,

                    nx = nx,
                    ny = ny,
                    nz = nz,

                    error = depth,
                    lambda = lambda,

                    data = data,
                    idx = idx,
                    envManifolds = envManifolds,
                    numPairsPerBody = numPairsPerBody
                )
            }

            runnable.a2sT1Blocks.add(
                cursor = c,
                temp = temp,
                body1Idx = bIdx,
                body2Idx = -1,

                manifoldIdx = i,
                numContacts = numContacts,
            ) { data, idx ->
                val nx = envManifolds.normX(i, idx)
                val ny = envManifolds.normY(i, idx)
                val nz = envManifolds.normZ(i, idx)

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

                val lambda = envManifolds.t1Lambda(i, idx)

                addA2SContact(
                    bIdx = bIdx,
                    b = b,
                    contactIdx = i,

                    nx = t1x,
                    ny = t1y,
                    nz = t1z,

                    error = 0f,
                    lambda = lambda,

                    data = data,
                    idx = idx,
                    envManifolds = envManifolds,
                    numPairsPerBody = numPairsPerBody
                )
            }

            runnable.a2sT2Blocks.add(
                cursor = c,
                temp = temp,
                body1Idx = bIdx,
                body2Idx = -1,

                manifoldIdx = i,
                numContacts = numContacts,
            ) { data, idx ->
                val nx = envManifolds.normX(i, idx)
                val ny = envManifolds.normY(i, idx)
                val nz = envManifolds.normZ(i, idx)

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

                val lambda = envManifolds.t2Lambda(i, idx)

                addA2SContact(
                    bIdx = bIdx,
                    b = b,
                    contactIdx = i,

                    nx = t2x,
                    ny = t2y,
                    nz = t2z,

                    error = 0f,
                    lambda = lambda,

                    data = data,
                    idx = idx,
                    envManifolds = envManifolds,
                    numPairsPerBody = numPairsPerBody
                )
            }
        }
    }

    fun warm() {
        val latch = CountDownLatch(threads)
        for (thread in 0..<threads) {
            executor.execute {
                runnables[thread].warm()

                latch.countDown()
            }
        }

        latch.await()

        average { runnable, averages ->
            sum(runnable.a2aNormalBlocks, runnable.a2aNumBlocks, averages, 3)
            sum(runnable.a2sNormalBlocks, runnable.a2sNumBlocks, averages, 3)

            sum(runnable.a2aT1Blocks, runnable.a2aNumBlocks, averages, 3)
            sum(runnable.a2sT1Blocks, runnable.a2sNumBlocks, averages, 3)

            sum(runnable.a2aT2Blocks, runnable.a2aNumBlocks, averages, 3)
            sum(runnable.a2sT2Blocks, runnable.a2sNumBlocks, averages, 3)
        }
    }

    fun solveNormals(iteration: Int) {
        val latch = CountDownLatch(threads)
        for (thread in 0..<threads) {
            executor.execute {
                runnables[thread].solveNormals(iteration)

                latch.countDown()
            }
        }

        latch.await()

        average { runnable, averages ->
            sum(runnable.a2aNormalBlocks, runnable.a2aNumBlocks, averages)
            sum(runnable.a2sNormalBlocks, runnable.a2sNumBlocks, averages)
        }
    }

    private inline fun average(mult: Float = 1f, block: (runnable: SolverRunnable, bodyAverages: FloatArray) -> Unit) {
        bodyAverages.fill(0f)

        for (runnable in runnables) {
            block(runnable, bodyAverages)

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

    private fun sum(blocks: ConstraintBlocks, num: Int, bodyAverages: FloatArray, m: Int = 1) {
        blocks.forEachBlock(num) { b1Idx, b2Idx, v10, v11, v12, v13, v14, v15, v20, v21, v22, v23, v24, v25, _, _, _ ->
            if (b1Idx != -1) {
                val n1 = numPairsPerBody[b1Idx]

                bodyAverages[b1Idx * 6 + 0] += v10 / n1 / m
                bodyAverages[b1Idx * 6 + 1] += v11 / n1 / m
                bodyAverages[b1Idx * 6 + 2] += v12 / n1 / m
                bodyAverages[b1Idx * 6 + 3] += v13 / n1 / m
                bodyAverages[b1Idx * 6 + 4] += v14 / n1 / m
                bodyAverages[b1Idx * 6 + 5] += v15 / n1 / m
            }

            if (b2Idx != -1) {
                val n2 = numPairsPerBody[b2Idx]

                bodyAverages[b2Idx * 6 + 0] += v20 / n2 / m
                bodyAverages[b2Idx * 6 + 1] += v21 / n2 / m
                bodyAverages[b2Idx * 6 + 2] += v22 / n2 / m
                bodyAverages[b2Idx * 6 + 3] += v23 / n2 / m
                bodyAverages[b2Idx * 6 + 4] += v24 / n2 / m
                bodyAverages[b2Idx * 6 + 5] += v25 / n2 / m
            }
        }
    }

    fun solveFriction() {
        val latch1 = CountDownLatch(threads)
        for (thread in 0..<threads) {
            executor.execute {
                runnables[thread].solveT1Friction()

                latch1.countDown()
            }
        }

        latch1.await()

        average { runnable, averages ->
            sum(runnable.a2aT1Blocks, runnable.a2aNumBlocks, averages)
            sum(runnable.a2sT1Blocks, runnable.a2sNumBlocks, averages)
        }

        val latch2 = CountDownLatch(threads)
        for (thread in 0..<threads) {
            executor.execute {
                runnables[thread].solveT2Friction()

                latch2.countDown()
            }
        }

        latch2.await()

        average { runnable, averages ->
            sum(runnable.a2aT2Blocks, runnable.a2aNumBlocks, averages)
            sum(runnable.a2sT2Blocks, runnable.a2sNumBlocks, averages)
        }
    }

    fun solvePositions() {
        /*
        dC/dt * dt = dC
        dC/dt * k = e
        JV = 0
        M^-1 * J * l
        p += dx
        
        C(x + dx) = 0
        C(x) + p dC/dx = 0
        
        x = X + R(q)r
        dx = do x r
        dC = (do x r).n
        dC/do = r x n
        
        (B_A + Q_A * R_A - B_B - Q_B * R_B).n = C
        take derivative of C with respect to B_A, Q_A, B_B, Q_B
        dC/dB_A = n
        dC/dB_B = -n
        for quaternions, we can say that a small rotation represented by a vector D_A, dP_A = dD_A x r, so a small change in D_A cross r is equal to a small change in the position. so we can say that with respect to D_A, dC/dD_A = r x n, dC/dD_B = -r x n,
        so
        
        dC/dx = [n, r x n, -n, -r x n]
        which, interestingly enough, is also exactly the same as jacobian of JV = 0 of dC/dt
        now however we can approximate the change needed with:
        C(x + dx) = 0
        (P_A - P_B).n + dC/dx . x = 0
        dC/dx . x = -(P_A - P_B).n
        right side is fixed; smallest x should be in direction of dC/dt, so let's say:
        x = l * M^-1 * dC/dx
        accounting for mass matrix containing inverse masses and inverse inertia matrices
        so dC/dx . (l * M^-1 * dC/dx) = -(P_A - P_B).n
        so finally l = -(P_A - P_B).n / ((dC/dx)^T M^-1 (dC/dx))
        */
        for (runnable in runnables) {
            runnable.a2aNormalBlocks.forEachConstraint(
                temp = temp,
                numBlocks = runnable.a2aNumBlocks,
            ) { b1Idx, b2Idx, data, _, _ ->
                solvePosition(b1Idx, b2Idx, data)
            }

            runnable.a2sNormalBlocks.forEachConstraint(
                temp = temp,
                numBlocks = runnable.a2sNumBlocks,
            ) { b1Idx, b2Idx, data, _, _ ->
                solvePosition(b1Idx, b2Idx, data)
            }
        }
    }

    private fun solvePosition(
        b1Idx: Int,
        b2Idx: Int,

        data: ConstraintData,
    ) {
        var b1: ActiveBody? = null

        var p1x = data.r1x
        var p1y = data.r1y
        var p1z = data.r1z

        if (b1Idx != -1) {
            b1 = constraintData.physicsWorld.activeBodies.fastGet(b1Idx)!!
            p1x += b1.pos.x.toFloat()
            p1y += b1.pos.y.toFloat()
            p1z += b1.pos.z.toFloat()
        }

        var b2: ActiveBody? = null

        var p2x = data.r2x
        var p2y = data.r2y
        var p2z = data.r2z

        if (b2Idx != -1) {
            b2 = constraintData.physicsWorld.activeBodies.fastGet(b2Idx)!!
            p2x += b2.pos.x.toFloat()
            p2y += b2.pos.y.toFloat()
            p2z += b2.pos.z.toFloat()
        }

        val nx = data.nx
        val ny = data.ny
        val nz = data.nz

        val e = max(0f, -fma(p1x - p2x, nx, fma(p1y - p2y, ny, (p1z - p2z) * nz)))
        if (e < slop) {
            return
        }

        val dl = (e * data.den * effectiveERP).toDouble()

        if (b1 != null) {
            b1.pos.add(
                data.ej10 * dl,
                data.ej11 * dl,
                data.ej12 * dl,
            )

            _tempQ.set(b1.q).conjugate().transform(
                data.ej13 * dl,
                data.ej14 * dl,
                data.ej15 * dl,
                _tempV
            )

            _tempQ.set(b1.q).mul(_tempV.x, _tempV.y, _tempV.z, 0.0).mul(0.5)

            b1.q.add(_tempQ).normalize()
        }

        if (b2 != null) {
            b2.pos.add(
                data.ej20 * dl,
                data.ej21 * dl,
                data.ej22 * dl,
            )

            _tempQ.set(b2.q).conjugate().transform(
                data.ej23 * dl,
                data.ej24 * dl,
                data.ej25 * dl,
                _tempV
            )

            _tempQ.set(b2.q).mul(_tempV.x, _tempV.y, _tempV.z, 0.0).mul(0.5)

            b2.q.add(_tempQ).normalize()
        }
    }

    fun after() {
        heatUp()
    }

    fun reportLambdas() {
        val iterations = Udar.CONFIG.collision.normalIterations
        if (iterations <= 0) return
        val normalDeltaLambdas = FloatArray(iterations)
        val n =
            constraintData.physicsWorld.manifoldBuffer.numContacts + constraintData.physicsWorld.envManifoldBuffer.numContacts.get()
        if (n == 0) return
        for (runnable in runnables) {
            val ls = runnable.normalDeltaLambdas
            for (i in 0..<ls.size) {
                normalDeltaLambdas[i] += ls[i] / n
            }
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

    private fun heatUp() {
        val carryover = Udar.CONFIG.collision.lambdaCarryover

        val a2aManifolds = constraintData.physicsWorld.manifoldBuffer
        val a2aPrevData = constraintData.physicsWorld.prevContactData
        val a2aPrevMap = constraintData.physicsWorld.prevContactMap

        val a2sManifolds = constraintData.physicsWorld.envManifoldBuffer
        val a2sPrevData = constraintData.physicsWorld.prevEnvContactData
        val a2sPrevMap = constraintData.physicsWorld.prevEnvContactMap

        for (runnable in runnables) {
            runnable.a2aNormalBlocks.forEachBlock(
                numBlocks = runnable.a2aNumBlocks,
            ) { b1Idx, b2Idx, _, _, _, _, _, _, _, _, _, _, _, _, i, numConstraints, cursor ->
                val manifoldID = a2aManifolds.manifoldID(i)
                val idx = a2aPrevData.start(numConstraints)

                var j = 0
                while (j < numConstraints) {
                    val nl = runnable.a2aNormalBlocks[cursor + j * CONSTRAINT_DATA_SIZE + LAMBDA_OFFSET] * carryover
                    val t1l = runnable.a2aT1Blocks[cursor + j * CONSTRAINT_DATA_SIZE + LAMBDA_OFFSET] * carryover
                    val t2l = runnable.a2aT2Blocks[cursor + j * CONSTRAINT_DATA_SIZE + LAMBDA_OFFSET] * carryover

                    val x1 = a2aManifolds.pointAX(i, j)
                    val y1 = a2aManifolds.pointAY(i, j)
                    val z1 = a2aManifolds.pointAZ(i, j)

                    val x2 = a2aManifolds.pointBX(i, j)
                    val y2 = a2aManifolds.pointBY(i, j)
                    val z2 = a2aManifolds.pointBZ(i, j)

                    a2aPrevData.ls.add(x1)
                    a2aPrevData.ls.add(y1)
                    a2aPrevData.ls.add(z1)

                    a2aPrevData.ls.add(x2)
                    a2aPrevData.ls.add(y2)
                    a2aPrevData.ls.add(z2)

                    a2aPrevData.ls.add(nl)
                    a2aPrevData.ls.add(t1l)
                    a2aPrevData.ls.add(t2l)

                    j++
                }

                a2aPrevMap.put(manifoldID, idx)
            }

            runnable.a2sNormalBlocks.forEachBlock(
                numBlocks = runnable.a2sNumBlocks,
            ) { b1Idx, b2Idx, _, _, _, _, _, _, _, _, _, _, _, _, i, numConstraints, cursor ->
                val manifoldID = a2sManifolds.manifoldID(i)
                val idx = a2sPrevData.start(numConstraints)

                var j = 0
                while (j < numConstraints) {
                    val nl = runnable.a2sNormalBlocks[cursor + j * CONSTRAINT_DATA_SIZE + LAMBDA_OFFSET] * carryover
                    val t1l = runnable.a2sT1Blocks[cursor + j * CONSTRAINT_DATA_SIZE + LAMBDA_OFFSET] * carryover
                    val t2l = runnable.a2sT2Blocks[cursor + j * CONSTRAINT_DATA_SIZE + LAMBDA_OFFSET] * carryover

                    val x = a2sManifolds.pointAX(i, j)
                    val y = a2sManifolds.pointAY(i, j)
                    val z = a2sManifolds.pointAZ(i, j)

                    a2sPrevData.ls.add(x)
                    a2sPrevData.ls.add(y)
                    a2sPrevData.ls.add(z)

                    a2sPrevData.ls.add(nl)
                    a2sPrevData.ls.add(t1l)
                    a2sPrevData.ls.add(t2l)

                    j++
                }

                a2sPrevMap.put(manifoldID, idx)
            }
        }
    }
}


private const val BASE_CONSTRAINT_BLOCK_SIZE = 16
private const val CONSTRAINT_DATA_SIZE = 36

private const val LAMBDA_OFFSET = 24

/*
for contact block solving we need:
  beta_1 (idx) (int)
  beta_2 (idx) (int)
  D_alpha_1 (velocity of body 1) (1x6)
  D_alpha_2 (velocity of body 2) (1x6)
  manifoldIdx (int)
  
  numContacts (int)
    J_alpha_beta_1 (jacobian) (1x6)
    J_alpha_beta_2 (jacobian) (1x6)
    M_1^-1*J_alpha_beta_1 (jacobian) (1x6)
    M_2^-1*J_alpha_beta_2 (jacobian) (1x6)
    lambda (float)
    den (float)
    error (float)
    ra (1x3)
    rb (1x3)
    norm (1x3)
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

        manifoldIdx: Int,
        numConstraints: Int,
        cursor: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()
        val manifoldIdx = this[p++].toRawBits()

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

        val numConstraints = this[p++].toRawBits()

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

            manifoldIdx,
            numConstraints,
            p
        )

        p += numConstraints * CONSTRAINT_DATA_SIZE
        i++
    }
}

private inline fun ConstraintBlocks.updateEachConstraint(
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
        p++ // manifold idx

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

            temp.r1x = this[p++]
            temp.r1y = this[p++]
            temp.r1z = this[p++]

            temp.r2x = this[p++]
            temp.r2y = this[p++]
            temp.r2z = this[p++]

            temp.nx = this[p++]
            temp.ny = this[p++]
            temp.nz = this[p++]

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

private inline fun ConstraintBlocks.forEachConstraint(
    numBlocks: Int,
    temp: ConstraintData,
    block: (
        b1Idx: Int,
        b2Idx: Int,

        data: ConstraintData,

        manifoldIdx: Int,
        constraintIdx: Int,
    ) -> Unit,
) {
    var p = 0
    var i = 0
    while (i < numBlocks) {
        val b1Idx = this[p++].toRawBits()
        val b2Idx = this[p++].toRawBits()
        val manifoldIdx = this[p++].toRawBits()

        p += 12

        val numConstraints = this[p++].toRawBits()
        for (j in 0..<numConstraints) {
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

            temp.r1x = this[p++]
            temp.r1y = this[p++]
            temp.r1z = this[p++]

            temp.r2x = this[p++]
            temp.r2y = this[p++]
            temp.r2z = this[p++]

            temp.nx = this[p++]
            temp.ny = this[p++]
            temp.nz = this[p++]

            block(
                b1Idx,
                b2Idx,

                temp,

                manifoldIdx, j
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
    manifoldIdx: Int,
    constraintDataSupplier: (data: ConstraintData, constraintIdx: Int) -> Unit,
): Int {
    var c = cursor

    this[c++] = Float.fromBits(body1Idx)
    this[c++] = Float.fromBits(body2Idx)
    this[c++] = Float.fromBits(manifoldIdx)

    //space for velocities
    fill(0f, c, c + 12)
    c += 12

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

        this[c++] = temp.r1x
        this[c++] = temp.r1y
        this[c++] = temp.r1z

        this[c++] = temp.r2x
        this[c++] = temp.r2y
        this[c++] = temp.r2z

        this[c++] = temp.nx
        this[c++] = temp.ny
        this[c++] = temp.nz
    }

    return c
}

private fun addA2SContact(
    bIdx: Int,
    b: ActiveBody,
    contactIdx: Int,

    nx: Float,
    ny: Float,
    nz: Float,

    error: Float,
    lambda: Float,

    data: ConstraintData,
    idx: Int,

    envManifolds: A2SManifoldBuffer,
    numPairsPerBody: IntArray,
) {
    val n = numPairsPerBody[bIdx]
    val im = b.inverseMass.toFloat()
    val ii = b.inverseInertia

    val pax = envManifolds.pointAX(contactIdx, idx)
    val pay = envManifolds.pointAY(contactIdx, idx)
    val paz = envManifolds.pointAZ(contactIdx, idx)

    val rax = pax - envManifolds.bodyX(contactIdx)
    val ray = pay - envManifolds.bodyY(contactIdx)
    val raz = paz - envManifolds.bodyZ(contactIdx)

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

    data.lambda = lambda
    data.den =
        1f / (
                (data.ej10 * data.j10 + data.ej11 * data.j11 + data.ej12 * data.j12 +
                 data.j13 * data.ej13 + data.j14 * data.ej14 + data.j15 * data.ej15) * n
             )

    data.error = error

    data.r1x = rax
    data.r1y = ray
    data.r1z = raz

    val depth = envManifolds.depth(contactIdx, idx)

    data.r2x = pax + depth * nx
    data.r2y = pay + depth * ny
    data.r2z = paz + depth * nz

    data.nx = nx
    data.ny = ny
    data.nz = nz
}

private fun addA2AContact(
    b1Idx: Int,
    b1: ActiveBody,
    b2Idx: Int,
    b2: ActiveBody,

    contactIdx: Int,

    nx: Float,
    ny: Float,
    nz: Float,

    error: Float,
    lambda: Float,

    data: ConstraintData,
    idx: Int,

    manifolds: A2AManifoldArray,
    numPairsPerBody: IntArray,
) {
    val n1 = numPairsPerBody[b1Idx]
    val n2 = numPairsPerBody[b2Idx]

    val im1 = b1.inverseMass.toFloat()
    val im2 = b2.inverseMass.toFloat()

    val ii1 = b1.inverseInertia
    val ii2 = b2.inverseInertia

    val rax = manifolds.pointAX(contactIdx, idx) - manifolds.bodyAX(contactIdx)
    val ray = manifolds.pointAY(contactIdx, idx) - manifolds.bodyAY(contactIdx)
    val raz = manifolds.pointAZ(contactIdx, idx) - manifolds.bodyAZ(contactIdx)

    val rbx = manifolds.pointBX(contactIdx, idx) - manifolds.bodyBX(contactIdx)
    val rby = manifolds.pointBY(contactIdx, idx) - manifolds.bodyBY(contactIdx)
    val rbz = manifolds.pointBZ(contactIdx, idx) - manifolds.bodyBZ(contactIdx)

    data.j10 = nx
    data.j11 = ny
    data.j12 = nz
    data.j13 = fma(ray, nz, -raz * ny)
    data.j14 = fma(raz, nx, -rax * nz)
    data.j15 = fma(rax, ny, -ray * nx)

    data.j20 = -nx
    data.j21 = -ny
    data.j22 = -nz
    data.j23 = -fma(rby, nz, -rbz * ny)
    data.j24 = -fma(rbz, nx, -rbx * nz)
    data.j25 = -fma(rbx, ny, -rby * nx)

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

    data.lambda = lambda
    data.den =
        1f / (
                (data.ej10 * data.j10 + data.ej11 * data.j11 + data.ej12 * data.j12 +
                 data.j13 * data.ej13 + data.j14 * data.ej14 + data.j15 * data.ej15) * n1 +
                (data.ej20 * data.j20 + data.ej21 * data.j21 + data.ej22 * data.j22 +
                 data.j23 * data.ej23 + data.j24 * data.ej24 + data.j25 * data.ej25) * n2
             )

    data.error = error

    data.r1x = rax
    data.r1y = ray
    data.r1z = raz

    data.r2x = rbx
    data.r2y = rby
    data.r2z = rbz

    data.nx = nx
    data.ny = ny
    data.nz = nz
}

private data class ConstraintData(
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

    var r1x: Float = 0f,
    var r1y: Float = 0f,
    var r1z: Float = 0f,

    var r2x: Float = 0f,
    var r2y: Float = 0f,
    var r2z: Float = 0f,

    var nx: Float = 0f,
    var ny: Float = 0f,
    var nz: Float = 0f,
)

private class SolverRunnable(val solver: MassSplittingConstraintSolver) {
    var a2aCursor = 0
    var a2aNumBlocks = 0
    var a2aNormalBlocks: ConstraintBlocks = ConstraintBlocks(0)
    var a2aT1Blocks: ConstraintBlocks = ConstraintBlocks(0)
    var a2aT2Blocks: ConstraintBlocks = ConstraintBlocks(0)

    var a2sCursor = 0
    var a2sNumBlocks = 0
    var a2sNormalBlocks: ConstraintBlocks = ConstraintBlocks(0)
    var a2sT1Blocks: ConstraintBlocks = ConstraintBlocks(0)
    var a2sT2Blocks: ConstraintBlocks = ConstraintBlocks(0)

    var flatBodyData: FloatArray = FloatArray(0)

    var normalDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)

    private val temp = ConstraintData()

    fun ensureSize(
        a2aNumManifolds: Int, a2aNumContacts: Int,
        a2sNumManifolds: Int, a2sNumContacts: Int,
    ) {
        if (a2aNormalBlocks.size < a2aNumManifolds * BASE_CONSTRAINT_BLOCK_SIZE + a2aNumContacts * CONSTRAINT_DATA_SIZE) {
            val newSize = max(
                a2aNumManifolds * BASE_CONSTRAINT_BLOCK_SIZE + a2aNumContacts * CONSTRAINT_DATA_SIZE,
                a2aNormalBlocks.size * 3 / 2
            )
            a2aNormalBlocks = ConstraintBlocks(newSize)
            a2aT1Blocks = ConstraintBlocks(newSize)
            a2aT2Blocks = ConstraintBlocks(newSize)
        }

        if (a2sNormalBlocks.size < a2sNumManifolds * BASE_CONSTRAINT_BLOCK_SIZE + a2sNumContacts * CONSTRAINT_DATA_SIZE) {
            val newSize = max(
                a2sNumManifolds * BASE_CONSTRAINT_BLOCK_SIZE + a2sNumContacts * CONSTRAINT_DATA_SIZE,
                a2sNormalBlocks.size * 3 / 2
            )
            a2sNormalBlocks = ConstraintBlocks(newSize)
            a2sT1Blocks = ConstraintBlocks(newSize)
            a2sT2Blocks = ConstraintBlocks(newSize)
        }

        if (normalDeltaLambdas.size != Udar.CONFIG.collision.normalIterations) {
            normalDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)
        } else {
            normalDeltaLambdas.fill(0f)
        }
    }

    private inline fun solve(
        blocks: ConstraintBlocks,
        num: Int,

        lambdaTransform: (l: Float, lIdx: Int) -> Float,
    ) {
        blocks.updateEachConstraint(
            flatBodyData = flatBodyData,
            temp = temp,
            numBlocks = num,
        ) { b1Idx, b2Idx, v1Idx, v2Idx, data, lIdx, numConstraints ->
            val t = blocks[lIdx]

            val v10 = blocks[v1Idx + 0]
            val v11 = blocks[v1Idx + 1]
            val v12 = blocks[v1Idx + 2]
            val v13 = blocks[v1Idx + 3]
            val v14 = blocks[v1Idx + 4]
            val v15 = blocks[v1Idx + 5]

            val v20 = blocks[v2Idx + 0]
            val v21 = blocks[v2Idx + 1]
            val v22 = blocks[v2Idx + 2]
            val v23 = blocks[v2Idx + 3]
            val v24 = blocks[v2Idx + 4]
            val v25 = blocks[v2Idx + 5]

            val bias = (-solver.bias * temp.error / solver.dt).toFloat()

            val l = lambdaTransform(
                t - temp.den * fma(
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
                ),
                lIdx
            )

            blocks[lIdx] = l

            blocks[v1Idx + 0] += temp.ej10 * (l - t)
            blocks[v1Idx + 1] += temp.ej11 * (l - t)
            blocks[v1Idx + 2] += temp.ej12 * (l - t)
            blocks[v1Idx + 3] += temp.ej13 * (l - t)
            blocks[v1Idx + 4] += temp.ej14 * (l - t)
            blocks[v1Idx + 5] += temp.ej15 * (l - t)

            blocks[v2Idx + 0] += temp.ej20 * (l - t)
            blocks[v2Idx + 1] += temp.ej21 * (l - t)
            blocks[v2Idx + 2] += temp.ej22 * (l - t)
            blocks[v2Idx + 3] += temp.ej23 * (l - t)
            blocks[v2Idx + 4] += temp.ej24 * (l - t)
            blocks[v2Idx + 5] += temp.ej25 * (l - t)
        }
    }

    private fun warm(
        blocks: ConstraintBlocks,
        num: Int,
    ) {
        blocks.updateEachConstraint(
            flatBodyData = flatBodyData,
            temp = temp,
            numBlocks = num,
        ) { b1Idx, b2Idx, v1Idx, v2Idx, data, lIdx, numConstraints ->
            val l = blocks[lIdx]

            blocks[v1Idx + 0] += temp.ej10 * l
            blocks[v1Idx + 1] += temp.ej11 * l
            blocks[v1Idx + 2] += temp.ej12 * l
            blocks[v1Idx + 3] += temp.ej13 * l
            blocks[v1Idx + 4] += temp.ej14 * l
            blocks[v1Idx + 5] += temp.ej15 * l

            blocks[v2Idx + 0] += temp.ej20 * l
            blocks[v2Idx + 1] += temp.ej21 * l
            blocks[v2Idx + 2] += temp.ej22 * l
            blocks[v2Idx + 3] += temp.ej23 * l
            blocks[v2Idx + 4] += temp.ej24 * l
            blocks[v2Idx + 5] += temp.ej25 * l
        }
    }

    fun warm() {
        warm(a2aNormalBlocks, a2aNumBlocks)
        warm(a2sNormalBlocks, a2sNumBlocks)

        warm(a2aT1Blocks, a2aNumBlocks)
        warm(a2sT1Blocks, a2sNumBlocks)

        warm(a2aT2Blocks, a2aNumBlocks)
        warm(a2sT2Blocks, a2sNumBlocks)
    }

    fun solveNormals(iteration: Int) {
        solve(a2aNormalBlocks, a2aNumBlocks) { l, lIdx ->
            val r = max(0f, l)
            normalDeltaLambdas[iteration - 1] += abs(r - a2aNormalBlocks[lIdx])
            r
        }

        solve(a2sNormalBlocks, a2sNumBlocks) { l, lIdx ->
            val r = max(0f, l)
            normalDeltaLambdas[iteration - 1] += abs(r - a2sNormalBlocks[lIdx])
            r
        }
    }

    fun solveT1Friction() {
        val friction = solver.friction

        solve(a2aT1Blocks, a2aNumBlocks) { l, i ->
            val nl = a2aNormalBlocks[i]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }

        solve(a2sT1Blocks, a2sNumBlocks) { l, i ->
            val nl = a2sNormalBlocks[i]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }
    }

    fun solveT2Friction() {
        val friction = solver.friction

        solve(a2aT2Blocks, a2aNumBlocks) { l, i ->
            val nl = a2aNormalBlocks[i]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }

        solve(a2sT2Blocks, a2sNumBlocks) { l, i ->
            val nl = a2sNormalBlocks[i]
            val lower = -nl * friction
            val upper = nl * friction

            min(upper, max(lower, l))
        }
    }
}