package com.ixume.udar.physics.splitting

import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.constraint.*
import org.joml.Quaterniond
import org.joml.Vector3d
import java.lang.Math.fma
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import kotlin.math.*

class MassSplittingConstraintSolver(val constraintData: LocalConstraintData) {
    var relaxation = Udar.CONFIG.massSplittingConfig.relaxation
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

    private var a2aTemp = A2AConstraintData()
    private var a2sTemp = A2SConstraintData()
    private var numPairsPerBody = IntArray(0)
    private var bodyAverages = FloatArray(0)

    private var setupExecutor = Executors.newFixedThreadPool(2)
    private var solvingExecutor = Executors.newFixedThreadPool(threads)

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
            solvingExecutor.shutdown()
            solvingExecutor = Executors.newFixedThreadPool(threads)
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

        val latch = CountDownLatch(2)

        setupExecutor.execute {
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
                    temp = a2aTemp,
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
                    temp = a2aTemp,
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
                    temp = a2aTemp,
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

            latch.countDown()
        }

        setupExecutor.execute {
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
                    temp = a2sTemp,
                    body1Idx = bIdx,

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
                    temp = a2sTemp,
                    body1Idx = bIdx,

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
                    temp = a2sTemp,
                    body1Idx = bIdx,

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

            latch.countDown()
        }

        latch.await()
    }

    fun warm() {
        val latch = CountDownLatch(threads)
        for (thread in 0..<threads) {
            solvingExecutor.execute {
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
            solvingExecutor.execute {
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

    private inline fun average(block: (runnable: SolverRunnable, bodyAverages: FloatArray) -> Unit) {
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

    private fun sum(blocks: A2AConstraintBlocks, num: Int, bodyAverages: FloatArray, m: Int = 1) {
        blocks.forEachBlock(num) { b1Idx, b2Idx, v10, v11, v12, v13, v14, v15, v20, v21, v22, v23, v24, v25, _, _, _ ->
            val n1 = numPairsPerBody[b1Idx]
            bodyAverages[b1Idx * 6 + 0] += v10 / n1 / m
            bodyAverages[b1Idx * 6 + 1] += v11 / n1 / m
            bodyAverages[b1Idx * 6 + 2] += v12 / n1 / m
            bodyAverages[b1Idx * 6 + 3] += v13 / n1 / m
            bodyAverages[b1Idx * 6 + 4] += v14 / n1 / m
            bodyAverages[b1Idx * 6 + 5] += v15 / n1 / m

            val n2 = numPairsPerBody[b2Idx]
            bodyAverages[b2Idx * 6 + 0] += v20 / n2 / m
            bodyAverages[b2Idx * 6 + 1] += v21 / n2 / m
            bodyAverages[b2Idx * 6 + 2] += v22 / n2 / m
            bodyAverages[b2Idx * 6 + 3] += v23 / n2 / m
            bodyAverages[b2Idx * 6 + 4] += v24 / n2 / m
            bodyAverages[b2Idx * 6 + 5] += v25 / n2 / m
        }
    }

    private fun sum(blocks: A2SConstraintBlocks, num: Int, bodyAverages: FloatArray, m: Int = 1) {
        blocks.forEachBlock(num) { b1Idx, v10, v11, v12, v13, v14, v15, _, _, _ ->
            val n1 = numPairsPerBody[b1Idx]
            bodyAverages[b1Idx * 6 + 0] += v10 / n1 / m
            bodyAverages[b1Idx * 6 + 1] += v11 / n1 / m
            bodyAverages[b1Idx * 6 + 2] += v12 / n1 / m
            bodyAverages[b1Idx * 6 + 3] += v13 / n1 / m
            bodyAverages[b1Idx * 6 + 4] += v14 / n1 / m
            bodyAverages[b1Idx * 6 + 5] += v15 / n1 / m
        }
    }

    fun solveFriction() {
        val latch1 = CountDownLatch(threads)
        for (thread in 0..<threads) {
            solvingExecutor.execute {
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
            solvingExecutor.execute {
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
                temp = a2aTemp,
                numBlocks = runnable.a2aNumBlocks,
            ) { b1Idx, b2Idx, data, _, _ ->
                solvePosition(b1Idx, b2Idx, data)
            }

            runnable.a2sNormalBlocks.forEachConstraint(
                temp = a2sTemp,
                numBlocks = runnable.a2sNumBlocks,
            ) { b1Idx, data, _, _ ->
                solvePosition(b1Idx, data)
            }
        }
    }

    private fun solvePosition(
        b1Idx: Int,
        b2Idx: Int,

        data: A2AConstraintData,
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

    private fun solvePosition(
        b1Idx: Int,

        data: A2SConstraintData,
    ) {
        val b1 = constraintData.physicsWorld.activeBodies.fastGet(b1Idx)!!

        val p1x = data.r1x + b1.pos.x.toFloat()
        val p1y = data.r1y + b1.pos.y.toFloat()
        val p1z = data.r1z + b1.pos.z.toFloat()

        val p2x = data.r2x
        val p2y = data.r2y
        val p2z = data.r2z

        val nx = data.nx
        val ny = data.ny
        val nz = data.nz

        val e = max(0f, -fma(p1x - p2x, nx, fma(p1y - p2y, ny, (p1z - p2z) * nz)))
        if (e < slop) {
            return
        }

        val dl = (e * data.den * effectiveERP).toDouble()

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

    /*
    when we update a manifold, we don't want to immediately lose the impulse data from old contacts since a manifold has multiple contacts
    so we should only rewrite the constraints there that are close to the existing points
     */
    fun heatUp() {
        val carryover = Udar.CONFIG.collision.lambdaCarryover

        val a2aNumManifolds = constraintData.physicsWorld.manifoldBuffer.size()
        val a2aManifolds = constraintData.physicsWorld.manifoldBuffer
        val a2aPrevData = constraintData.physicsWorld.prevContactData
        a2aPrevData.ensureCapacity(a2aNumManifolds)
        val a2aPrevMap = constraintData.physicsWorld.prevContactMap

        val a2sNumManifolds = constraintData.physicsWorld.envManifoldBuffer.size()
        val a2sManifolds = constraintData.physicsWorld.envManifoldBuffer
        val a2sPrevData = constraintData.physicsWorld.prevEnvContactData
        a2sPrevData.ensureCapacity(a2sNumManifolds)
        val a2sPrevMap = constraintData.physicsWorld.prevEnvContactMap

        val stay = Udar.CONFIG.massSplittingConfig.lambdaStay

        for (runnable in runnables) {
            runnable.a2aNormalBlocks.forEachBlock(
                numBlocks = runnable.a2aNumBlocks,
            ) { b1Idx, b2Idx, _, _, _, _, _, _, _, _, _, _, _, _, i, numConstraints, cursor ->
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
                        runnable.a2aNormalBlocks[cursor + j * A2A_CONSTRAINT_DATA_SIZE + A2A_LAMBDA_OFFSET] * carryover
                    val t1l =
                        runnable.a2aT1Blocks[cursor + j * A2A_CONSTRAINT_DATA_SIZE + A2A_LAMBDA_OFFSET] * carryover
                    val t2l =
                        runnable.a2aT2Blocks[cursor + j * A2A_CONSTRAINT_DATA_SIZE + A2A_LAMBDA_OFFSET] * carryover

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

            runnable.a2sNormalBlocks.forEachBlock(
                numBlocks = runnable.a2sNumBlocks,
            ) { b1Idx, _, _, _, _, _, _, i, numConstraints, cursor ->
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
                        runnable.a2sNormalBlocks[cursor + j * A2S_CONSTRAINT_DATA_SIZE + A2S_LAMBDA_OFFSET] * carryover
                    val t1l =
                        runnable.a2sT1Blocks[cursor + j * A2S_CONSTRAINT_DATA_SIZE + A2S_LAMBDA_OFFSET] * carryover
                    val t2l =
                        runnable.a2sT2Blocks[cursor + j * A2S_CONSTRAINT_DATA_SIZE + A2S_LAMBDA_OFFSET] * carryover

                    val x = a2sManifolds.pointAX(i, j)
                    val y = a2sManifolds.pointAY(i, j)
                    val z = a2sManifolds.pointAZ(i, j)

                    c = a2sPrevData.add(
                        cursor = c,
                        x = x,
                        y = y,
                        z = z,
                        nl = nl,
                        t1l = t1l,
                        t2l = t2l,
                    )

                    j++
                }
            }
        }
    }
}


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

private class SolverRunnable(val solver: MassSplittingConstraintSolver) {
    var a2aCursor = 0
    var a2aNumBlocks = 0
    var a2aNormalBlocks: A2AConstraintBlocks = A2AConstraintBlocks(0)
    var a2aT1Blocks: A2AConstraintBlocks = A2AConstraintBlocks(0)
    var a2aT2Blocks: A2AConstraintBlocks = A2AConstraintBlocks(0)

    var a2sCursor = 0
    var a2sNumBlocks = 0
    var a2sNormalBlocks: A2SConstraintBlocks = A2SConstraintBlocks(0)
    var a2sT1Blocks: A2SConstraintBlocks = A2SConstraintBlocks(0)
    var a2sT2Blocks: A2SConstraintBlocks = A2SConstraintBlocks(0)

    var flatBodyData: FloatArray = FloatArray(0)

    var normalDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)

    private val a2aTemp = A2AConstraintData()
    private val a2sTemp = A2SConstraintData()

    fun ensureSize(
        a2aNumManifolds: Int, a2aNumContacts: Int,
        a2sNumManifolds: Int, a2sNumContacts: Int,
    ) {
        if (a2aNormalBlocks.value.size < a2aNumManifolds * A2A_BASE_CONSTRAINT_BLOCK_SIZE + a2aNumContacts * A2A_CONSTRAINT_DATA_SIZE) {
            val newSize = max(
                a2aNumManifolds * A2A_BASE_CONSTRAINT_BLOCK_SIZE + a2aNumContacts * A2A_CONSTRAINT_DATA_SIZE,
                a2aNormalBlocks.value.size * 3 / 2
            )
            a2aNormalBlocks = A2AConstraintBlocks(newSize)
            a2aT1Blocks = A2AConstraintBlocks(newSize)
            a2aT2Blocks = A2AConstraintBlocks(newSize)
        }

        if (a2sNormalBlocks.value.size < a2sNumManifolds * A2S_BASE_CONSTRAINT_BLOCK_SIZE + a2sNumContacts * A2S_CONSTRAINT_DATA_SIZE) {
            val newSize = max(
                a2sNumManifolds * A2S_BASE_CONSTRAINT_BLOCK_SIZE + a2sNumContacts * A2S_CONSTRAINT_DATA_SIZE,
                a2sNormalBlocks.value.size * 3 / 2
            )
            a2sNormalBlocks = A2SConstraintBlocks(newSize)
            a2sT1Blocks = A2SConstraintBlocks(newSize)
            a2sT2Blocks = A2SConstraintBlocks(newSize)
        }

        if (normalDeltaLambdas.size != Udar.CONFIG.collision.normalIterations) {
            normalDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)
        } else {
            normalDeltaLambdas.fill(0f)
        }
    }

    private inline fun solve(
        blocks: A2AConstraintBlocks,
        num: Int,

        lambdaTransform: (l: Float, lIdx: Int) -> Float,
    ) {
        blocks.updateEachConstraint(
            flatBodyData = flatBodyData,
            temp = a2aTemp,
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

            val bias = (-solver.bias * max(0f, a2aTemp.error - solver.slop) / solver.dt).toFloat()

            val l = lambdaTransform(
                t - a2aTemp.den * fma(
                    a2aTemp.j10, v10,
                    fma(
                        a2aTemp.j11, v11,
                        fma(
                            a2aTemp.j12, v12,
                            fma(
                                a2aTemp.j13, v13,
                                fma(
                                    a2aTemp.j14, v14,
                                    fma(
                                        a2aTemp.j15, v15,
                                        fma(
                                            a2aTemp.j20, v20,
                                            fma(
                                                a2aTemp.j21, v21,
                                                fma(
                                                    a2aTemp.j22, v22,
                                                    fma(
                                                        a2aTemp.j23, v23,
                                                        fma(
                                                            a2aTemp.j24, v24,
                                                            fma(
                                                                a2aTemp.j25, v25,
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

            blocks[v1Idx + 0] += a2aTemp.ej10 * (l - t) * solver.relaxation
            blocks[v1Idx + 1] += a2aTemp.ej11 * (l - t) * solver.relaxation
            blocks[v1Idx + 2] += a2aTemp.ej12 * (l - t) * solver.relaxation
            blocks[v1Idx + 3] += a2aTemp.ej13 * (l - t) * solver.relaxation
            blocks[v1Idx + 4] += a2aTemp.ej14 * (l - t) * solver.relaxation
            blocks[v1Idx + 5] += a2aTemp.ej15 * (l - t) * solver.relaxation

            blocks[v2Idx + 0] += a2aTemp.ej20 * (l - t) * solver.relaxation
            blocks[v2Idx + 1] += a2aTemp.ej21 * (l - t) * solver.relaxation
            blocks[v2Idx + 2] += a2aTemp.ej22 * (l - t) * solver.relaxation
            blocks[v2Idx + 3] += a2aTemp.ej23 * (l - t) * solver.relaxation
            blocks[v2Idx + 4] += a2aTemp.ej24 * (l - t) * solver.relaxation
            blocks[v2Idx + 5] += a2aTemp.ej25 * (l - t) * solver.relaxation
        }
    }

    private inline fun solve(
        blocks: A2SConstraintBlocks,
        num: Int,

        lambdaTransform: (l: Float, lIdx: Int) -> Float,
    ) {
        blocks.updateEachConstraint(
            flatBodyData = flatBodyData,
            temp = a2sTemp,
            numBlocks = num,
        ) { b1Idx, v1Idx, data, lIdx, numConstraints ->
            val t = blocks[lIdx]

            val v10 = blocks[v1Idx + 0]
            val v11 = blocks[v1Idx + 1]
            val v12 = blocks[v1Idx + 2]
            val v13 = blocks[v1Idx + 3]
            val v14 = blocks[v1Idx + 4]
            val v15 = blocks[v1Idx + 5]

            val bias = (-solver.bias * max(0f, a2sTemp.error - solver.slop) / solver.dt).toFloat()

            val l = lambdaTransform(
                t - a2sTemp.den * fma(
                    a2sTemp.j10, v10,
                    fma(
                        a2sTemp.j11, v11,
                        fma(
                            a2sTemp.j12, v12,
                            fma(
                                a2sTemp.j13, v13,
                                fma(
                                    a2sTemp.j14, v14,
                                    fma(
                                        a2sTemp.j15, v15,
                                        bias
                                    )
                                )
                            )
                        )
                    )
                ),
                lIdx
            )

            blocks[lIdx] = l

            blocks[v1Idx + 0] += a2sTemp.ej10 * (l - t) * solver.relaxation
            blocks[v1Idx + 1] += a2sTemp.ej11 * (l - t) * solver.relaxation
            blocks[v1Idx + 2] += a2sTemp.ej12 * (l - t) * solver.relaxation
            blocks[v1Idx + 3] += a2sTemp.ej13 * (l - t) * solver.relaxation
            blocks[v1Idx + 4] += a2sTemp.ej14 * (l - t) * solver.relaxation
            blocks[v1Idx + 5] += a2sTemp.ej15 * (l - t) * solver.relaxation
        }
    }

    private fun warm(
        blocks: A2AConstraintBlocks,
        num: Int,
    ) {
        blocks.updateEachConstraint(
            flatBodyData = flatBodyData,
            temp = a2aTemp,
            numBlocks = num,
        ) { b1Idx, b2Idx, v1Idx, v2Idx, data, lIdx, numConstraints ->
            val l = blocks[lIdx] * solver.relaxation

            blocks[v1Idx + 0] += a2aTemp.ej10 * l
            blocks[v1Idx + 1] += a2aTemp.ej11 * l
            blocks[v1Idx + 2] += a2aTemp.ej12 * l
            blocks[v1Idx + 3] += a2aTemp.ej13 * l
            blocks[v1Idx + 4] += a2aTemp.ej14 * l
            blocks[v1Idx + 5] += a2aTemp.ej15 * l

            blocks[v2Idx + 0] += a2aTemp.ej20 * l
            blocks[v2Idx + 1] += a2aTemp.ej21 * l
            blocks[v2Idx + 2] += a2aTemp.ej22 * l
            blocks[v2Idx + 3] += a2aTemp.ej23 * l
            blocks[v2Idx + 4] += a2aTemp.ej24 * l
            blocks[v2Idx + 5] += a2aTemp.ej25 * l
        }
    }

    private fun warm(
        blocks: A2SConstraintBlocks,
        num: Int,
    ) {
        blocks.updateEachConstraint(
            flatBodyData = flatBodyData,
            temp = a2sTemp,
            numBlocks = num,
        ) { b1Idx, v1Idx, data, lIdx, numConstraints ->
            val l = blocks[lIdx] * solver.relaxation

            blocks[v1Idx + 0] += a2sTemp.ej10 * l
            blocks[v1Idx + 1] += a2sTemp.ej11 * l
            blocks[v1Idx + 2] += a2sTemp.ej12 * l
            blocks[v1Idx + 3] += a2sTemp.ej13 * l
            blocks[v1Idx + 4] += a2sTemp.ej14 * l
            blocks[v1Idx + 5] += a2sTemp.ej15 * l
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
