package com.ixume.udar

import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.broadphase.aabb.AABBTree
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.collisiondetection.pool.MathPool
import com.ixume.udar.physics.*
import com.ixume.udar.testing.PhysicsWorldTestDebugData
import kotlinx.coroutines.*
import org.bukkit.Bukkit
import org.bukkit.World
import org.joml.Vector3d
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.roundToInt
import kotlin.system.measureNanoTime

class PhysicsWorld(
    val world: World
) {
    private val bodiesToAdd = AtomicList<ActiveBody>()
    private val bodiesToRemove = AtomicList<ActiveBody>()

    private val activeBodies = AtomicList<ActiveBody>()
    fun bodiesSnapshot(): List<ActiveBody> {
        return activeBodies.get()
    }

    val contacts = ConcurrentLinkedQueue<IContact>()
    val meshes: MutableList<Mesh> = mutableListOf()

    private val environmentBody = EnvironmentBody(this)

    private var time = 0
    private var physicsTime = 0
    var frozen = false
    var untilCollision = false
    var steps = 0

    val debugData = PhysicsWorldTestDebugData()

    private val entityUpdater = EntityUpdater(this)
    private val statusUpdater = StatusUpdater(this)

    val realWorldHandler = RealWorldHandler(world)

    private val simTask = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, Runnable { tick() }, 1, 1)
    private val entityTask = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { entityUpdater.tick() }, 1, 1)

    private val busy = AtomicBoolean(false)

    private val processors = 3//Runtime.getRuntime().availableProcessors()
    private val mathPool = MathPool(processors)
    private val scope = CoroutineScope(Dispatchers.Default)

    private val dataInterval = 400

    private var rollingAverage = 0.0
    private var rollingNarrowAverage = 0.0
    private var rollingBroadAverage = 0.0
    private var rollingEnvAverage = 0.0
    private var rollingContactAverage = 0.0
    private var rollingStepAverage = 0.0

    private val aabbTree = AABBTree()

    fun registerBody(body: ActiveBody) {
        bodiesToAdd += body
    }

    fun registerBodies(bodies: Collection<ActiveBody>) {
        bodiesToAdd += bodies
    }

    fun removeBody(body: ActiveBody) {
        body.onKill()
        bodiesToRemove += body
    }

    fun removeBodies(bodies: Collection<ActiveBody>) {
        for (body in bodies) {
            body.onKill()
        }

        bodiesToRemove += bodies
    }

    private fun tick() {
//        aabbTree.visualize(world)
        time++
        repeat((0.05 / TIME_STEP).roundToInt()) {
            var doTick = true
            if (frozen) {
                if (!untilCollision && --steps < 0) doTick = false
            }

            if (doTick) {
                if (!busy.compareAndSet(false, true)) return@repeat

                val startTime = System.nanoTime()

                processToAdd()
                processToRemove()

                physicsTime++

                debugData.reset()

                contacts.clear()
                meshes.clear()

                val bodiesSnapshot = activeBodies.get()

                statusUpdater.updateBodies()

                val startBroadTime = System.nanoTime()
                val activePairs = broadPhase(bodiesSnapshot, processors)
                val endBroadTime = System.nanoTime()

                var job: Job? = null

                val startNarrowTime = System.nanoTime()

                if (activePairs != null) {
                    check(activePairs.size == processors)

                    job = scope.launch {
                        (0..<processors).forEach { i ->
                            val ps = activePairs[i]
                            launch { narrowPhase(ps) }
                        }
                    }
                }

                runBlocking {
                    job?.join()

                    val endNarrowTime = System.nanoTime()

                    val math = mathPool.get()

                    val startEnvTime = System.nanoTime()
                    try {
                        for (body in bodiesSnapshot) {
                            if (!body.awake.get()) {
                                continue
                            }

                            if (body.capableCollision(environmentBody) < 0) continue

                            val result = body.collides(environmentBody, math)

                            debugData.totalEnvironmentCollisionChecks++

                            if (result.isEmpty()) continue

                            debugData.environmentCollisions++

                            for (c in result) {
                                val ourContacts =
                                    body.previousContacts.filter { it.first is EnvironmentBody || it.second is EnvironmentBody }

                                for (ourContact in ourContacts) {
                                    if (ourContact.result.point.distance(c.result.point) < 1e-2) {
                                        c.lambdaSum += ourContact.lambdaSum * Udar.CONFIG.collision.lambdaCarryover

                                        break

                                    }
                                }

                                body.contacts += c
                                contacts += c
                            }
                        }

                    } finally {
                        mathPool.put(math)
                    }

                    val endEnvTime = System.nanoTime()

                    for (body in bodiesSnapshot) {
                        if (body.awake.get() && body.hasGravity) body.velocity.add(
                            Vector3d(Udar.CONFIG.gravity).mul(
                                TIME_STEP
                            )
                        )
                    }

                    val startContactTime = System.nanoTime()

                    ContactSolver.solve(contacts)

                    val endContactTime = System.nanoTime()

                    val startStepTime = System.nanoTime()

                    for (body in bodiesSnapshot) {
                        if (body.awake.get()) {
                            body.step()
                        }
                    }

                    val endStepTime = System.nanoTime()

                    if (untilCollision && contacts.isNotEmpty()) {
                        untilCollision = false
                        frozen = true
                    }

                    val duration = (System.nanoTime() - startTime).toDouble()
                    rollingAverage += duration / dataInterval.toDouble()

                    val narrowDuration = (endNarrowTime - startNarrowTime).toDouble()
                    rollingNarrowAverage += narrowDuration / dataInterval.toDouble()

                    val broadDuration = (endBroadTime - startBroadTime).toDouble()
                    rollingBroadAverage += broadDuration / dataInterval.toDouble()

                    val contactDuration = (endContactTime - startContactTime).toDouble()
                    rollingContactAverage += contactDuration / dataInterval.toDouble()

                    val envDuration = (endEnvTime - startEnvTime).toDouble()
                    rollingEnvAverage += envDuration / dataInterval.toDouble()

                    val stepDuration = (endStepTime - startStepTime).toDouble()
                    rollingStepAverage += stepDuration / dataInterval.toDouble()

                    if (physicsTime % dataInterval == 0) {
                        if (Udar.CONFIG.debug.timings) {
                            println("Total takes ${rollingAverage / 1_000.0}us on average")
                            println("  - Broadphase takes ${rollingBroadAverage / 1_000.0}us on average")
                            println("  - Narrowphase takes ${rollingNarrowAverage / 1_000.0}us on average")
                            println("  - Env takes ${rollingEnvAverage / 1_000.0}us on average")
                            println("  - Contact takes ${rollingContactAverage / 1_000.0}us on average")
                            println("  - Step takes ${rollingStepAverage / 1_000.0}us on average")
                            println("ACCOUNTED FOR ${(rollingNarrowAverage + rollingBroadAverage + rollingContactAverage + rollingEnvAverage + rollingStepAverage) / rollingAverage * 100.0}%")
                        }
                        rollingAverage = 0.0
                        rollingNarrowAverage = 0.0
                        rollingBroadAverage = 0.0
                        rollingContactAverage = 0.0
                        rollingEnvAverage = 0.0
                        rollingStepAverage = 0.0
                    }

                    busy.set(false)
                }
            }
        }
    }

    fun updateBB(body: ActiveBody) {
        body.fatBB.updateTree(aabbTree)
    }

    private fun processToAdd() {
//        val start = System.nanoTime()

        val ss = bodiesToAdd.getAndClear()
        for (body in ss) {
            body.fatBB.body = body
            body.update()
        }

        activeBodies += ss

//        val finish = System.nanoTime()
//
//        println("Insertions took ${(finish - start).toDouble() / 1_000.0}us")
    }

    private fun processToRemove() {
//        val start = System.nanoTime()

        val ss = bodiesToRemove.getAndClear()
        for (body in ss) {
            kill(body)
        }

//        val finish = System.nanoTime()
//
//        println("Deletions took ${(finish - start).toDouble() / 1_000.0}us")
    }

    private fun broadPhase(bodies: List<ActiveBody>, groups: Int): Array<MutableMap<ActiveBody, List<ActiveBody>>>? {
        require(groups > 0)
        if (bodies.size <= 1) return null

        val cs = aabbTree.collisions()

        val groups = Array(groups) { mutableMapOf<ActiveBody, List<ActiveBody>>() }

        for ((a, bs) in cs) {
            groups.minBy { it.size }[a] = bs
        }

        return groups
    }

    private fun narrowPhase(ps: Map<ActiveBody, List<ActiveBody>>) {
        val math = mathPool.get()

        try {
            for ((first, ls) in ps) {
                for (second in ls) {
                    val result: List<IContact>

                    val t = measureNanoTime {
                        result = first.collides(second, math)
                    }

                    if (result.isEmpty()) continue

                    debugData.pairCollisions++

                    first.awake.set(true)
                    second.awake.set(true)

                    if (Udar.CONFIG.debug.collisionTimes > 0) {
                        println("B-B COLLISION TOOK: ${t.toDouble() / 1_000_000.0} ms")
                    }

                    val ourContacts =
                        first.previousContacts.filter { it.second.id == second.id }

                    for (contact in result) {
                        first.contacts += contact

                        for (ourContact in ourContacts) {
                            if (ourContact.result.point.distance(contact.result.point) < 1.0) {
                                contact.lambdaSum = ourContact.lambdaSum * Udar.CONFIG.collision.lambdaCarryover
                                contact.t1Sum = ourContact.t1Sum * Udar.CONFIG.collision.lambdaCarryover
                                contact.t2Sum = ourContact.t2Sum * Udar.CONFIG.collision.lambdaCarryover

                                break
                            }
                        }

                        contacts += contact
                    }
                }
            }
        } finally {
            mathPool.put(math)
        }
    }

    fun clear() {
        removeBodies(activeBodies.get())
    }

    private fun kill(obj: ActiveBody) {
        println("REMOVED A BODY!")
        aabbTree.remove(obj.fatBB.node!!)
        activeBodies -= obj
    }

    fun kill() {
        clear()

        simTask.cancel()
        realWorldHandler.kill()
        entityTask.cancel()
        scope.cancel()
    }
}