package com.ixume.udar

import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.WorldMeshesManager
import com.ixume.udar.collisiondetection.pool.MathPool
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.dynamicaabb.FlattenedBodyAABBTree
import com.ixume.udar.physics.EntityUpdater
import com.ixume.udar.physics.EnvPhaseCallable
import com.ixume.udar.physics.NarrowPhaseCallable
import com.ixume.udar.physics.StatusUpdater
import com.ixume.udar.physics.constraint.ConstraintSolverManager
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldBuffer
import com.ixume.udar.physics.contact.a2a.manifold.A2APrevManifoldData
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldBuffer
import com.ixume.udar.physics.contact.a2s.manifold.A2SPrevManifoldData
import com.ixume.udar.testing.PhysicsWorldTestDebugData
import com.ixume.udar.testing.debugConnect
import com.ixume.udar.testing.listener.PlayerInteractListener
import it.unimi.dsi.fastutil.ints.Int2ObjectMaps
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap
import it.unimi.dsi.fastutil.ints.IntArrayList
import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap
import org.bukkit.*
import org.joml.Vector3d
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.roundToInt
import kotlin.system.measureNanoTime

class PhysicsWorld(
    val world: World,
) {
    private val bodiesToAdd = AtomicList<ActiveBody>()
    private val bodiesToRemove = AtomicList<ActiveBody>()

    val activeBodies = ActiveBodiesCollection()

    private val runningID = AtomicLong(0)
    fun createID(): Long {
        return runningID.andIncrement
    }

    fun bodiesSnapshot(): List<ActiveBody> {
        return activeBodies.activeBodies()
    }

    var numPossibleContacts = 0

    val manifoldBuffer = A2AManifoldBuffer(4)

    val prevContactMap = Long2IntOpenHashMap()
    val prevContactData = A2APrevManifoldData()

    val envManifoldBuffer = A2SManifoldBuffer(8)

    val prevEnvContactMap = Long2IntOpenHashMap()
    val prevEnvContactData = A2SPrevManifoldData()

    init {
        prevEnvContactMap.defaultReturnValue(-1)
        prevContactMap.defaultReturnValue(-1)
    }

    val environmentBody = EnvironmentBody(this)

    private var time = 0
    private var physicsTime = 0
    val frozen = AtomicBoolean(false)
    val untilCollision = AtomicBoolean(false)
    val steps = AtomicInteger(0)

    val debugData = PhysicsWorldTestDebugData()

    private val entityUpdater = EntityUpdater(this)
    private val statusUpdater = StatusUpdater(this)

    val worldMeshesManager = WorldMeshesManager(this)

    private val simTask = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, Runnable { tick() }, 1, 1)
    private val entityTask = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { entityUpdater.tick() }, 2, 2)

    private val busy = AtomicBoolean(false)

    private val NARROWPHASE_PROCESSORS = 5//Runtime.getRuntime().availableProcessors()
    private val ENV_PROCESSORS = 5
    val mathPool = MathPool(this, NARROWPHASE_PROCESSORS)
    private val executor = Executors.newFixedThreadPool(NARROWPHASE_PROCESSORS)
    private val narrowPhaseCallables = Array(NARROWPHASE_PROCESSORS) { NarrowPhaseCallable(this) }

    private val envPhaseCallables = Array(ENV_PROCESSORS) { EnvPhaseCallable(this) }

    private val constraintSolverManager = ConstraintSolverManager(this)

    private val dataInterval = 400

    private var rollingAverage = 0.0
    private var rollingBroadAverage = 0.0
    private var rollingNarrowAverage = 0.0
    private var rollingEnvAverage = 0.0
    private var rollingParallelContactAverage = 0.0
    private var rollingStepAverage = 0.0

    val bodyAABBTree = FlattenedBodyAABBTree(this, 0)

    fun registerBody(body: ActiveBody) {
        bodiesToAdd += body
    }

    fun registerBodies(bodies: Collection<ActiveBody>) {
        bodiesToAdd += bodies
    }

    fun removeBody(body: ActiveBody) {
        body.onKill()
        body.dead.set(true)
        bodiesToRemove += body
    }

    fun removeBodies(bodies: Collection<ActiveBody>) {
        for (body in bodies) {
            body.dead.set(true)
            body.onKill()
        }

        bodiesToRemove += bodies
    }

    private fun tick() {
        PlayerInteractListener.tick(world)
        if (Udar.CONFIG.debug.bbs) {
            bodyAABBTree.visualize(world)
        }

        time++
        repeat((0.05 / Udar.CONFIG.timeStep).roundToInt()) {
            var doTick = true
            if (frozen.get()) {
                if (!untilCollision.get() && steps.decrementAndGet() < 0) doTick = false
            }

            if (doTick) {
                if (!busy.compareAndSet(false, true)) return@repeat

                val startTime = System.nanoTime()

                processToAdd()
                processToRemove()

                physicsTime++

                debugData.reset()
                manifoldBuffer.clear()
                envManifoldBuffer.clear()

                val bodiesSnapshot = activeBodies.activeBodies()

                statusUpdater.updateBodies(bodiesSnapshot)
                constraintSolverManager.prepare()

                val startBroadTime = System.nanoTime()
                val activePairs = broadPhase(bodiesSnapshot)

                val endBroadTime = System.nanoTime()

                val startNarrowTime = System.nanoTime()

                if (activePairs != null) {
                    check(activePairs.size == NARROWPHASE_PROCESSORS)

                    val latch = CountDownLatch(NARROWPHASE_PROCESSORS)

                    for (proc in 0..<NARROWPHASE_PROCESSORS) {
                        val callable = narrowPhaseCallables[proc]

                        callable.ps = activePairs[proc]

                        executor.execute {
                            callable.run()

                            latch.countDown()
                        }
                    }

                    latch.await()
                }

                val endNarrowTime = System.nanoTime()

//                println("TICK")
                val envDuration = measureNanoTime {
                    if (bodiesSnapshot.isNotEmpty()) {
                        val per = (bodiesSnapshot.size / ENV_PROCESSORS).coerceAtLeast(1)
                        val latch = CountDownLatch(ENV_PROCESSORS)
//                        println("ENV PHASE")
                        var last = 0
                        for (proc in 0..<ENV_PROCESSORS) {
                            val callable = envPhaseCallables[proc]
                            callable.start = last
                            last = if (proc == ENV_PROCESSORS - 1) {
                                bodiesSnapshot.size
                            } else {
                                (per * (proc + 1)).coerceAtMost(bodiesSnapshot.size)
                            }

                            callable.end = last

                            callable.bodiesSnapshot = bodiesSnapshot

                            executor.execute {
                                callable.run()

                                latch.countDown()
                            }
                        }

                        latch.await()
                    }
                }

                for (body in bodiesSnapshot) {
                    if (!body.isChild && body.awake.get() && body.hasGravity) body.velocity.add(
                        Vector3d(Udar.CONFIG.gravity).mul(
                            Udar.CONFIG.timeStep
                        )
                    )
                }

                val parallelConstraintDuration = measureNanoTime {
                    constraintSolverManager.solve()
                }

                val stepDuration = measureNanoTime {
                    for (body in bodiesSnapshot) {
                        if (body.isChild) continue

                        if (body.awake.get()) {
                            body.step()
                        }
                    }
                }

                if (untilCollision.get() && (!manifoldBuffer.isEmpty() || !envManifoldBuffer.isEmpty())) {
                    untilCollision.set(false)
                    frozen.set(true)
                }

                val duration = (System.nanoTime() - startTime).toDouble()
                rollingAverage += duration / dataInterval.toDouble()

                val narrowDuration = (endNarrowTime - startNarrowTime).toDouble()
                rollingNarrowAverage += narrowDuration / dataInterval.toDouble()

                val broadDuration = (endBroadTime - startBroadTime).toDouble()
                rollingBroadAverage += broadDuration / dataInterval.toDouble()

                rollingEnvAverage += envDuration / dataInterval.toDouble()

                rollingStepAverage += stepDuration / dataInterval.toDouble()

                rollingParallelContactAverage += parallelConstraintDuration / dataInterval.toDouble()

                if (physicsTime % dataInterval == 0) {
                    if (Udar.CONFIG.debug.timings) {
                        println("Total takes ${rollingAverage / 1_000.0}us on average")
                        println("  - Broadphase takes ${rollingBroadAverage / 1_000.0}us (${rollingBroadAverage / rollingAverage * 100.0}%) on average")
                        println("  - Narrowphase takes ${rollingNarrowAverage / 1_000.0}us (${rollingNarrowAverage / rollingAverage * 100.0}%) on average")
                        println("  - Env takes ${rollingEnvAverage / 1_000.0}us (${rollingEnvAverage / rollingAverage * 100.0}%) on average")
                        println("  - Parallel Constraint takes ${rollingParallelContactAverage / 1_000.0}us (${rollingParallelContactAverage / rollingAverage * 100.0}%) on average")
                        println("  - Step takes ${rollingStepAverage / 1_000.0}us (${rollingStepAverage / rollingAverage * 100.0}%) on average")
                        println("ACCOUNTED FOR ${(rollingNarrowAverage + rollingBroadAverage + rollingParallelContactAverage + rollingEnvAverage + rollingStepAverage) / rollingAverage * 100.0}%")
                    }
                    rollingAverage = 0.0
                    rollingNarrowAverage = 0.0
                    rollingBroadAverage = 0.0
                    rollingParallelContactAverage = 0.0
                    rollingEnvAverage = 0.0
                    rollingStepAverage = 0.0
                }

                busy.set(false)
            }

            if (Udar.CONFIG.debug.normals > 0) {
                val s = manifoldBuffer.size()
                var i = 0
                while (i < s) {
                    val num = manifoldBuffer.numContacts(i)
                    var k = 0

                    while (k < num) {
                        world.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                world,
                                manifoldBuffer.pointAX(i, k).toDouble(),
                                manifoldBuffer.pointAY(i, k).toDouble(),
                                manifoldBuffer.pointAZ(i, k).toDouble()
                            ),
                            1,
                            Particle.DustOptions(Color.RED, 0.3f),
                        )

                        world.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                world,
                                manifoldBuffer.pointBX(i, k).toDouble(),
                                manifoldBuffer.pointBY(i, k).toDouble(),
                                manifoldBuffer.pointBZ(i, k).toDouble()
                            ),
                            1,
                            Particle.DustOptions(Color.WHITE, 0.3f),
                        )

                        world.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                world,
                                manifoldBuffer.pointAX(i, k).toDouble(),
                                manifoldBuffer.pointAY(i, k).toDouble(),
                                manifoldBuffer.pointAZ(i, k).toDouble()
                            ),
                            1,
                            Particle.DustOptions(Color.BLUE, 0.3f),
                        )

                        world.debugConnect(
                            start = Vector3d(
                                manifoldBuffer.pointAX(i, k).toDouble(),
                                manifoldBuffer.pointAY(i, k).toDouble(),
                                manifoldBuffer.pointAZ(i, k).toDouble()
                            ),
                            end = Vector3d(
                                manifoldBuffer.pointAX(i, k).toDouble(),
                                manifoldBuffer.pointAY(i, k).toDouble(),
                                manifoldBuffer.pointAZ(i, k).toDouble()
                            ).add(
                                manifoldBuffer.normX(i, k).toDouble() * 0.77,
                                manifoldBuffer.normY(i, k).toDouble() * 0.77,
                                manifoldBuffer.normZ(i, k).toDouble() * 0.77,
                            ),
                            options = Particle.DustOptions(Color.PURPLE, 0.25f),
                        )

                        k++
                    }

                    i++
                }

                val es = envManifoldBuffer.size()
                var j = 0
                while (j < es) {
                    val num = envManifoldBuffer.numContacts(j)
                    var k = 0
                    while (k < num) {
                        world.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                world,
                                envManifoldBuffer.pointAX(j, k).toDouble(),
                                envManifoldBuffer.pointAY(j, k).toDouble(),
                                envManifoldBuffer.pointAZ(j, k).toDouble()
                            ),
                            1,
                            Particle.DustOptions(Color.RED, 0.3f),
                        )

                        world.debugConnect(
                            start = Vector3d(
                                envManifoldBuffer.pointAX(j, k).toDouble(),
                                envManifoldBuffer.pointAY(j, k).toDouble(),
                                envManifoldBuffer.pointAZ(j, k).toDouble()
                            ),
                            end = Vector3d(
                                envManifoldBuffer.pointAX(j, k).toDouble(),
                                envManifoldBuffer.pointAY(j, k).toDouble(),
                                envManifoldBuffer.pointAZ(j, k).toDouble()
                            ).add(
                                envManifoldBuffer.normX(j, k).toDouble(),
                                envManifoldBuffer.normY(j, k).toDouble(),
                                envManifoldBuffer.normZ(j, k).toDouble()
                            ),
                            options = Particle.DustOptions(Color.BLUE, 0.25f),
                        )

                        k++
                    }

                    j++
                }
            }
        }
    }

    fun updateBB(body: ActiveBody, tight: AABB) {
        body.fatBB = bodyAABBTree.update(body.fatBB, tight, body.id)
    }

    private fun processToAdd() {
        val ss = bodiesToAdd.getAndClear()
        for (body in ss) {
            activeBodies.add(body)
            if (body is Composite) {
                for (body in body.parts) {
                    activeBodies.add(body)
                }
            }

            body.update()
        }
    }

    private fun processToRemove() {
        val ss = bodiesToRemove.getAndClear()
        for (body in ss) {
            kill(body)
        }
    }

    private val _broadCollisions = Int2ObjectOpenHashMap<IntArrayList>().apply { defaultReturnValue(null) }
    private val _groupedBroadCollisions =
        Array<Int2ObjectOpenHashMap<IntArrayList>>(NARROWPHASE_PROCESSORS) { Int2ObjectOpenHashMap() }

    private fun broadPhase(bodies: List<ActiveBody>): Array<Int2ObjectOpenHashMap<IntArrayList>>? {
        numPossibleContacts = 0
        if (bodies.size <= 1) return null

        _broadCollisions.forEach { it.value.clear() }
        _groupedBroadCollisions.forEach { it.clear() }

        bodyAABBTree.constructCollisions(_broadCollisions)
        val iterator = Int2ObjectMaps.fastIterator(_broadCollisions)

        while (iterator.hasNext()) {
            val en = iterator.next()
            if (en.value.isEmpty) continue
            numPossibleContacts += en.value.size
            _groupedBroadCollisions.minBy { it.size }.put(en.intKey, en.value)
        }

        return _groupedBroadCollisions
    }

    fun clear() {
        removeBodies(activeBodies.activeBodies())
    }

    private fun kill(obj: ActiveBody) {
        if (activeBodies.remove(obj.uuid) != null) {
            bodyAABBTree.remove(obj.fatBB)
        }
        if (obj is Composite) {
            for (body in obj.parts) {
                activeBodies.remove(body.uuid)
            }
        }
    }

    fun kill() {
        clear()

        simTask.cancel()
        entityTask.cancel()
        executor.shutdown()

        worldMeshesManager.kill()
    }
}