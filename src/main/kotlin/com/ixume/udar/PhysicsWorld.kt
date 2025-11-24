package com.ixume.udar

import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.WorldMeshesManager
import com.ixume.udar.collisiondetection.envphase.EnvPhaseHandler
import com.ixume.udar.collisiondetection.narrowphase.NarrowPhaseHandler
import com.ixume.udar.collisiondetection.pool.MathPool
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.dynamicaabb.FlattenedBodyAABBTree
import com.ixume.udar.physics.EntityUpdater
import com.ixume.udar.physics.StatusUpdater
import com.ixume.udar.physics.angular.AngularConstraintManager
import com.ixume.udar.physics.constraint.ConstraintSolverManager
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import com.ixume.udar.physics.contact.a2a.manifold.A2APrevManifoldData
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldBuffer
import com.ixume.udar.physics.contact.a2s.manifold.A2SPrevManifoldData
import com.ixume.udar.physics.sphericaljoint.SphericalJointConstraintManager
import com.ixume.udar.testing.PhysicsWorldTestDebugData
import com.ixume.udar.testing.debugConnect
import com.ixume.udar.testing.listener.PlayerInteractListener
import it.unimi.dsi.fastutil.ints.Int2ObjectMaps
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap
import it.unimi.dsi.fastutil.ints.IntArrayList
import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap
import org.bukkit.*
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.roundToInt
import kotlin.system.measureNanoTime
import kotlin.time.DurationUnit
import kotlin.time.toDuration

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

    val manifoldBuffer = A2AManifoldArray(4)

    val prevContactMap = Long2IntOpenHashMap()
    val prevContactData = A2APrevManifoldData(8)

    val envManifoldBuffer = A2SManifoldBuffer(8)

    val prevEnvContactMap = Long2IntOpenHashMap()
    val prevEnvContactData = A2SPrevManifoldData(8)

    val sphericalJointConstraints = SphericalJointConstraintManager(this)
    val angularConstraints = AngularConstraintManager(this)

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

    private val busy = AtomicBoolean(false)

    val mathPool = MathPool(this)
    private val narrowPhaseHandler = NarrowPhaseHandler(this)
    private val envPhaseHandler = EnvPhaseHandler(this)

    private val constraintSolverManager = ConstraintSolverManager(this)

    private var rollingAverage = 0.0
    private var rollingBroadAverage = 0.0
    private var rollingNarrowAverage = 0.0
    private var rollingEnvAverage = 0.0
    private var rollingParallelContactAverage = 0.0
    private var rollingStepAverage = 0.0

    val bodyAABBTree = FlattenedBodyAABBTree(this, 0)

    private val entityTask = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { entityUpdater.tick() }, 1, 1)
    val worldMeshesManager = WorldMeshesManager(this, Udar.CONFIG.worldDiffingProcessors, Udar.CONFIG.meshingProcessors)

    private val entityUpdater = EntityUpdater(this)
    private val statusUpdater = StatusUpdater(this)

    private val simTask = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, Runnable {
        val t = measureNanoTime {
            tick()
        }

//        if (Udar.CONFIG.debug.timings) {
//            println("Tick took ${t.toDuration(DurationUnit.NANOSECONDS)}!")
//        }
    }, 1, 1)

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

    private var rollingSubtickTime = 1_000_000 // ns

    private fun tick() {
        PlayerInteractListener.tick(world)
        if (Udar.CONFIG.debug.bbs) {
            bodyAABBTree.visualize(world)
        }

        val tickStartTime = System.nanoTime()

        time++
        repeat((TICK_DURATION / Udar.CONFIG.timeStep).roundToInt()) {
            var doTick = true
            if (frozen.get()) {
                if (!untilCollision.get() && steps.decrementAndGet() < 0) doTick = false
            }

            if (doTick) {
                if (!busy.compareAndSet(false, true)) return@repeat

                val subtickStartTime = System.nanoTime()

                processToAdd()
                processToRemove()
                sphericalJointConstraints.tick()
                angularConstraints.tick()

                physicsTime++

                debugData.reset()
                manifoldBuffer.clear()
                envManifoldBuffer.clear()

                val bodiesSnapshot = activeBodies.activeBodies()

                statusUpdater.updateBodies(bodiesSnapshot)

                val startBroadTime = System.nanoTime()
                val activePairs = broadPhase(bodiesSnapshot)

                val endBroadTime = System.nanoTime()

                val startNarrowTime = System.nanoTime()

                if (activePairs != null) {
                    narrowPhaseHandler.process(activePairs)
                }

                val endNarrowTime = System.nanoTime()

//                println("TICK")
                val envDuration = measureNanoTime {
                    if (bodiesSnapshot.isNotEmpty()) {
                        envPhaseHandler.process(bodiesSnapshot)
                    }
                }

                for (body in bodiesSnapshot) {
                    if (!body.isChild && body.awake.get() && body.hasGravity) body.velocity.add(
                        Udar.CONFIG.gravity.x * Udar.CONFIG.timeStep,
                        Udar.CONFIG.gravity.y * Udar.CONFIG.timeStep,
                        Udar.CONFIG.gravity.z * Udar.CONFIG.timeStep,
                    )
                }

                val parallelConstraintDuration = measureNanoTime {
                    constraintSolverManager.solve()
                }

                val stepDuration = measureNanoTime {
                    for (body in bodiesSnapshot) {
                        if (body.isChild) continue
                        if (body.pos.y < -64) {
                            Bukkit.getScheduler().runTask(Udar.INSTANCE, Runnable {
                                removeBody(body)
                            })

                            continue
                        }

                        if (body.awake.get()) {
                            body.step()
                        }
                    }
                }
                
                prevContactData.tick(prevContactMap)
                prevEnvContactData.tick(prevEnvContactMap)

                if (untilCollision.get() && (!manifoldBuffer.isEmpty() || !envManifoldBuffer.isEmpty())) {
                    untilCollision.set(false)
                    frozen.set(true)
                }

                val dataInterval = Udar.CONFIG.debug.timingsSimReportInterval

                val duration = (System.nanoTime() - subtickStartTime).toDouble()
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
                        println("Subtick takes ${rollingAverage.toDuration(DurationUnit.NANOSECONDS)} on average")
                        println("  - Rolling: ${rollingSubtickTime.toDuration(DurationUnit.NANOSECONDS)}")
                        println("  - Broadphase takes ${rollingBroadAverage.toDuration(DurationUnit.NANOSECONDS)} (${rollingBroadAverage / rollingAverage * 100.0}%) on average")
                        println("  - Narrowphase takes ${rollingNarrowAverage.toDuration(DurationUnit.NANOSECONDS)} (${rollingNarrowAverage / rollingAverage * 100.0}%) on average")
                        println("  - Env takes ${rollingEnvAverage.toDuration(DurationUnit.NANOSECONDS)} (${rollingEnvAverage / rollingAverage * 100.0}%) on average")
                        println("  - Parallel Constraint takes ${rollingParallelContactAverage.toDuration(DurationUnit.NANOSECONDS)} (${rollingParallelContactAverage / rollingAverage * 100.0}%) on average")
                        println("  - Step takes ${rollingStepAverage.toDuration(DurationUnit.NANOSECONDS)} (${rollingStepAverage / rollingAverage * 100.0}%) on average")
                        println("ACCOUNTED FOR ${(rollingNarrowAverage + rollingBroadAverage + rollingParallelContactAverage + rollingEnvAverage + rollingStepAverage) / rollingAverage * 100.0}%")
                    }
                    rollingAverage = 0.0
                    rollingNarrowAverage = 0.0
                    rollingBroadAverage = 0.0
                    rollingParallelContactAverage = 0.0
                    rollingEnvAverage = 0.0
                    rollingStepAverage = 0.0
                }

                val current = System.nanoTime()
                val subtickDur = current - subtickStartTime
                rollingSubtickTime =
                    ((rollingSubtickTime * SUBTICK_DUR_ROLLOVER) + (subtickDur * (1.0 - SUBTICK_DUR_ROLLOVER))).toInt()

                if (current - tickStartTime + rollingSubtickTime > MAX_TIME_PER_TICK) {
                    busy.set(false)
                    return@tick
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
                            Particle.DUST,
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
                            Particle.DUST,
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
                            Particle.DUST,
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
                            Particle.DUST,
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

    private fun broadPhase(bodies: List<ActiveBody>): Array<Int2ObjectOpenHashMap<IntArrayList>>? {
        numPossibleContacts = 0
        if (bodies.size <= 1) return null

        _broadCollisions.forEach { it.value.clear() }
        narrowPhaseHandler._groupedBroadCollisions.forEach { it.clear() }

        bodyAABBTree.constructCollisions(_broadCollisions)
        val iterator = Int2ObjectMaps.fastIterator(_broadCollisions)

        while (iterator.hasNext()) {
            val en = iterator.next()
            if (en.value.isEmpty) continue
            numPossibleContacts += en.value.size
            narrowPhaseHandler._groupedBroadCollisions.minBy { it.size }.put(en.intKey, en.value)
        }

        return narrowPhaseHandler._groupedBroadCollisions
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
        narrowPhaseHandler.shutdown()
        envPhaseHandler.shutdown()

        worldMeshesManager.kill()
    }
}

private const val SUBTICK_DUR_ROLLOVER = 0.5
private const val MAX_TIME_PER_TICK = 48_000_000
private const val TICK_DURATION = 0.05