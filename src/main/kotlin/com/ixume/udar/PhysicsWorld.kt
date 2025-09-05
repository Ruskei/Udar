package com.ixume.udar

import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.WorldMeshesManager
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.collisiondetection.pool.MathPool
import com.ixume.udar.dynamicaabb.AABBTree
import com.ixume.udar.physics.BodyIDMap
import com.ixume.udar.physics.EntityUpdater
import com.ixume.udar.physics.StatusUpdater
import com.ixume.udar.physics.constraint.ConstraintSolverManager
import com.ixume.udar.physics.contact.A2AContactBuffer
import com.ixume.udar.physics.contact.A2SContactBuffer
import com.ixume.udar.testing.PhysicsWorldTestDebugData
import com.ixume.udar.testing.debugConnect
import kotlinx.coroutines.*
import org.bukkit.*
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.roundToInt
import kotlin.system.measureNanoTime

class PhysicsWorld(
    val world: World,
) {
    private val bodiesToAdd = AtomicList<ActiveBody>()
    private val bodiesToRemove = AtomicList<ActiveBody>()

    val activeBodies = ActiveBodiesCollection()

    private val bodyIDMap = BodyIDMap(this)

    fun bodiesSnapshot(): List<ActiveBody> {
        return activeBodies.bodies()
    }

    var numPossibleContacts = 0
    val contactBuffer = A2AContactBuffer()
    val envContactBuffer = A2SContactBuffer()

    val meshes: MutableList<Mesh> = mutableListOf()

    private val environmentBody = EnvironmentBody(this)

    private var time = 0
    private var physicsTime = 0
    val frozen = AtomicBoolean(false)
    val untilCollision = AtomicBoolean(false)
    val steps = AtomicInteger(0)

    val debugData = PhysicsWorldTestDebugData()

    private val entityUpdater = EntityUpdater(this)
    private val statusUpdater = StatusUpdater(this)

    val worldMeshesManager = WorldMeshesManager(this)

    val runningContactID = AtomicInteger(0)

    private val simTask = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, Runnable { tick() }, 1, 1)
    private val entityTask = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { entityUpdater.tick() }, 1, 2)

    private val busy = AtomicBoolean(false)

    private val NARROWPHASE_PROCESSORS = 3//Runtime.getRuntime().availableProcessors()
    private val mathPool = MathPool(this, NARROWPHASE_PROCESSORS)
    private val scope = CoroutineScope(Dispatchers.Default)
    private val constraintSolverManager = ConstraintSolverManager(this)

    private val dataInterval = 400

    private var rollingAverage = 0.0
    private var rollingBroadAverage = 0.0
    private var rollingNarrowAverage = 0.0
    private var rollingEnvAverage = 0.0
    private var rollingParallelContactAverage = 0.0
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

                bodyIDMap.update()

                physicsTime++

                debugData.reset()
                runningContactID.set(0)
                contactBuffer.clear()
                envContactBuffer.clear()
                meshes.clear()

                val bodiesSnapshot = activeBodies.bodies()

                statusUpdater.updateBodies(bodiesSnapshot)
                constraintSolverManager.prepare()

                val startBroadTime = System.nanoTime()
                val activePairs = broadPhase(bodiesSnapshot, NARROWPHASE_PROCESSORS)

                val endBroadTime = System.nanoTime()

                var job: Job? = null

                val startNarrowTime = System.nanoTime()

                if (activePairs != null) {
                    check(activePairs.size == NARROWPHASE_PROCESSORS)

                    job = scope.launch {
                        (0..<NARROWPHASE_PROCESSORS).forEach { i ->
                            val ps = activePairs[i]

                            launch { narrowPhase(ps) }
                        }
                    }
                }

                runBlocking {
                    job?.join()
                }

                val endNarrowTime = System.nanoTime()

                val math = mathPool.get()

                val envDuration = measureNanoTime {
                    try {
                        for (body in bodiesSnapshot) {
                            if (!body.awake.get()) {
                                continue
                            }

                            if (body.capableCollision(environmentBody) < 0) continue

                            val collided = body.collides(environmentBody, math, envContactBuffer)

                            debugData.totalEnvironmentCollisionChecks++

                            if (!collided) continue

                            debugData.environmentCollisions++
                        }

                    } finally {
                        mathPool.put(math)
                    }
                }

                for (body in bodiesSnapshot) {
                    if (body.awake.get() && body.hasGravity) body.velocity.add(
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
                        if (body.awake.get()) {
                            body.step()
                        }
                    }
                }

                if (untilCollision.get() && (!contactBuffer.isEmpty() || !envContactBuffer.isEmpty())) {
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
                val s = contactBuffer.size()
                var i = 0
                while (i < s) {
                    world.spawnParticle(
                        Particle.REDSTONE,
                        Location(
                            world,
                            contactBuffer.pointAX(i).toDouble(),
                            contactBuffer.pointAY(i).toDouble(),
                            contactBuffer.pointAZ(i).toDouble()
                        ),
                        1,
                        Particle.DustOptions(Color.RED, 0.3f),
                    )

                    world.debugConnect(
                        start = Vector3d(
                            contactBuffer.pointAX(i).toDouble(),
                            contactBuffer.pointAY(i).toDouble(),
                            contactBuffer.pointAZ(i).toDouble()
                        ),
                        end = Vector3d(
                            contactBuffer.pointAX(i).toDouble(),
                            contactBuffer.pointAY(i).toDouble(),
                            contactBuffer.pointAZ(i).toDouble()
                        ).add(
                            contactBuffer.normX(i).toDouble(),
                            contactBuffer.normY(i).toDouble(),
                            contactBuffer.normZ(i).toDouble()
                        ),
                        options = Particle.DustOptions(Color.BLUE, 0.25f),
                    )
                    i++
                }

                val es = envContactBuffer.size()
                var j = 0
                while (j < es) {
                    world.spawnParticle(
                        Particle.REDSTONE,
                        Location(
                            world,
                            envContactBuffer.pointAX(j).toDouble(),
                            envContactBuffer.pointAY(j).toDouble(),
                            envContactBuffer.pointAZ(j).toDouble()
                        ),
                        1,
                        Particle.DustOptions(Color.RED, 0.3f),
                    )

                    world.debugConnect(
                        start = Vector3d(
                            envContactBuffer.pointAX(j).toDouble(),
                            envContactBuffer.pointAY(j).toDouble(),
                            envContactBuffer.pointAZ(j).toDouble()
                        ),
                        end = Vector3d(
                            envContactBuffer.pointAX(j).toDouble(),
                            envContactBuffer.pointAY(j).toDouble(),
                            envContactBuffer.pointAZ(j).toDouble()
                        ).add(
                            envContactBuffer.normX(j).toDouble(),
                            envContactBuffer.normY(j).toDouble(),
                            envContactBuffer.normZ(j).toDouble()
                        ),
                        options = Particle.DustOptions(Color.BLUE, 0.25f),
                    )

                    j++
                }
            }
        }
    }

    fun updateBB(body: ActiveBody) {
        body.fatBB.updateTree(aabbTree)
    }

    private fun processToAdd() {
        val ss = bodiesToAdd.getAndClear()
        for (body in ss) {
            body.fatBB.body = body
            body.update()

            activeBodies.add(body)
        }
    }

    private fun processToRemove() {
        val ss = bodiesToRemove.getAndClear()
        for (body in ss) {
            kill(body)
        }
    }

    private fun broadPhase(bodies: List<ActiveBody>, groups: Int): Array<MutableMap<ActiveBody, List<ActiveBody>>>? {
        require(groups > 0)
        numPossibleContacts = 0
        if (bodies.size <= 1) return null

        val cs = aabbTree.collisions()

        val groups = Array(groups) { mutableMapOf<ActiveBody, List<ActiveBody>>() }

        for ((a, bs) in cs) {
            numPossibleContacts += bs.size
            groups.minBy { it.size }[a] = bs
        }

        return groups
    }

    private fun narrowPhase(ps: Map<ActiveBody, List<ActiveBody>>) {
        val math = mathPool.get()

        try {
            for ((first, ls) in ps) {
                for (second in ls) {
                    val collided: Boolean

                    val t = measureNanoTime {
                        collided = first.collides(second, math, contactBuffer)
                    }

                    if (!collided) continue

                    debugData.pairCollisions++

                    first.awake.set(true)
                    second.awake.set(true)

                    if (Udar.CONFIG.debug.collisionTimes > 0) {
                        println("B-B COLLISION TOOK: ${t.toDouble() / 1_000_000.0} ms")
                    }
                }
            }
        } finally {
            mathPool.put(math)
        }
    }

    fun clear() {
        removeBodies(activeBodies.bodies())
    }

    private fun kill(obj: ActiveBody) {
        aabbTree.remove(obj.fatBB.node!!)
        activeBodies.remove(obj.uuid)
    }

    fun kill() {
        clear()

        simTask.cancel()
        entityTask.cancel()
        scope.cancel()

        worldMeshesManager.kill()
    }
}