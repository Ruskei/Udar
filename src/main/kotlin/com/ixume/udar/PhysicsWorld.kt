package com.ixume.udar

import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.collisiondetection.pool.MathPool
import com.ixume.udar.physics.*
import com.ixume.udar.testing.PhysicsWorldTestDebugData
import kotlinx.coroutines.*
import org.bukkit.Bukkit
import org.bukkit.World
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.roundToInt
import kotlin.system.measureNanoTime

class PhysicsWorld(
    val world: World
) {
    val activeBodies = AtomicList<ActiveBody>()
    val contacts: MutableList<IContact> = mutableListOf()
    val meshes: MutableList<Mesh> = mutableListOf()

    private val environmentBody = EnvironmentBody(this)

    private var time = 0
    private var physicsTime = 0
    var frozen = false
    var untilCollision = false
    var steps = 0

    val debugData = PhysicsWorldTestDebugData()

    val realWorldGetter = RealWorldGetter(world)
    private val entityUpdater = EntityUpdater(this)
    private val statusUpdater = StatusUpdater(this)

    private val simTask = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, Runnable { tick() }, 1, 1)
    private val realWorldTask =
        Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { realWorldGetter.tick() }, 1, 1)
    private val entityTask = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { entityUpdater.tick() }, 1, 1)

    private val busy = AtomicBoolean(false)

    private val processors = 3//Runtime.getRuntime().availableProcessors()
    private val mathPool = MathPool(processors)
    private val scope = CoroutineScope(Dispatchers.Default)

    private val dataInterval = 400
    private var rollingAverage = 0.0

    private fun tick() {
        time++
        repeat((0.05 / TIME_STEP).roundToInt()) {
            var doTick = true
            if (frozen) {
                if (!untilCollision && --steps < 0) doTick = false
            }

            if (doTick) {
                if (!busy.compareAndSet(false, true)) return@repeat

                val startTime = System.nanoTime()

                physicsTime++

                debugData.reset()

                contacts.clear()
                meshes.clear()

                val bodiesSnapshot = activeBodies.get()

                statusUpdater.updateBodies()

                var job: Job? = null

                val activePairs = broadPhase(bodiesSnapshot, processors)


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

                    val math = mathPool.get()

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

                    for (body in bodiesSnapshot) {
                        if (body.awake.get() && body.hasGravity) body.velocity.add(
                            Vector3d(Udar.CONFIG.gravity).mul(
                                TIME_STEP
                            )
                        )
                    }

                    ContactSolver.solve(contacts)

                    for (body in bodiesSnapshot) {
                        if (body.awake.get()) {
                            body.step()
                        }
                    }

                    if (untilCollision && contacts.isNotEmpty()) {
                        untilCollision = false
                        frozen = true
                    }

                    val duration = (System.nanoTime() - startTime).toDouble()
                    rollingAverage += duration / dataInterval.toDouble()

                    if (physicsTime % dataInterval == 0) {
                        println("Narrowphase takes ${rollingAverage / 1_000.0}us on average")
                        rollingAverage = 0.0
                    }

                    busy.set(false)
                }
            }
        }
    }

    private fun broadPhase(bodies: List<ActiveBody>, groups: Int): Array<MutableList<Pair<ActiveBody, ActiveBody>>>? {
        require(groups > 0)
        if (bodies.size <= 1) return null

        val groups = Array(groups) { mutableListOf<Pair<ActiveBody, ActiveBody>>() }

        for (first in bodies) {
            val myPairs = mutableListOf<Pair<ActiveBody, ActiveBody>>()
            first.ensureNonAligned()
            val firstBoundingBox = first.boundingBox

            for (second in bodies) {
                if (second === first) continue

                debugData.totalPairs++
                if (!first.awake.get() && !second.awake.get()) {
                    continue
                }

                if (!firstBoundingBox.overlaps(second.boundingBox)) continue

                val canFirst = first.capableCollision(second)
                if (canFirst < 0) continue

                val d = first.pos.distance(second.pos)
                if (d > first.radius + second.radius) {
                    debugData.missedEarlies++
                    continue
                }

                myPairs += first to second
            }

            groups.minBy { it.size } += myPairs
        }

        return groups
    }

    private fun narrowPhase(ps: List<Pair<ActiveBody, ActiveBody>>) {
        val math = mathPool.get()

        try {
            for ((first, second) in ps) {
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

                for (contact in result) {
                    val ourContacts =
                        first.previousContacts.filter { it.first == second || it.second == second }

                    first.contacts += contact
                    second.contacts += contact

                    for (ourContact in ourContacts) {
                        if (ourContact.result.point.distance(contact.result.point) < 1e-2) {
                            contact.lambdaSum += ourContact.lambdaSum * Udar.CONFIG.collision.lambdaCarryover

                            break
                        }
                    }

                    contacts += contact
                }
            }
        } finally {
            mathPool.put(math)
        }
    }

    fun clear() {
        activeBodies.get().forEach { it.kill() }
        activeBodies.clear()
    }

    fun kill() {
        clear()

        simTask.cancel()
        realWorldTask.cancel()
        entityTask.cancel()
    }
}