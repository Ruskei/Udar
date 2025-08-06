package com.ixume.udar

import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.physics.*
import com.ixume.udar.testing.PhysicsWorldTestDebugData
import org.bukkit.*
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

    private var time = 0
    var frozen = false
    var untilCollision = false
    var steps = 0

    val debugData = PhysicsWorldTestDebugData()
    val math = LocalMathUtil()

    val realWorldGetter = RealWorldGetter(world)
    private val entityUpdater = EntityUpdater(this)
    private val statusUpdater = StatusUpdater(this)

    private val simTask = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, Runnable { tick() }, 1, 1)
    private val realWorldTask =
        Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { realWorldGetter.tick() }, 1, 1)
    private val entityTask = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { entityUpdater.tick() }, 1, 1)

    private val busy = AtomicBoolean(false)

    private fun tick() {
        time++
        repeat((0.05 / TIME_STEP).roundToInt()) {
            var doTick = true
            if (frozen) {
                if (!untilCollision && --steps < 0) doTick = false
            }

            if (doTick) {
                if (busy.get()) return@repeat
                busy.set(true)
                debugData.reset()

                contacts.clear()
                meshes.clear()

                val bodiesSnapshot = activeBodies.get()

                statusUpdater.updateBodies()

                for (i in 0..<bodiesSnapshot.size) {
                    val first = bodiesSnapshot[i]
                    first.ensureNonAligned()
                    val firstBoundingBox = first.boundingBox
                    if (bodiesSnapshot.size > 1) {
                        for (j in (i + 1)..<bodiesSnapshot.size) {
                            debugData.totalPairs++
                            val second = bodiesSnapshot[j]
                            if (!first.awake.get() && !second.awake.get()) {
                                continue
                            }

                            val canFirst = first.capableCollision(second)
                            val canSecond = second.capableCollision(first)

                            var choice = 0 //0 = none, 1 = first, 2 = second
                            if (canFirst.capable && canSecond.capable) {
                                choice = if (canFirst.priority > canSecond.priority) {
                                    1
                                } else {
                                    2
                                }
                            } else if (canFirst.capable) {
                                choice = 1
                            } else if (canSecond.capable) {
                                choice = 2
                            }

                            if (choice == 0) continue

                            if (!firstBoundingBox.overlaps(second.boundingBox)) continue

                            val d = first.pos.distance(second.pos)
                            if (d > first.radius + second.radius) {
                                debugData.missedEarlies++
                                continue
                            }

                            val result: List<IContact>

                            debugData.totalPairCollisionChecks++
                            val t = measureNanoTime {
                                result = if (choice == 1) first.collides(second) else second.collides(first)
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
                    }

                    if (!first.awake.get()) {
                        continue
                    }

                    val environmentBody = EnvironmentBody(world)
                    if (!first.capableCollision(environmentBody).capable) continue

                    val result = first.collides(environmentBody)

                    debugData.totalEnvironmentCollisionChecks++

                    if (result.isEmpty()) continue

                    debugData.environmentCollisions++

                    for (c in result) {
                        val ourContacts =
                            first.previousContacts.filter { it.first is EnvironmentBody || it.second is EnvironmentBody }

                        for (ourContact in ourContacts) {
                            if (ourContact.result.point.distance(c.result.point) < 1e-2) {
                                c.lambdaSum += ourContact.lambdaSum * Udar.CONFIG.collision.lambdaCarryover

                                break

                            }
                        }

                        first.contacts += c
                        contacts += c
                    }
                }
                for (body in bodiesSnapshot) {
                    if (body.awake.get() && body.hasGravity) body.velocity.add(Vector3d(Udar.CONFIG.gravity).mul(TIME_STEP))
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

                busy.set(false)

                if (Udar.CONFIG.debug.data > 0) {
                    debugData.print()
                }
            }
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