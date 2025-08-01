package com.ixume.udar

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.physics.ContactSolver
import com.ixume.udar.physics.IContact
import com.ixume.udar.testing.debugConnect
import org.bukkit.*
import org.joml.Vector3d
import kotlin.math.roundToInt
import kotlin.system.measureNanoTime

class PhysicsWorld(
    val world: World
) {
    val activeBodies: MutableList<ActiveBody> = mutableListOf()
    val contacts: MutableList<IContact> = mutableListOf()
    val meshes: MutableList<Mesh> = mutableListOf()

    private var time = 0
    var frozen = false
    var untilCollision = false
    var steps = 0

    private val task = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { tick() }, 1, 1)

    private fun tick() {
        time++
        repeat((0.05 / TIME_STEP).roundToInt()) {
            var doTick = true
            if (frozen) {
                if (!untilCollision && --steps < 0) doTick = false
            }

            if (doTick) {
                contacts.clear()
                meshes.clear()

                for (i in 0..<activeBodies.size) {
                    val first = activeBodies[i]
                    first.ensureNonAligned()
                    val firstBoundingBox = first.boundingBox
                    if (activeBodies.size > 1) {
                        for (j in (i + 1)..<activeBodies.size) {
                            val second = activeBodies[j]

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

                            val result: List<IContact>

                            val t = measureNanoTime {
                                result = if (choice == 1) first.collides(second) else second.collides(first)
                            }

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

                    val environmentBody = EnvironmentBody(world)
                    if (!first.capableCollision(environmentBody).capable) continue

                    val result = first.collides(environmentBody)

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

                for (body in activeBodies) {
                    if (body.hasGravity) body.velocity.add(Vector3d(Udar.CONFIG.gravity).mul(TIME_STEP))
                }

                ContactSolver.solve(contacts)

                for (body in activeBodies) {
                    body.step()
                }

                if (untilCollision && contacts.isNotEmpty()) {
                    untilCollision = false
                    frozen = true
                }
            }

            if (time % Udar.CONFIG.debug.frequency == 0) {
//                for (body in activeBodies) {
//                    body.visualize()
//                }

                for (contact in contacts) {
                    if (Udar.CONFIG.debug.normals > 0) {
                        val (point, norm, _) = contact.result

                        world.debugConnect(
                            point,
                            Vector3d(point).add(Vector3d(norm).mul(0.5)),
                            Particle.DustOptions(Color.BLUE, 0.15f)
                        )

                        world.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                world,
                                point.x, point.y, point.z,
                            ),
                            1, Particle.DustOptions(Color.RED, 0.3f)
                        )
                    }
                }
            }
        }
    }

    fun clear() {
        activeBodies.forEach { it.kill() }
        activeBodies.clear()
    }

    fun kill() {
        clear()

        task.cancel()
    }
}