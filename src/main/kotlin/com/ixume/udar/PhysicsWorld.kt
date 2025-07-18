package com.ixume.udar

import com.ixume.udar.body.ActiveBody
import com.ixume.udar.body.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.body.BodyType
import com.ixume.udar.body.ImmovableBody
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.ContactsSolver
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.testing.debugConnect
import org.bukkit.*
import org.joml.Vector3d
import kotlin.math.roundToInt

class PhysicsWorld(
    val world: World
) {
    val activeBodies: MutableList<ActiveBody> = mutableListOf()
    var contacts: MutableList<Contact> = mutableListOf()
    var meshes: MutableList<Mesh> = mutableListOf()

    private var time = 0
    private var frozen = false
    private var untilCollision = false
    private var steps = 0

    private val task = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { tick() }, 1, 1)

    private fun tick() {
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

                            if (!firstBoundingBox.overlaps(second.boundingBox)) continue

                            val result = first.collidesBody(second) ?: continue
                            val contact = Contact(second, first, result)
//
                            val ourContacts =
                                first.previousContacts.filter { it.first == second || it.second == second }

                            first.contacts += contact
                            second.contacts += contact

                            for (ourContact in ourContacts) {
                                if (ourContact.result.point.distance(result.point) < 1e-2) {
                                    contact.lambdaSum += ourContact.lambdaSum * Udar.CONFIG.collision.lambdaCarryover

                                    break
                                }
                            }

                            contacts += contact
                        }
                    }

                    val mesh = ImmovableBody(world)
                    val result = first.collidesEnvironment()

                    for (r in result) {
                        val c = Contact(first, mesh, r)

                        val ourContacts =
                            first.previousContacts.filter { it.first.type == BodyType.PASSIVE || it.second.type == BodyType.PASSIVE }

                        for (ourContact in ourContacts) {
                            if (ourContact.result.point.distance(r.point) < 1e-2) {
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

                ContactsSolver.solve(contacts)

                for (body in activeBodies) {
                    body.step()
                }

                if (untilCollision && contacts.isNotEmpty()) {
                    untilCollision = false
                    frozen = true
                }
            }

            if (time % Udar.CONFIG.debug.frequency == 0) {
                for (body in activeBodies) {
                    body.visualize()
                }

                for (contact in contacts) {
                    val (point,
                        norm,
                        _) = contact.result

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


//                        world.debugConnect(
//                            point,
//                            Vector3d(point).add(contact.t1),
//                            DustOptions(Color.YELLOW, 0.2f)
//                        )
//
//                        world.spawnParticle(
//                            Particle.REDSTONE,
//                            Location(
//                                world,
//                                point.x, point.y, point.z,
//                            ),
//                            1, DustOptions(Color.FUCHSIA, 0.4f)
//                        )
//
//                        world.debugConnect(
//                            point,
//                            Vector3d(point).add(contact.t2),
//                            DustOptions(Color.YELLOW, 0.2f)
//                        )
//
//                        world.spawnParticle(
//                            Particle.REDSTONE,
//                            Location(
//                                world,
//                                point.x, point.y, point.z,
//                            ),
//                            1, DustOptions(Color.FUCHSIA, 0.4f)
//                        )


//                    val minkowskiDebugOrigin = Vector3d(point).add(0.0, 3.0, 0.0)
//                    val vertices = mutableSetOf<Vector3d>()
//                    if (minkowski != null) {
//                        originals!!
//                        closest!!
//                        for ((a, b, c) in minkowski) {
//                            vertices += a
//                            vertices += b
//                            vertices += c
//
//                            if (DEBUG_LEVEL > 2) {
//                                val color =
//                                    choices[Vector3d(a).add(b).add(c).hashCode().absoluteValue % (choices.size)]
//
//                                world.debugConnect(
//                                    Vector3d(minkowskiDebugOrigin).add(Vector3d(a).mul(enhancement)),
//                                    Vector3d(minkowskiDebugOrigin).add(Vector3d(b).mul(enhancement)),
//                                    Particle.DustOptions(Color.BLACK, 0.2f)
//                                )
//
//                                world.debugConnect(
//                                    Vector3d(minkowskiDebugOrigin).add(Vector3d(a).mul(enhancement)),
//                                    Vector3d(minkowskiDebugOrigin).add(Vector3d(c).mul(enhancement)),
//                                    Particle.DustOptions(Color.BLACK, 0.2f)
//                                )
//
//                                world.debugConnect(
//                                    Vector3d(minkowskiDebugOrigin).add(Vector3d(c).mul(enhancement)),
//                                    Vector3d(minkowskiDebugOrigin).add(Vector3d(b).mul(enhancement)),
//                                    Particle.DustOptions(Color.BLACK, 0.2f)
//                                )
//
//                                repeat(50) {
//                                    var cA = Random.nextDouble()
//                                    var cB = Random.nextDouble()
//                                    var cC = Random.nextDouble()
//
//                                    val t = cA + cB + cC
//
//                                    cA /= t
//                                    cB /= t
//                                    cC /= t
//
//                                    val bp =
//                                        Vector3d(a).mul(cA).add(Vector3d(b).mul(cB)).add(Vector3d(c).mul(cC))
//
//                                    world.spawnParticle(
//                                        Particle.REDSTONE,
//                                        Location(
//                                            world,
//                                            minkowskiDebugOrigin.x + bp.x * enhancement.x,
//                                            minkowskiDebugOrigin.y + bp.y * enhancement.y,
//                                            minkowskiDebugOrigin.z + bp.z * enhancement.z,
//                                        ),
//                                        1,
//                                        Particle.DustOptions(color, 0.2f)
//                                    )
//                                }
//                            }
//                        }
//
//                        for (vertex in vertices) {
//                            val (start, end) = originals[vertex]!!
//
//                            val newPos = Vector3d(minkowskiDebugOrigin).add(vertex)
//
//                            if (DEBUG_LEVEL > 2) {
//                                world.debugConnect(
//                                    start,
//                                    end,
//                                    Particle.DustOptions(Color.BLUE, 0.1f)
//                                )
//
//                                world.debugConnect(
//                                    newPos,
//                                    end,
//                                    Particle.DustOptions(Color.YELLOW, 0.1f)
//                                )
//
//                                world.debugConnect(
//                                    start,
//                                    newPos,
//                                    Particle.DustOptions(Color.YELLOW, 0.1f)
//                                )
//
//                                world.spawnParticle(
//                                    Particle.REDSTONE,
//                                    Location(
//                                        world,
//                                        newPos.x,
//                                        newPos.y,
//                                        newPos.z,
//                                    ),
//                                    1, Particle.DustOptions(Color.WHITE, 0.5f)
//                                )
//                            }
//
//                            if (DEBUG_LEVEL > 2) {
//                                world.spawnParticle(
//                                    Particle.REDSTONE,
//                                    Location(
//                                        world,
//                                        minkowskiDebugOrigin.x,
//                                        minkowskiDebugOrigin.y,
//                                        minkowskiDebugOrigin.z,
//                                    ),
//                                    1, Particle.DustOptions(Color.ORANGE, 0.5f)
//                                )
//
//                                world.spawnParticle(
//                                    Particle.REDSTONE,
//                                    Location(
//                                        world,
//                                        minkowskiDebugOrigin.x + closest.first.x,
//                                        minkowskiDebugOrigin.y + closest.first.y,
//                                        minkowskiDebugOrigin.z + closest.first.z,
//                                    ),
//                                    1, Particle.DustOptions(Color.RED, 0.5f)
//                                )
//
//                                world.spawnParticle(
//                                    Particle.REDSTONE,
//                                    Location(
//                                        world,
//                                        minkowskiDebugOrigin.x + closest.second.x,
//                                        minkowskiDebugOrigin.y + closest.second.y,
//                                        minkowskiDebugOrigin.z + closest.second.z,
//                                    ),
//                                    1, Particle.DustOptions(Color.RED, 0.5f)
//                                )
//
//                                world.spawnParticle(
//                                    Particle.REDSTONE,
//                                    Location(
//                                        world,
//                                        minkowskiDebugOrigin.x + closest.third.x,
//                                        minkowskiDebugOrigin.y + closest.third.y,
//                                        minkowskiDebugOrigin.z + closest.third.z,
//                                    ),
//                                    1, Particle.DustOptions(Color.RED, 0.5f)
//                                )
//                            }
//                        }
                    }
                }
            }

//                    if (DEBUG_LEVEL > 0) {
//                        for (body in bodies) {
//                            val boundingBox = body.boundingBox
//
//                            if (DEBUG_LEVEL > 1) {
//                                val blocks = boundingBox.overlappingBlocks(world)
//                                for (block in blocks) {
//                                    world.debugBoundingBox(block.boundingBox, DustOptions(Color.RED, 0.4f), 0.24)
//                                }
//                            }
//
//                            world.debugBoundingBox(boundingBox, DustOptions(Color.BLUE, 0.4f))
//                        }
//                    }

//            if (Udar.CONFIG.debug.mesh > 2) {
//                PhysicsListener.mesh?.visualize(world, visualizeFaces = false, visualizeEdges = true)
//                for (mesh in meshes) {
//                    mesh.visualize(world, visualizeFaces = false, visualizeEdges = true)
//                }
//            }
//        }
    }

    fun clear() {
        activeBodies.forEach { it.kill() }
    }

    fun kill() {
        clear()

        task.cancel()
    }
}