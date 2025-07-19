package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.Udar
import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.capability.SDFCapable
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.IContact
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d
import kotlin.math.min

class SDFContactGenerator<T>(
    val activeBody: T
) : Collidable where T : Body, T : SDFCapable {
    override fun capableCollision(other: Body): Capability {
        return Capability(other is SDFCapable, Udar.CONFIG.sdf.priority)
    }

    override fun collides(other: Body): List<IContact> {
        other as SDFCapable

        val myStartPoints = activeBody.startPoints
        val otherStartPoints = activeBody.startPoints

        val contacts = mutableListOf<IContact>()

        for (start in myStartPoints) {
            val r = traverse(start, activeBody, other) ?: continue
            contacts += r

            if (Udar.CONFIG.sdf.endFast) {
                return contacts
            }
        }

        for (start in otherStartPoints) {
            val r = traverse(start, other, activeBody) ?: continue
            contacts += r

            if (Udar.CONFIG.sdf.endFast) {
                return contacts
            }
        }

        return contacts
    }


    companion object {
        private fun traverse(
            start: Vector3d,
            my: Body,
            other: Body
        ): SDFContact? {
            my as SDFCapable
            other as SDFCapable
            val stepSize = Udar.CONFIG.sdf.stepSize
            val maxSteps = Udar.CONFIG.sdf.maxSteps
            val maxNormalSteps = Udar.CONFIG.sdf.maxNormalSteps
            val epsilon = Udar.CONFIG.sdf.epsilon
            val errorEpsilon = Udar.CONFIG.sdf.errorEpsilon

            val detectionPath = mutableListOf<Vector3d>()
            val myPath = mutableListOf<Vector3d>()
            val otherPath = mutableListOf<Vector3d>()

            val p = Vector3d(start)

            myPath += p

            var myDistance: Double = my.distance(p)
            if (myDistance > 0) {
                p.sub(my.gradient(p).mul(myDistance))
            }

            val pp = Vector3d(p)

            var otherDistance: Double = other.distance(p)
            if (otherDistance > maxSteps * stepSize) return null

            var itr = 0

            do {
                itr++

                move(
                    p = p,
                    pp = pp,
                    on = my,
                    toward = other,
                )

                myDistance = my.distance(p)
                otherDistance = other.distance(p)

                if (Udar.CONFIG.debug.SDFContact > 2) {
                    println("COLL ITR!")
                    println(" - itr: $itr")
                    println(" - myDistace: $myDistance")
                    println(" - otherDistance: $otherDistance")
                    println(" - p2pp: ${pp.distance(p)} p: $p pp: $pp")
                }

                detectionPath += Vector3d(p)
            } while ((myDistance > 0.0 || otherDistance > 0.0) && pp.distance(p) > epsilon && itr < maxSteps)

            detectionPath += Vector3d(p)

            if (myDistance > 0.0 || otherDistance > 0.0) {
                if (Udar.CONFIG.debug.SDFContact > 1) {
                    println("  * NO COLLISION")
                }

                SDFDebugDatabase.ls += SDFDebugDatabase.SDFDebugInfo(
                    my.world,
                    Vector3d(start) to detectionPath,
                    null,
                    null,
                )

                return null
            }


            val collision = Vector3d(p)

            if (Udar.CONFIG.debug.SDFContact > 0) {
                println("  * COLLISION TOOK $itr ITERATIONS")
            }

            itr = 0

            do {
                itr++

                move(
                    p = p,
                    pp = pp,
                    on = my,
                    toward = other,
                )

                myDistance = my.distance(p)
                otherDistance = other.distance(p)

                myPath += Vector3d(p)
            } while (myDistance <= errorEpsilon && otherDistance <= errorEpsilon && pp.distance(p) > epsilon && itr < maxNormalSteps)
            if (Udar.CONFIG.debug.SDFContact > 0) {
                println("  * MY TOOK $itr ITERATIONS")
            }
            val myPoint = Vector3d(p)

            p.set(collision)
            pp.set(p)

            myPath += Vector3d(p)

            itr = 0

            do {
                itr++

                move(
                    p = p,
                    pp = pp,
                    on = other,
                    toward = my,
                )

                otherDistance = other.distance(p)
                myDistance = my.distance(p)

                otherPath += Vector3d(p)
            } while (myDistance <= errorEpsilon && otherDistance <= errorEpsilon && pp.distance(p) > epsilon && itr < maxNormalSteps)
            if (Udar.CONFIG.debug.SDFContact > 0) {
                println("  * OTHER TOOK $itr ITERATIONS")
            }

            otherPath += Vector3d(p)

            val otherPoint = Vector3d(p)

            val norm = Vector3d(myPoint).sub(otherPoint).normalize()
            if (!norm.isFinite) return null

            val depth = otherPoint.distance(myPoint)

            if (Udar.CONFIG.debug.SDFContact > 2) {
                println("SDF COLLISION:")
                println(" - myPoint: $myPoint")
                println(" - otherPoint: $otherPoint")
                println(" - norm: $norm")
                println(" - depth: $depth")
            }

            SDFDebugDatabase.ls += SDFDebugDatabase.SDFDebugInfo(
                my.world,
                Vector3d(start) to detectionPath,
                Vector3d(collision) to myPath,
                Vector3d(collision) to otherPath
            )

            return SDFContact(
                other, my,
                CollisionResult(
                    point = Vector3d(myPoint).mul(0.5).add(Vector3d(otherPoint).mul(0.5)),
                    norm = norm,
                    depth = depth,
                ),
            )
        }

        private fun move(
            p: Vector3d,
            pp: Vector3d,
            on: SDFCapable,
            toward: SDFCapable
        ) {
            val stepSize = Udar.CONFIG.sdf.stepSize
            pp.set(p)

            p.sub(toward.gradient(p).mul(stepSize))
            val d = on.distance(p)
            if (d > 0) {
                p.sub(on.gradient(p).mul(d))
            }

            check(p.isFinite)
        }
    }
}

object SDFDebugDatabase {
    val ls = mutableListOf<SDFDebugInfo>()

    data class SDFDebugInfo(
        val world: World,
        val detectionPath: Pair<Vector3d, List<Vector3d>>,
        val myPath: Pair<Vector3d, List<Vector3d>>?,
        val otherPath: Pair<Vector3d, List<Vector3d>>?,
    ) {
        fun visualize() {
            val w = world
            val nc = Udar.CONFIG.debug.SDFNode

            if (Udar.CONFIG.debug.SDFDetection) {
                w.spawnParticle(
                    Particle.REDSTONE,
                    Location(
                        w,
                        detectionPath.first.x,
                        detectionPath.first.y,
                        detectionPath.first.z,
                    ),
                    Udar.CONFIG.debug.SDFParticleCount,
                    Particle.DustOptions(Color.ORANGE, Udar.CONFIG.debug.SDFStartSize)
                )

                if (nc <= -1) {
                    for (p in detectionPath.second) {
                        w.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                w,
                                p.x,
                                p.y,
                                p.z,
                            ),
                            Udar.CONFIG.debug.SDFParticleCount,
                            Particle.DustOptions(Color.RED, Udar.CONFIG.debug.SDFNodeSize)
                        )
                    }
                } else {
                    val p = detectionPath.second[min(nc, detectionPath.second.size - 1)]
                    w.spawnParticle(
                        Particle.REDSTONE,
                        Location(
                            w,
                            p.x,
                            p.y,
                            p.z,
                        ),
                        Udar.CONFIG.debug.SDFParticleCount,
                        Particle.DustOptions(Color.RED, Udar.CONFIG.debug.SDFNodeSize)
                    )
                }
            }

            if (Udar.CONFIG.debug.SDFMy && myPath != null) {
                w.spawnParticle(
                    Particle.REDSTONE, Location(
                        w,
                        myPath.first.x,
                        myPath.first.y,
                        myPath.first.z,
                    ), Udar.CONFIG.debug.SDFParticleCount, Particle.DustOptions(Color.LIME, Udar.CONFIG.debug.SDFStartSize)
                )

                if (nc <= -1) {
                    for (p in myPath.second) {
                        w.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                w,
                                p.x,
                                p.y,
                                p.z,
                            ),
                            Udar.CONFIG.debug.SDFParticleCount,
                            Particle.DustOptions(Color.WHITE, Udar.CONFIG.debug.SDFNodeSize)
                        )
                    }
                } else {
                    val p = myPath.second[min(nc, myPath.second.size - 1)]
                    w.spawnParticle(
                        Particle.REDSTONE,
                        Location(
                            w,
                            p.x,
                            p.y,
                            p.z,
                        ),
                        Udar.CONFIG.debug.SDFParticleCount,
                        Particle.DustOptions(Color.WHITE, Udar.CONFIG.debug.SDFNodeSize)
                    )
                }
            }

            if (Udar.CONFIG.debug.SDFOther && otherPath != null) {
                w.spawnParticle(
                    Particle.REDSTONE,
                    Location(
                        w,
                        otherPath.first.x,
                        otherPath.first.y,
                        otherPath.first.z,
                    ),
                    Udar.CONFIG.debug.SDFParticleCount,
                    Particle.DustOptions(Color.FUCHSIA, Udar.CONFIG.debug.SDFStartSize)
                )

                if (nc <= -1) {
                    for (p in otherPath.second) {
                        w.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                w,
                                p.x,
                                p.y,
                                p.z,
                            ),
                            Udar.CONFIG.debug.SDFParticleCount,
                            Particle.DustOptions(Color.BLACK, Udar.CONFIG.debug.SDFNodeSize)
                        )
                    }
                } else {
                    val p = otherPath.second[min(nc, otherPath.second.size - 1)]
                    w.spawnParticle(
                        Particle.REDSTONE,
                        Location(
                            w,
                            p.x,
                            p.y,
                            p.z,
                        ),
                        Udar.CONFIG.debug.SDFParticleCount,
                        Particle.DustOptions(Color.BLACK, Udar.CONFIG.debug.SDFNodeSize)
                    )
                }
            }
        }
    }
}

class SDFContact(
    first: Body,
    second: Body,
    result: CollisionResult,
) : IContact by Contact(first, second, result, 0.0) {
}