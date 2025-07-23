package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.proverka.Proverka
import com.ixume.proverka.feature.Feature
import com.ixume.proverka.feature.impl.PointTextDisplay
import com.ixume.proverka.feature.impl.PointTextDisplay.Companion.pointText
import com.ixume.udar.Udar
import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.capability.SDFCapable
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.IContact
import com.ixume.udar.testing.debugConnectProverka
import net.kyori.adventure.text.format.TextColor
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d
import java.util.*
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
        val otherStartPoints = other.startPoints

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
                    fine = false,
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
                    null, null, null, null, null, null
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
                    fine = false,
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
                    fine = true,
                )

                otherDistance = other.distance(p)
                myDistance = my.distance(p)

                otherPath += Vector3d(p)
                if (pp.distance(p) <= Udar.CONFIG.sdf.epsilon) {
                    println("below!")
                }

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

            val myNorm = my.gradient(myPoint)
            val otherNorm = other.gradient(otherPoint)

            SDFDebugDatabase.ls += SDFDebugDatabase.SDFDebugInfo(
                my.world,
                Vector3d(start) to detectionPath,
                Vector3d(myPoint) to myPath,
                Vector3d(otherPoint) to otherPath,
                myPoint, otherPoint, myNorm, otherNorm,
            )

            val point = Vector3d(myPoint).mul(0.5).add(Vector3d(otherPoint).mul(0.5))

            return SDFContact(
                other, my,
                CollisionResult(
                    point = point,
                    norm = norm,
                    depth = depth,
                ),
            )
        }

        private fun move(
            p: Vector3d,
            pp: Vector3d,
            on: SDFCapable,
            toward: SDFCapable,
            fine: Boolean,
        ) {
            val stepSize = if (fine) Udar.CONFIG.sdf.fineStepSize else Udar.CONFIG.sdf.stepSize
            pp.set(p)

            val g0 = toward.gradient(p)
            p.sub(g0.mul(stepSize))
            val d = on.distance(p)
            if (d > 0.0) {
                val g = on.gradient(p)
                p.sub(g.mul(d))

                if (pp.distance(p) <= Udar.CONFIG.sdf.epsilon) {
                    println("below!")
                }
            }

            if (pp.distance(p) <= Udar.CONFIG.sdf.epsilon) {
                println("below!")
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
        val myPoint: Vector3d?,
        val otherPoint: Vector3d?,
        val myNormal: Vector3d?,
        val otherNormal: Vector3d?,
    ) {
        private val mine = mutableListOf<UUID>()
        val wm = Proverka.INSTANCE.getWorldManager(world)

        private fun register(feature: Feature) {
            mine += feature.id
            wm.features[feature.id] = feature
        }

        fun step() {
            val w = world

            val nc = Udar.CONFIG.debug.SDFNode

            if (Udar.CONFIG.debug.SDFDetection) {
                register(
                    PointTextDisplay(
                        Location(
                            w,
                            detectionPath.first.x,
                            detectionPath.first.y,
                            detectionPath.first.z,
                        ),
                        text = pointText(TextColor.color(Color.ORANGE.asRGB()))
                    )
                )

                if (nc == -1) {
                    for (p in detectionPath.second) {
                        register(
                            PointTextDisplay(
                                Location(
                                    w,
                                    p.x,
                                    p.y,
                                    p.z,
                                ),
                                text = pointText(TextColor.color(Color.RED.asRGB()))
                            )
                        )
                    }
                } else if (nc > 0) {
                    val p = detectionPath.second[min(nc, detectionPath.second.size - 1)]
                    register(
                        PointTextDisplay(
                            Location(
                                w,
                                p.x,
                                p.y,
                                p.z,
                            ),
                            text = pointText(TextColor.color(Color.RED.asRGB()))
                        )
                    )
                }
            }

            if (Udar.CONFIG.debug.SDFMy && myPath != null) {
                register(
                    PointTextDisplay(
                        Location(
                            w,
                            myPath.first.x,
                            myPath.first.y,
                            myPath.first.z,
                        ),
                        text = pointText(TextColor.color(Color.LIME.asRGB()))
                    )
                )

                if (nc == -1) {
                    for (p in myPath.second) {
                        register(
                            PointTextDisplay(
                                Location(
                                    w,
                                    p.x,
                                    p.y,
                                    p.z,
                                ),
                                text = pointText(TextColor.color(Color.WHITE.asRGB()))
                            )
                        )
                    }
                } else if (nc > 0) {
                    val p = myPath.second[min(nc, myPath.second.size - 1)]
                    register(
                        PointTextDisplay(
                            Location(
                                w,
                                p.x,
                                p.y,
                                p.z,
                            ),
                            text = pointText(TextColor.color(Color.WHITE.asRGB()))
                        )
                    )
                }
            }

            if (Udar.CONFIG.debug.SDFOther && otherPath != null) {
                register(
                    PointTextDisplay(
                        Location(
                            w,
                            otherPath.first.x,
                            otherPath.first.y,
                            otherPath.first.z,
                        ),
                        text = pointText(TextColor.color(Color.FUCHSIA.asRGB()))
                    )
                )


                if (nc == -1) {
                    for (p in otherPath.second) {
                        register(
                            PointTextDisplay(
                                Location(
                                    w,
                                    p.x,
                                    p.y,
                                    p.z,
                                ),
                                text = pointText(TextColor.color(Color.BLACK.asRGB()))
                            )
                        )
                    }
                } else if (nc > 0) {
                    val p = otherPath.second[min(nc, otherPath.second.size - 1)]
                    register(
                        PointTextDisplay(
                            Location(
                                w,
                                p.x,
                                p.y,
                                p.z,
                            ),
                            text = pointText(TextColor.color(Color.BLACK.asRGB()))
                        )
                    )
                }
            }

            if (Udar.CONFIG.debug.SDFContact > 1) {
                if (myPoint != null) {
                    register(
                        PointTextDisplay(
                            Location(
                                w,
                                myPoint.x,
                                myPoint.y,
                                myPoint.z,
                            ),
                            text = pointText(TextColor.color(Color.SILVER.asRGB()))
                        )
                    )

                    if (myNormal != null) {
                        w.debugConnectProverka(
                            myPoint,
                            Vector3d(myPoint).add(Vector3d(myNormal).mul(0.5)),
                            Color.SILVER,
                        ) { mine += it.id }
                    }
                }

                if (otherPoint != null) {
                    w.spawnParticle(
                        Particle.REDSTONE,
                        Location(
                            w,
                            otherPoint.x,
                            otherPoint.y,
                            otherPoint.z,
                        ),
                        Udar.CONFIG.debug.SDFParticleCount,
                        Particle.DustOptions(Color.PURPLE, Udar.CONFIG.debug.SDFNodeSize)
                    )

                    if (otherNormal != null) {
                        w.debugConnectProverka(
                            otherPoint,
                            Vector3d(otherPoint).add(Vector3d(otherNormal).mul(0.5)),
                            Color.PURPLE,
                        ) { mine += it.id }
                    }
                }
            }
        }

        fun kill() {
            for (id in mine) {
                wm.features[id]?.kill()
                wm.features -= id
            }

            mine.clear()
        }
    }
}

class SDFContact(
    first: Body,
    second: Body,
    result: CollisionResult,
) : IContact by Contact(first, second, result, 0.0) {
}