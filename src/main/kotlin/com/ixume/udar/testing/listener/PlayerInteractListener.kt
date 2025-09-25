package com.ixume.udar.testing.listener

import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.physicsWorld
import com.ixume.udar.testing.debugConnect
import org.bukkit.*
import org.bukkit.event.EventHandler
import org.bukkit.event.Listener
import org.bukkit.event.block.Action
import org.bukkit.event.player.PlayerInteractEvent
import org.joml.Vector3d
import org.joml.Vector3i
import kotlin.math.max
import kotlin.math.min
import kotlin.system.measureNanoTime

object PlayerInteractListener : Listener {
    private const val PUSH_STRENGTH = 0.5
    fun init() {
        Bukkit.getPluginManager().registerEvents(this, Udar.INSTANCE)
    }

    private val mesher = LocalMesher()
    private var meshStart: Vector3i? = null
    private var mesh: LocalMesher.Mesh2? = null

    @EventHandler
    fun onInteract(e: PlayerInteractEvent) {
        val type = e.player.inventory.itemInMainHand.type
        if (e.action == Action.RIGHT_CLICK_BLOCK || e.action == Action.RIGHT_CLICK_AIR) {
            if (type == Material.END_ROD) {
                e.isCancelled = true
                val physicsWorld = e.player.world.physicsWorld ?: return
                val start = e.player.eyeLocation.toVector().toVector3d()
                val dir = e.player.location.direction.toVector3d().mul(20.0)
                val end = Vector3d(start).add(dir)

                val allIntersections = mutableListOf<Triple<ActiveBody, Vector3d, Vector3d>>()
                val snapshot = physicsWorld.bodiesSnapshot()
                for (body in snapshot) {
                    if (body.isChild) continue
                    
                    allIntersections += body.intersect(start, end).map { Triple(body, it.first, it.second) }
                }

                val (body, intersection, normal) = allIntersections.minByOrNull { (_, inter, _) ->
                    inter.distanceSquared(
                        start
                    )
                }
                                                   ?: return

                e.player.world.spawnParticle(
                    Particle.REDSTONE, Location(
                        e.player.world,
                        intersection.x,
                        intersection.y,
                        intersection.z,
                    ),
                    5, Particle.DustOptions(Color.YELLOW, 0.5f)
                )

                e.player.world.debugConnect(
                    Vector3d(
                        intersection.x,
                        intersection.y,
                        intersection.z,
                    ),
                    Vector3d(
                        intersection.x + normal.x,
                        intersection.y + normal.y,
                        intersection.z + normal.z,
                    ), Particle.DustOptions(Color.BLUE, 0.25f)
                )

                body.applyImpulse(
                    intersection,
                    normal,
                    Vector3d(dir).normalize(PUSH_STRENGTH * e.player.inventory.itemInMainHand.amount)
                )
            }
        } else if (e.action == Action.LEFT_CLICK_BLOCK) {
            if (type == Material.DIAMOND_SWORD) {
                e.isCancelled = true

                val ms = meshStart

                if (ms == null) {
                    meshStart = e.clickedBlock!!.location.toVector().toVector3i()
                    return
                }

                val t = measureNanoTime {
                    mesh = mesher.mesh(
                        e.player.world, AABB(
                            minX = min(ms.x.toDouble(), e.clickedBlock!!.location.x),
                            minY = min(ms.y.toDouble(), e.clickedBlock!!.location.y),
                            minZ = min(ms.z.toDouble(), e.clickedBlock!!.location.z),
                            maxX = max(ms.x.toDouble(), e.clickedBlock!!.location.x),
                            maxY = max(ms.y.toDouble(), e.clickedBlock!!.location.y),
                            maxZ = max(ms.z.toDouble(), e.clickedBlock!!.location.z),
                        )
                    )

                    meshStart = null
                }

                println("MESHING TOOK ${t.toDouble() / 1_000_000.0}ms")
            }
        }
    }

    fun tick(world: World) {
        mesh?.visualize(world)
    }
}