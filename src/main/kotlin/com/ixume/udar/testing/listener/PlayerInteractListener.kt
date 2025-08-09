package com.ixume.udar.testing.listener

import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physicsWorld
import com.ixume.udar.testing.debugConnect
import org.bukkit.*
import org.bukkit.event.EventHandler
import org.bukkit.event.Listener
import org.bukkit.event.player.PlayerInteractEvent
import org.joml.Vector3d

object PlayerInteractListener : Listener {
    private const val PUSH_STRENGTH = 0.5
    fun init() {
        Bukkit.getPluginManager().registerEvents(this, Udar.INSTANCE)
    }

    @EventHandler
    fun onInteract(event: PlayerInteractEvent) {
        val physicsWorld = event.player.world.physicsWorld ?: return
        val start = event.player.eyeLocation.toVector().toVector3d()
        val dir = event.player.location.direction.toVector3d().mul(20.0)
        val end = Vector3d(start).add(dir)

        val allIntersections = mutableListOf<Triple<ActiveBody, Vector3d, Vector3d>>()
        val snapshot = physicsWorld.bodiesSnapshot()
        for (body in snapshot) {
            allIntersections += body.intersect(start, end).map { Triple(body, it.first, it.second) }
        }

        val (body, intersection, normal) = allIntersections.minByOrNull { (_, inter, _) -> inter.distanceSquared(start) }
            ?: return

        event.player.world.spawnParticle(
            Particle.REDSTONE, Location(
                event.player.world,
                intersection.x,
                intersection.y,
                intersection.z,
            ),
            5, Particle.DustOptions(Color.YELLOW, 0.5f)
        )

        event.player.world.debugConnect(
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

        val type = event.player.inventory.itemInMainHand.type
        if (type == Material.END_ROD) {
            body.applyImpulse(intersection, normal, Vector3d(dir).normalize(PUSH_STRENGTH * event.player.inventory.itemInMainHand.amount))
        }
    }
}