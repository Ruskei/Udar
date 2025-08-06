package com.ixume.udar.testing.commands

import com.ixume.udar.Udar
import com.ixume.udar.collisiondetection.capability.SDFCapable
import com.ixume.udar.physicsWorld
import org.bukkit.Bukkit
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Particle
import org.bukkit.command.CommandSender
import org.bukkit.command.TabExecutor
import org.bukkit.entity.Player
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3d

object SDFProjectionCommand : Command {
    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?
    ): List<String?> {
        return emptyList()
    }

    private val players = mutableListOf<Player>()

    private var task: BukkitTask? = null

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>
    ): Boolean {
        if (sender !is Player) return true

        if (sender in players) {
            players.remove(sender)
            return true
        }

        players.add(sender)

        val world = sender.world
        val physicsWorld = world.physicsWorld ?: return true

        if (task == null) {
            task = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable {
                val o = sender.location.toVector().toVector3d()

                val obj = physicsWorld.activeBodies
                    .get()
                    .filter { it is SDFCapable }
                    .minByOrNull { it.pos.distance(o) }
                    ?: return@Runnable

                obj as SDFCapable

                val gradient = obj.gradient(o)
                val d = obj.distance(o)
                val p = Vector3d(o).sub(Vector3d(gradient).mul(d))

                world.spawnParticle(
                    Particle.REDSTONE, Location(
                        world, p.x, p.y, p.z
                    ), 5, Particle.DustOptions(Color.RED, 0.3f)
                )
            }, 1, 1)
        }


        return true
    }

    override val arg: String = "sdf-project"
    override val description: String = "Project a point onto closest object's surfaces with SDF"
}