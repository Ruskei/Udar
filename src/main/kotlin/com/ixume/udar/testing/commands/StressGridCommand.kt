package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.physicsWorld
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaterniond
import org.joml.Vector3d

object StressGridCommand : Command {
    override val arg: String = "stress-grid"
    override val description: String = "Fuck you!"

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>
    ): List<String> {
        return emptyList()
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>
    ): Boolean {
        if (sender !is Player) return true

        val bodies = mutableListOf<ActiveBody>()
        for (x in 1..10) {
            for (z in 1..10) {

                val origin = sender.location.toVector().toVector3d().add(x * 1.5, 0.0, z * 1.5)
                val rb = Cuboid(
                    world = sender.world,
                    pos = Vector3d(origin),
                    velocity = Vector3d(),
                    width = 0.99,
                    height = 0.99,
                    length = 0.99,
                    q = Quaterniond(),
                    omega = Vector3d(),
                    density = 1.0,
                    hasGravity = true,
                )

                bodies += rb
            }
        }

        sender.world.physicsWorld?.activeBodies += bodies

        return true
    }
}