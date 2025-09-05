package com.ixume.udar.testing.commands

import com.ixume.udar.physicsWorld
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

object StepCommand : Command {
    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?,
    ): List<String?>? {
        return listOf("<ticks>", "collision")
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true
        val world = sender.world
        val physicsWorld = world.physicsWorld ?: return true

        if (physicsWorld.frozen.get()) {
            if (args.isEmpty()) {
                physicsWorld.steps.set(1)
                sender.sendMessage("Stepping!")
            } else {
                val n = args[0].toIntOrNull()
                if (n == null) {
                    if (args[0] == "collision") {
                        physicsWorld.untilCollision.set(true)

                        sender.sendMessage("Stepping until a collision!")

                        return true
                    }
                    sender.sendMessage("Must provide a # of steps!")
                    return true
                }

                physicsWorld.steps.set(n)
                sender.sendMessage("Stepping for $n steps!")
            }
        }

        return true
    }

    override val arg: String = "step"
    override val description: String = "Step a certain number of ticks"
}