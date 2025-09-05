package com.ixume.udar.testing.commands

import com.ixume.udar.physicsWorld
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

object FreezeCommand : Command {
    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?,
    ): List<String?> {
        return emptyList()
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?,
    ): Boolean {
        if (sender !is Player) return true
        val world = sender.world
        val physicsWorld = world.physicsWorld ?: return true

        physicsWorld.frozen.set(!physicsWorld.frozen.get())

        if (physicsWorld.frozen.get()) {
            sender.sendMessage("Frozen!")
        } else {
            sender.sendMessage("Unfrozen!")
        }

        return true
    }

    override val arg: String = "freeze"
    override val description: String = "Freeze physics objects in your world"
}