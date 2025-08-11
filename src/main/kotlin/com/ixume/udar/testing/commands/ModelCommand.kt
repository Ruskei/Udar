package com.ixume.udar.testing.commands

import com.ixume.udar.Udar
import com.ixume.udar.physicsWorld
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

object ModelCommand : Command {
    override val arg: String = "model"
    override val description: String = "Spawn a model"

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<String>
    ): List<String> {
        return Udar.MODELS.map { it.name }
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<String>
    ): Boolean {
        if (sender !is Player) return true

        if (args.isEmpty()) return true

        val modelName = args[0]
        val model = Udar.MODELS.firstOrNull { it.name == modelName } ?: return true

        val physicsWorld = sender.world.physicsWorld ?: return true
        val origin = sender.location.toVector().toVector3d()

        physicsWorld.registerBody(model.realize(physicsWorld, origin))

        return true
    }
}