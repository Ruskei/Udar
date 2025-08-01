package com.ixume.udar.testing.commands

import com.ixume.udar.collisiondetection.contactgeneration.SDFDebugDatabase
import com.ixume.udar.testing.ConfigLoader
import org.bukkit.command.CommandSender

object ReloadCommand : Command {
    override val arg: String = "reload"
    override val description: String = "Reload config"

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>
    ): List<String?> {
        return emptyList()
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?
    ): Boolean {
        ConfigLoader.load()
        TestCommand.load()

        SDFDebugDatabase.ls.forEach { it.kill(); it.step() }

        sender.sendMessage("Reloaded!")

        return true
    }
}