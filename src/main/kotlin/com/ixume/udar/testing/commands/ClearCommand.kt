package com.ixume.udar.testing.commands

import com.ixume.udar.PhysicsWorldsManager
import com.ixume.udar.Udar
import org.bukkit.Bukkit
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

object ClearCommand : Command {
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
        if (sender is Player) {
            PhysicsWorldsManager.getPhysicsWorld(sender.world)?.clear()
        } else {
            for (world in Bukkit.getWorlds()) {
                PhysicsWorldsManager.getPhysicsWorld(world)?.clear()
            }

            for (test in Udar.CONFIG.debug.tests.tests.values) {
                test.kill()
            }
        }

        return true
    }

    override val arg: String = "clear"
    override val description: String = "Clear active objects"
}