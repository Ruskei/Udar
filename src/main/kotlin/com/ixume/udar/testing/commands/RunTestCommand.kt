package com.ixume.udar.testing.commands

import com.ixume.udar.Udar
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

object RunTestCommand : Command {
    override val arg: String = "test"
    override val description: String = "Run a test from the config"

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) {
            return true
        }
        if (args.isEmpty()) {
            return true
        }

        val test = Udar.CONFIG.debug.tests.tests[args.first()] ?: return true

        test.run(sender)

        return true
    }

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>,
    ): List<String> {
        return Udar.CONFIG.debug.tests.tests.keys.toList()
    }
}