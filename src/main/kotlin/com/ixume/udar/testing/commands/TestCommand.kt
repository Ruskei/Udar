package com.ixume.udar.testing.commands

import com.ixume.udar.Udar
import org.bukkit.Bukkit
import org.bukkit.command.CommandSender
import org.bukkit.command.defaults.BukkitCommand

class TestCommand : BukkitCommand("udar") {
    private val commandMap = mapOf(
        CuboidCommand.arg to CuboidCommand,
        ReloadCommand.arg to ReloadCommand,
        ClearCommand.arg to ClearCommand,
        SDFProjectionCommand.arg to SDFProjectionCommand,
        FreezeCommand.arg to FreezeCommand,
        StepCommand.arg to StepCommand,
        MarkPointCommand.arg to MarkPointCommand,
        CompositeCommand.arg to CompositeCommand,
        RunTestCommand.arg to RunTestCommand,
    )

    private fun sendHelp(sender: CommandSender) {
        val builder = StringBuilder()
        builder.append("Usage:\n")
        for ((a, v) in commandMap) {
            builder.append(" - $a: ${v.description}\n")
        }

        sender.sendMessage(builder.toString())
    }

    override fun execute(
        sender: CommandSender,
        commandLabel: String,
        args: Array<out String>
    ): Boolean {
        if (args.isEmpty()) {
            sendHelp(sender)
            return true
        }

        val subArgs = Array(args.size - 1) { args[it + 1] }
        commandMap[args.first()]?.onCommand(sender, this, commandLabel, subArgs)

        return true
    }

    override fun tabComplete(
        sender: CommandSender,
        alias: String,
        args: Array<out String>
    ): List<String> {
        if (args.isEmpty()) return emptyList()
        if (args.size == 1) return commandMap.keys.toList()

        val subArgs = Array(args.size - 1) { args[it + 1] }
        return commandMap[args.first()]?.onTabComplete(sender, this, alias, subArgs) ?: listOf()
    }

    companion object {
        fun load() {
            if (Udar.CONFIG.enabled) {
                if (Bukkit.getCommandMap().getCommand("udar") == null) {
                    Bukkit.getCommandMap().register("udar", TestCommand())
                }
            } else {
                Bukkit.getCommandMap().knownCommands.remove("udar")
            }
        }
    }
}