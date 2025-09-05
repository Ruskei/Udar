package com.ixume.udar.testing.commands

import com.ixume.proverka.Proverka
import com.ixume.proverka.feature.impl.PointTextDisplay
import com.ixume.proverka.feature.impl.PointTextDisplay.Companion.pointText
import net.kyori.adventure.text.format.TextColor
import org.bukkit.Location
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player

object MarkPointCommand : Command {
    override val arg: String = "mark"
    override val description: String = "Mark a location"

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>,
    ): List<String> {
        return when (args.size) {
            1 -> listOf("point", "kill")
            2 -> listOf("<x>")
            3 -> listOf("<y>")
            4 -> listOf("<z>")
            else -> listOf()
        }
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true

        val w = sender.world

        if (args.isEmpty()) {
            println("Specify!")
            return true
        }

        when (args[0]) {
            "point" -> {
                val loc = if (args.size == 1) sender.location else let {
                    if (args.size >= 4) {
                        val x = args[1].toDoubleOrNull() ?: return@let null
                        val y = args[2].toDoubleOrNull() ?: return@let null
                        val z = args[3].toDoubleOrNull() ?: return@let null

                        Location(w, x, y, z)
                    } else {
                        null
                    }
                }

                if (loc == null) {
                    sender.sendMessage("Invalid location!")
                    return true
                }

                val feature = PointTextDisplay(
                    loc = loc,
                    text = pointText(TextColor.color(255, 0, 0))
                )

                Proverka.INSTANCE.getWorldManager(w).register(feature)
            }

            "kill" -> {
                Proverka.INSTANCE.getWorldManager(w).kill()
            }
        }

        return true
    }
}