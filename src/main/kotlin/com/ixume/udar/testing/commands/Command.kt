package com.ixume.udar.testing.commands

import org.bukkit.command.TabExecutor

interface Command : TabExecutor {
    val arg: String
    val description: String
}