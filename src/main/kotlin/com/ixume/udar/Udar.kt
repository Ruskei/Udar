package com.ixume.udar

import com.ixume.udar.testing.Config
import com.ixume.udar.testing.TestingConfigLoader
import com.ixume.udar.testing.commands.TestCommand
import org.bukkit.plugin.java.JavaPlugin
import java.util.logging.Logger

class Udar : JavaPlugin() {
    override fun onEnable() {
        INSTANCE = this
        LOGGER = logger

        TestingConfigLoader.load()
        TestCommand.load()

        PhysicsWorldsManager.init()
    }

    override fun onDisable() {
        PhysicsWorldsManager.disable()
    }

    companion object {
        lateinit var INSTANCE: Udar
        lateinit var LOGGER: Logger
        lateinit var CONFIG: Config
    }
}
