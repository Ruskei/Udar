package com.ixume.udar

import com.ixume.proverka.Proverka
import com.ixume.udar.model.JavaModel
import com.ixume.udar.rp.RPManager
import com.ixume.udar.testing.Config
import com.ixume.udar.testing.ConfigLoader
import com.ixume.udar.testing.commands.TestCommand
import com.ixume.udar.testing.listener.PlayerInteractListener
import org.bukkit.Bukkit
import org.bukkit.plugin.java.JavaPlugin
import java.util.logging.Logger

class Udar : JavaPlugin() {
    override fun onEnable() {
        INSTANCE = this
        LOGGER = logger

        ConfigLoader.load()
        PhysicsWorldsManager.init()

        TestCommand.load()
        RPManager.createResources()

        PlayerInteractListener.init()
    }

    override fun onDisable() {
        PhysicsWorldsManager.disable()

        for (world in Bukkit.getWorlds()) {
            Proverka.INSTANCE.getWorldManager(world).kill()
        }
    }

    companion object {
        lateinit var INSTANCE: Udar
        lateinit var LOGGER: Logger
        lateinit var CONFIG: Config
    }
}
