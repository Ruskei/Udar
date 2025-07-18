package com.ixume.udar

import org.bukkit.Bukkit
import org.bukkit.World
import java.util.concurrent.ConcurrentHashMap

object PhysicsWorldsManager {
    private val worlds: MutableMap<World, PhysicsWorld> = ConcurrentHashMap()

    fun init() {
        if (!Udar.CONFIG.enabled) return
        worlds.clear()

        for (world in Bukkit.getWorlds()) {
            worlds[world] = PhysicsWorld(world)
        }
    }

    fun disable() {
        if (!Udar.CONFIG.enabled) return

        worlds.values.forEach { it.kill() }
        worlds.clear()
    }

    fun getPhysicsWorld(world: World): PhysicsWorld? {
        if (!Udar.CONFIG.enabled) return null
        return worlds.getOrPut(world) { PhysicsWorld(world) }
    }
}

val World.physicsWorld: PhysicsWorld?
    get() = PhysicsWorldsManager.getPhysicsWorld(this)