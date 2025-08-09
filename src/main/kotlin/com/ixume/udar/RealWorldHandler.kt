package com.ixume.udar

import com.ixume.udar.physics.RealWorldGetter
import org.bukkit.Bukkit
import org.bukkit.World

class RealWorldHandler(
    val world: World
) {

    val getter = RealWorldGetter(world)
    private val realWorldTask =
        Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable { getter.tick() }, 1, 1)

    fun kill() {
        realWorldTask.cancel()
    }
}