package com.ixume.udar.body.active.hook

import java.util.concurrent.ConcurrentLinkedQueue

class HookManager {
    private val collisionListeners = ConcurrentLinkedQueue<(CollisionContext) -> Unit>()

    fun registerOnCollisionListener(listener: (CollisionContext) -> Unit) {
        collisionListeners += listener
    }

    fun onCollision(
        x: Double, y: Double, z: Double,
        impulse: Double
    ) {
        if (collisionListeners.isEmpty()) return
        val context = CollisionContext(x, y, z, impulse)
        collisionListeners.forEach { it.invoke(context) }
    }
}

@JvmRecord
data class CollisionContext(
    val x: Double, val y: Double, val z: Double,
    val impulse: Double,
)