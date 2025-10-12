package com.ixume.udar.body.active.hook

import java.util.concurrent.CopyOnWriteArrayList

class HookManager {
    private val collisionListeners = CopyOnWriteArrayList<(CollisionContext, RemovalLambda) -> Unit>()

    fun registerOnCollisionListener(listener: (CollisionContext, RemovalLambda) -> Unit) {
        collisionListeners += listener
    }

    fun deregisterOnCollisionListener(listener: (CollisionContext, RemovalLambda) -> Unit) {
        collisionListeners -= listener
    }

    fun onCollision(
        x: Double, y: Double, z: Double,
        impulse: Double,
    ) {
        if (collisionListeners.isEmpty()) return
        val context = CollisionContext(x, y, z, impulse)
        collisionListeners.forEach {
            it.invoke(context, RemovalLambda {
                collisionListeners.remove(it)
            })
        }
    }
}

@JvmRecord
data class CollisionContext(
    val x: Double, val y: Double, val z: Double,
    val impulse: Double,
)

@JvmInline
value class RemovalLambda(val lambda: () -> Unit)