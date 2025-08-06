package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld

class EntityUpdater(
    val physicsWorld: PhysicsWorld
) {
    fun tick() {
        synchronized(physicsWorld.activeBodies) {
            physicsWorld.activeBodies.forEach { it.visualize() }
        }
    }
}