package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld

class EntityUpdater(
    val physicsWorld: PhysicsWorld
) {
    fun tick() {
        physicsWorld.activeBodies.get().forEach { it.visualize() }
    }
}