package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar

class StatusUpdater(
    val physicsWorld: PhysicsWorld,
) {
    fun updateBodies() {
        val config = Udar.CONFIG
        val timeStep = config.timeStep
        val birthTime = config.birthTime
        val linear = config.sleepLinearVelocity
        val angular = config.sleepAngularVelocity

        for (obj in physicsWorld.activeBodies) {
            obj.age++
            if (obj.age > birthTime) {
                if (obj.startled) {
                    obj.awake = true
                    obj.startled = false
                    continue
                }

                val sleep = (obj.linearDelta.length() < linear * timeStep && obj.angularDelta < angular * timeStep)

                obj.awake = !sleep
            }
        }
    }
}