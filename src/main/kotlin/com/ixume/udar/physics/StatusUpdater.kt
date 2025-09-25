package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody

class StatusUpdater(
    val physicsWorld: PhysicsWorld,
) {
    fun updateBodies(snapshot: List<ActiveBody>) {
        val config = Udar.CONFIG
        val timeStep = config.timeStep
        val birthTime = config.birthTime
        val linear = config.sleepLinearVelocity
        val angular = config.sleepAngularVelocity

        for (obj in snapshot) {
            if (obj.isChild) continue
            
            obj.age++
            if (obj.age > birthTime) {
                if (obj.startled.get()) {
                    obj.awake.set(true)
                    obj.startled.set(false)
                    continue
                }

//                val sleep = (obj.linearDelta.length() < linear * timeStep && obj.angularDelta < angular * timeStep)
//
//                obj.awake.set(!sleep)
            }
        }
    }
}