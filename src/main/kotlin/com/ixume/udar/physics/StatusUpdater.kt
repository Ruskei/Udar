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
        val sleepTime = config.sleepTime

        for (obj in snapshot) {
            if (obj.isChild) continue

            obj.age++
            if (obj.age > birthTime) {
                if (obj.startled.get()) {
                    obj.awake.set(true)
                    obj.startled.set(false)
                    continue
                }

                val sleep =
                    obj.velocity.lengthSquared() < linear && obj.omega.lengthSquared() < angular
                if (sleep) {
                    obj.idleTime++

                    if (obj.idleTime >= sleepTime) {
                        obj.awake.set(false)
                    }
                } else {
                    obj.idleTime = 0
                    obj.awake.set(true)
                }
            }
        }
    }
}