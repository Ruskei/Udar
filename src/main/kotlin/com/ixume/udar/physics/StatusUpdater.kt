package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Composite

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
                if (obj.startled.get() || (obj is Composite && obj.isStartled())) {
                    obj.idleTime = 0
                    obj.awake.set(true)
                    obj.startled.set(false)
                    if (obj is Composite) {
                        var i = 0
                        val parts = obj.parts
                        while (i < parts.size) {
                            parts[i].startled.set(false)
                            i++
                        }
                    }
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

fun Composite.isStartled(): Boolean {
    var i = 0
    val parts = parts
    while (i < parts.size) {
        if (parts[i].startled.get()) return true
        i++
    }
    
    return false
}