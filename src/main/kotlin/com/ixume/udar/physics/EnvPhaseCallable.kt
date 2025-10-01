package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody

class EnvPhaseCallable(val world: PhysicsWorld) : Runnable {
    lateinit var bodiesSnapshot: List<ActiveBody>
    var start = 0
    var end = 0

    override fun run() {
//        println(" * Doing $start..<$end, doing: ${(end - start).toDouble() / bodiesSnapshot.size * 100.0}%")
        if (start >= end) {
            return
        }

        val math = world.mathPool.get()
        try {
            var i = start
            while (i < end) {
                val body = bodiesSnapshot[i]

                if (body.isChild) {
                    i++
                    continue
                }

                if (!body.awake.get()) {
                    i++
                    continue
                }

                if (body.capableCollision(world.environmentBody) < 0) {
                    i++
                    continue
                }

                body.collides(world.environmentBody, math, world.envManifoldBuffer)

                i++
            }
        } finally {
            world.mathPool.put(math)
        }
    }
}