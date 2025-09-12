package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import kotlin.system.measureNanoTime

class NarrowPhaseCallable(val world: PhysicsWorld) : Runnable {
    lateinit var ps: MutableMap<ActiveBody, List<ActiveBody>>

    override fun run() {
        val math = world.mathPool.get()

        try {
            for ((first, ls) in ps) {
                for (second in ls) {
                    val collided: Boolean

                    val t = measureNanoTime {
                        collided = first.collides(second, math, world.manifoldBuffer)
                    }

                    if (!collided) continue

                    first.awake.set(true)
                    second.awake.set(true)

                    if (Udar.CONFIG.debug.collisionTimes > 0) {
                        println("B-B COLLISION TOOK: ${t.toDouble() / 1_000_000.0} ms")
                    }
                }
            }
        } finally {
            world.mathPool.put(math)
        }
    }
}