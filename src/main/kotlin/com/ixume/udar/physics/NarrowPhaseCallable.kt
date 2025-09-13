package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import it.unimi.dsi.fastutil.ints.Int2ObjectMaps
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap
import it.unimi.dsi.fastutil.ints.IntArrayList
import kotlin.system.measureNanoTime

class NarrowPhaseCallable(val world: PhysicsWorld) : Runnable {
    lateinit var ps: Int2ObjectOpenHashMap<IntArrayList>

    override fun run() {
        val math = world.mathPool.get()

        try {
            val iterator = Int2ObjectMaps.fastIterator(ps)
            while (iterator.hasNext()) {
                val en = iterator.next()
                val first = world.activeBodies[en.intKey]!!
                val ls = en.value

                for (i in 0..<ls.size) {
                    val second = world.activeBodies[ls.getInt(i)]!!

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