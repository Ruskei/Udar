package com.ixume.udar.collisiondetection.narrowphase

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import it.unimi.dsi.fastutil.ints.Int2ObjectMaps
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap
import it.unimi.dsi.fastutil.ints.IntArrayList
import kotlin.system.measureNanoTime

class NarrowPhaseCallable(val world: PhysicsWorld) : Runnable {
    lateinit var ps: Int2ObjectOpenHashMap<IntArrayList>
    val buf = A2AManifoldArray(4)

    override fun run() {
        buf.clear()
        val math = world.mathPool.get()

        try {
            val iterator = Int2ObjectMaps.fastIterator(ps)
            while (iterator.hasNext()) {
                val en = iterator.next()
                val first = world.activeBodies.fastGet(en.intKey)!!
                val ls = en.value

                for (i in 0..<ls.size) {
                    val second = world.activeBodies.fastGet(ls.getInt(i))!!

                    var shouldCollide = true
                    run check@{
                        for (tag1 in first.tags) {
                            for (tag2 in second.tags) {
                                if (tag1 == tag2 && !tag1.collide) {
                                    shouldCollide = false;
                                    return@check
                                }
                            }
                        }
                    }

                    if (!shouldCollide) continue

                    val collided: Boolean

                    val t = measureNanoTime {
                        collided = first.collides(second, math, buf)
                    }

                    if (!collided) continue

                    first.awake.set(true)
                    second.awake.set(true)

                    if (Udar.Companion.CONFIG.debug.collisionTimes > 0) {
                        println("B-B COLLISION TOOK: ${t.toDouble() / 1_000_000.0} ms")
                    }
                }
            }
        } finally {
            world.mathPool.put(math)
        }
    }
}