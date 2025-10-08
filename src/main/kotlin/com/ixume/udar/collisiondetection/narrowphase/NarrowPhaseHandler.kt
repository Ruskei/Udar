package com.ixume.udar.collisiondetection.narrowphase

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import it.unimi.dsi.fastutil.ints.Int2ObjectOpenHashMap
import it.unimi.dsi.fastutil.ints.IntArrayList
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors

class NarrowPhaseHandler(
    val physicsWorld: PhysicsWorld,
) {
    private val processors = Udar.CONFIG.narrowPhaseProcessors
    private val executor = Executors.newFixedThreadPool(processors)
    private val callables = Array(processors) { NarrowPhaseCallable(physicsWorld) }
    val _groupedBroadCollisions =
        Array<Int2ObjectOpenHashMap<IntArrayList>>(processors) { Int2ObjectOpenHashMap() }

    fun process(pairs: Array<Int2ObjectOpenHashMap<IntArrayList>>) {
        check(pairs.size == processors)

        val latch = CountDownLatch(processors)

        for (proc in 0..<processors) {
            val callable = callables[proc]

            callable.ps = pairs[proc]

            executor.execute {
                callable.run()

                latch.countDown()
            }
        }

        latch.await()

        for (callable in callables) {
            for (i in 0..<callable.buf.size()) {
                physicsWorld.manifoldBuffer.load(callable.buf, i)
            }
        }
    }

    fun shutdown() {
        executor.shutdown()
    }
}