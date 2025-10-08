package com.ixume.udar.collisiondetection.envphase

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.multithreading.runPartitioned
import java.util.concurrent.Executors

class EnvPhaseHandler(
    val physicsWorld: PhysicsWorld,
) {
    private val processors = Udar.CONFIG.envPhaseProcessors
    private val callables = Array(processors) { EnvPhaseCallable(physicsWorld) }
    private val executor = Executors.newFixedThreadPool(processors)
    
    fun process(snapshot: List<ActiveBody>) {
        executor.runPartitioned(snapshot.size, callables) { start, end ->
            this.bodiesSnapshot = snapshot
            this.start = start
            this.end = end
        }
    }

    fun shutdown() {
        executor.shutdown()
    }
}