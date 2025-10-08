package com.ixume.udar.collisiondetection.pool

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import java.util.concurrent.ConcurrentLinkedDeque

class MathPool(
    val world: PhysicsWorld,
) : Pool<LocalMathUtil> {
    private val items = ConcurrentLinkedDeque<LocalMathUtil>()

    init {
        items += LocalMathUtil(world)
    }

    override fun get(): LocalMathUtil {
        return items.poll() ?: LocalMathUtil(world)
    }

    override fun put(element: LocalMathUtil) {
        items.add(element)
    }
}