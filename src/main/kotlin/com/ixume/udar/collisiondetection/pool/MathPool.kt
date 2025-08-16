package com.ixume.udar.collisiondetection.pool

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.collisiondetection.LocalMathUtil
import java.util.concurrent.ConcurrentLinkedDeque

class MathPool(
    world: PhysicsWorld,
    amount: Int
) : Pool<LocalMathUtil> {
    private val items = ConcurrentLinkedDeque<LocalMathUtil>()

    init {
        repeat(amount * 2) {
            items += LocalMathUtil(world)
        }
    }

    override fun get(): LocalMathUtil {
        check(items.isNotEmpty()) { "Too few math utils allocated!" }

        return items.poll()
    }

    override fun put(element: LocalMathUtil) {
        items.add(element)
    }
}