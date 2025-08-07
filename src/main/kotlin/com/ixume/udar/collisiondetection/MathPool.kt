package com.ixume.udar.collisiondetection

import java.util.concurrent.ConcurrentLinkedDeque

class MathPool(
    amount: Int
) {
    private val items = ConcurrentLinkedDeque<LocalMathUtil>()

    init {
        repeat(amount * 2) { // multiplied by 2 for async safety
            items += LocalMathUtil()
        }
    }

    fun get(): LocalMathUtil {
        check(items.isNotEmpty()) { "Too few math utils allocated!" }

        return items.poll()
    }

    fun put(element: LocalMathUtil) {
        items.add(element)
    }
}