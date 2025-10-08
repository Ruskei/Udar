package com.ixume.udar.collisiondetection.multithreading

import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor

/**
 * @param total Number of tasks that must be split up between threads
 * @param setup Must reset the callable, set its start and end
 */
inline fun <reified T : Runnable> Executor.runPartitioned(
    total: Int,
    callables: Array<T>,
    setup: T.(start: Int, end: Int) -> Unit,
) {
    var last = 0
    val num = callables.size
    val latch = CountDownLatch(num)
    val per = (total / num).coerceAtLeast(1)
    for (proc in 0..<num) {
        val start = last
        last = if (proc == num - 1) {
            total
        } else {
            (per * (proc + 1)).coerceAtMost(total)
        }

        val callable = callables[proc]
        callable.setup(start, last)

        execute {
            callable.run()

            latch.countDown()
        }
    }

    latch.await()
}