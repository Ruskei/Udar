package com.ixume.udar.util

import java.util.concurrent.atomic.AtomicReference

/**
 * Must only be read from a single thread, and must only have the `update` method called from a single thread, but supports concurrent deletions and removals via a queue
 */
class ConcurrentBuffer<T> {
    private val items: AtomicReference<Pair<List<T>, Boolean>> = AtomicReference(ArrayList<T>() to false)

    fun add(elem: T) {
        do {
            val curr = items.get()!!
            val n = (curr.first + elem) to true
        } while (!items.compareAndSet(curr, n))
    }

    fun remove(elem: T) {
        do {
            val curr = items.get()!!
            val n = (curr.first - elem) to true
        } while (!items.compareAndSet(curr, n))
    }

    fun sync(): Pair<List<T>, Boolean> {
        var n: Pair<List<T>, Boolean>
        do {
            val curr = items.get()
            n = curr.first to false
        } while (!items.compareAndSet(curr, n))

        return n
    }
}