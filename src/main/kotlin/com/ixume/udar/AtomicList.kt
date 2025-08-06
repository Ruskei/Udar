package com.ixume.udar

import java.util.concurrent.atomic.AtomicReference

class AtomicList<T> {
    private val items = AtomicReference<List<T>>(emptyList())

    fun add(element: T) {
        do {
            val curr = items.get()!!
            val n = curr + element
        } while (!items.compareAndSet(curr, n))
    }

    operator fun plusAssign(element: T) {
        add(element)
    }

    operator fun minusAssign(element: T) {
        remove(element)
    }

    operator fun minusAssign(elements: Collection<T>) {
        removeAll(elements)
    }

    fun remove(element: T) {
        do {
            val curr = items.get()!!

            if (element !in curr) return

            val n = curr - element
        } while (!items.compareAndSet(curr, n))
    }

    fun removeAll(elements: Collection<T>) {
        do {
            val curr = items.get()!!
            val n = curr - elements
        } while (!items.compareAndSet(curr, n))
    }

    fun get(): List<T> {
        return items.get()
    }

    fun clear() {
        do {
            val curr = items.get()!!
            if (curr.isEmpty()) return
        } while (!items.compareAndSet(curr, emptyList()))
    }
}