package com.ixume.udar.util

import java.util.concurrent.atomic.AtomicReference

class AtomicList<T> {
    val items = AtomicReference<List<T>>(emptyList())

    fun add(element: T) {
        do {
            val curr = items.get()!!
            val n = curr + element
        } while (!items.compareAndSet(curr, n))
    }

    fun addAll(elements: Collection<T>) {
        do {
            val curr = items.get()!!
            val n = curr + elements
        } while (!items.compareAndSet(curr, n))
    }

    operator fun plusAssign(element: T) {
        add(element)
    }

    operator fun plusAssign(elements: Collection<T>) {
        addAll(elements)
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

    inline fun removeAll(filter: (T) -> Boolean) {
        do {
            val curr = items.get()
            val n = curr.filterNot(filter)
        } while (!items.compareAndSet(curr, n))
    }

    fun getAndClear(): List<T> {
        return items.getAndSet(emptyList())
    }

    fun clear() {
        do {
            val curr = items.get()!!
            if (curr.isEmpty()) return
        } while (!items.compareAndSet(curr, emptyList()))
    }

    override fun toString() = items.toString()
}