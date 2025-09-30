package com.ixume.udar.dynamicaabb.array

import kotlin.math.max

class IntStack {
    private var size = 0
    private var arr = IntArray(0)

    fun enqueue(i: Int) {
        size++
        grow(size)

        arr[size - 1] = i
    }

    /**
     * Returns -INT.MAX_VALUE if array is empty
     */
    fun dequeue(): Int {
        return arr[--size]
    }

    fun hasNext(): Boolean {
        return size > 0
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)

        arr = arr.copyOf(newSize)
    }
}