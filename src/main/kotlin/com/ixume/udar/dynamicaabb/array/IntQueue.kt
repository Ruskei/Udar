package com.ixume.udar.dynamicaabb.array

import kotlin.math.max

class IntQueue {
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
        size--
        
        if (size < 0) return -Int.MAX_VALUE
        
        return arr[size]
    }
    
    fun hasNext(): Boolean {
        return size > 0
    }
    
    private fun grow(required: Int) {
        if (arr.size >= required) return
        
        val newSize = max(required, arr.size * 2)
        
        arr = arr.copyOf(newSize)
    }
}