package com.ixume.udar.collisiondetection.pool

interface Pool<T> {
    fun get(): T
    fun put(element: T)
}