package com.ixume.udar.collisiondetection.pool

import org.joml.Vector3d

class TrackingD3Pool(
    amount: Int
) : Pool<Vector3d> {
    private val items = ArrayDeque<Vector3d>(amount)
    private val tracked = ArrayDeque<Vector3d>()

    init {
        repeat(amount) {
            items += Vector3d()
        }
    }

    override fun get(): Vector3d {
        val e = if (items.isNotEmpty()) {
            items.removeLast()
        } else {
            Vector3d()
        }
        tracked += e
        return e
    }

    override fun put(element: Vector3d) {}

    fun clearTracked() {
        items.addAll(tracked)
        tracked.clear()
    }
}