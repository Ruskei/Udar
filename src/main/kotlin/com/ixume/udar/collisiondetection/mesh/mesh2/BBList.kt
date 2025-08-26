package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.dynamicaabb.AABB
import it.unimi.dsi.fastutil.doubles.DoubleArrayList

class BBList {
    private val elements = DoubleArrayList()

    val size: Int
        get() {
            return elements.size / 6
        }

    fun add(aabb: AABB) {
        elements.add(aabb.minX)
        elements.add(aabb.minY)
        elements.add(aabb.minZ)
        elements.add(aabb.maxX)
        elements.add(aabb.maxY)
        elements.add(aabb.maxZ)
    }

    fun clear() {
        elements.clear()
    }

    fun get(idx: Int, out: DoubleArray) {
        elements.getElements(idx * 6, out, 0, 6)
    }
}