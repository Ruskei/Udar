package com.ixume.udar.body.active

import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import org.joml.Vector3d

/**
 * VERTICES MUST BE WOUND COUNTER-CLOCKWISE
 */
class Face(val vertices: Array<Vector3d>) {
    init {
        check(vertices.size >= 3)
    }

    val normal = Vector3d()

    fun updateNormal() {
        val first = vertices[0]
        val last = vertices[vertices.size - 1]

        normal.set(vertices[1]).sub(first).cross(last.x - first.x, last.y - first.y, last.z - first.z).normalize()
    }

    fun point(): Vector3d {
        return vertices[0]
    }

    fun populate(out: DoubleArrayList) {
        var i = 0
        while (i < vertices.size) {
            val v = vertices[i]

            out.add(v.x)
            out.add(v.y)
            out.add(v.z)

            i++
        }
    }

    fun center(out: Vector3d) {
        out.set(0.0)
        val n = vertices.size.toDouble()

        var i = 0
        while (i < vertices.size) {
            val v = vertices[i]
            out.add(v.x / n, v.y / n, v.z / n)

            i++
        }
    }

    override fun toString(): String {
        return "[${vertices.joinToString { it.toString() }}]"
    }
}