package com.ixume.udar.collisiondetection.mesh.quadtree

import com.ixume.udar.collisiondetection.mesh.LocalMesher
import it.unimi.dsi.fastutil.doubles.DoubleAVLTreeSet

class QuadtreeEdge(
    val f1: LocalMesher.MeshFace,
    val f2: LocalMesher.MeshFace,
) {
    init {
        f1.edges += this
        f2.edges += this
    }

    val points = DoubleAVLTreeSet()
    private val xoredPoints = DoubleAVLTreeSet() // if a point has been xor'd out, then future calls must be invalid

    fun xor(a: Double, b: Double) {
        if (xoredPoints.contains(a) || xoredPoints.contains(b)) return
        _xor(a)
        _xor(b)
    }

    private fun _xor(d: Double) {
        if (!points.remove(d)) {
            points.add(d)
        } else {
            xoredPoints.add(d)
        }
    }
}