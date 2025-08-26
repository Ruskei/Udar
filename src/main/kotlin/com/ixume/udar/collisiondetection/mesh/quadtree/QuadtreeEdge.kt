package com.ixume.udar.collisiondetection.mesh.quadtree

import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFace
import it.unimi.dsi.fastutil.doubles.DoubleAVLTreeSet
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList

class QuadtreeEdge(
    val f1: MeshFace,
    val f2: MeshFace,
    val a: Double,
    val b: Double,
) {
    init {
        f1.edges += this
        f2.edges += this
    }

    val _points = DoubleAVLTreeSet()
    val points = DoubleArrayList()

    val pointMounts = IntArrayList()

    private val xoredPoints = DoubleAVLTreeSet() // if a point has been xor'd out, then future calls must be invalid

    fun xor(a: Double, b: Double) {
        if (xoredPoints.contains(a) || xoredPoints.contains(b)) return
        _xor(a)
        _xor(b)
    }

    private fun _xor(d: Double) {
        if (!_points.remove(d)) {
            _points.add(d)
        } else {
            xoredPoints.add(d)
        }
    }
}