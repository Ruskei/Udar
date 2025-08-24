package com.ixume.udar.collisiondetection.mesh.quadtree

import it.unimi.dsi.fastutil.doubles.DoubleAVLTreeSet

class QuadtreeEdge {
    val points = DoubleAVLTreeSet()
    private val xoredPoints = DoubleAVLTreeSet() // if a point has been xor'd out, then future calls must be invalid

    //    val points = DoubleArrayList()
//    lateinit var fixedUp: DoubleAVLTreeSet
//
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