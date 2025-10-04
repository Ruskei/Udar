package com.ixume.udar.collisiondetection.mesh.quadtree

import it.unimi.dsi.fastutil.doubles.DoubleAVLTreeSet
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList

class EdgeData {
    val _points = DoubleArrayList()
    val points = DoubleAVLTreeSet()
    lateinit var finalizedPoints: DoubleArray
    val pointMounts = IntArrayList()

    fun finalizePoints() {
        finalizedPoints = DoubleArray(points.size)

        val itr = points.doubleIterator()
        var i = 0
        while (itr.hasNext()) {
            finalizedPoints[i] = itr.nextDouble()
            i++
        }
    }
}