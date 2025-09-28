package com.ixume.udar.collisiondetection.mesh.mesh2

class MeshFaces(
    val xFaces: MeshFaceSortedList,
    val yFaces: MeshFaceSortedList,
    val zFaces: MeshFaceSortedList,
) {
    val xs = xFaces.ls.size
    val ys = yFaces.ls.size
    val zs = zFaces.ls.size

    val size = xs + ys + zs
    var cursor = 0

    fun resetIterator() {
        cursor = 0
    }

    fun hasNext(): Boolean {
        return cursor + 1 < size
    }

    fun next(): MeshFace {
        val face = if (cursor < xs) {
            xFaces.ls[cursor]
        } else if (cursor - xs < ys) {
            yFaces.ls[cursor - xs]
        } else {
            zFaces.ls[cursor - xs - ys]
        }

        cursor++

        return face
    }
}