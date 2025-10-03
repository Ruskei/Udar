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

private const val X_PRIME = 93960703L
private const val Y_PRIME = 56726983L
private const val Z_PRIME = 35230339L

fun faceID(axis: LocalMesher.AxisD, idx: Int): Long {
    val prime = when (axis) {
        LocalMesher.AxisD.X -> X_PRIME
        LocalMesher.AxisD.Y -> Y_PRIME
        LocalMesher.AxisD.Z -> Z_PRIME
    }

    return prime * idx
}
