package com.ixume.udar.collisiondetection.mesh

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABBTree2D
import kotlin.math.abs

class MeshFaceSortedList(
    val axis: LocalMesher.AxisD,
) {
    val ls = mutableListOf<LocalMesher.MeshFace>()

    fun placeFaceAt(level: Double): LocalMesher.MeshFace {
        var low = 0
        var high = ls.size - 1

        while (low <= high) {
            val mid = (low + high) ushr 1
            val midFace = ls[mid]

            if (abs(midFace.level - level) < FACE_EPSILON) {
                return midFace
            }

            if (midFace.level < level) {
                low = mid + 1
            } else {
                high = mid - 1
            }
        }

        val face = LocalMesher.MeshFace(
            axis = axis,
            level = level,
            holes = AABBTree2D(),
        )

        ls.add(low, face)

        return face
    }

    fun getFaceAt(level: Double): LocalMesher.MeshFace? {
        var low = 0
        var high = ls.size - 1

        while (low <= high) {
            val mid = (low + high) ushr 1
            val midFace = ls[mid]

            if (abs(midFace.level - level) < FACE_EPSILON) {
                return midFace
            }

            if (midFace.level < level) {
                low = mid + 1
            } else {
                high = mid - 1
            }
        }

        return null
    }

    fun constructAntiHoles() {
        for (face in ls) {
            face.antiHoles = face.holes.collisions()
        }
    }

    companion object {
        private const val FACE_EPSILON = 1e-8f
    }
}