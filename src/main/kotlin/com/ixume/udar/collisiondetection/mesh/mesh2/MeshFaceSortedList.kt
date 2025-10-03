package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.FlattenedAABBTree2D
import com.ixume.udar.dynamicaabb.AABB
import org.joml.Vector3i
import kotlin.math.abs

class MeshFaceSortedList(
    val axis: LocalMesher.AxisD,
    val meshStart: Vector3i,
    val meshEnd: Vector3i,
) {
    val ls = mutableListOf<MeshFace>()

    fun placeFaceAt(level: Double): MeshFace? {
        if (level <= meshStart.get(axis.levelOffset) - 1.0 || level >= meshEnd.get(axis.levelOffset) + 2.0) return null

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

        val face = MeshFace(
            axis = axis,
            level = level,
            holes = FlattenedAABBTree2D(0, level, axis),
        )

        ls.add(low, face)

        return face
    }

    fun getFaceIdxAt(level: Double): Int {
        var low = 0
        var high = ls.size - 1

        while (low <= high) {
            val mid = (low + high) ushr 1
            val midFace = ls[mid]

            if (abs(midFace.level - level) < FACE_EPSILON) {
                return mid
            }

            if (midFace.level < level) {
                low = mid + 1
            } else {
                high = mid - 1
            }
        }

        return -1
    }

    inline fun forEachOverlapping(bb: AABB, action: (MeshFace) -> Unit) {
        // find smallest (smallest level that fits in bb), then iterate until no longer fits
        val levelMin = when (axis) {
            LocalMesher.AxisD.X -> bb.minX
            LocalMesher.AxisD.Y -> bb.minY
            LocalMesher.AxisD.Z -> bb.minZ
        }

        val levelMax = when (axis) {
            LocalMesher.AxisD.X -> bb.maxX
            LocalMesher.AxisD.Y -> bb.maxY
            LocalMesher.AxisD.Z -> bb.maxZ
        }

        var low = 0
        var high = ls.size - 1

        while (low <= high) {
            val mid = (low + high) ushr 1
            val midFace = ls[mid]

            if (midFace.level < levelMin) {
                low = mid + 1
            } else {
                high = mid - 1
            }
        }

        // low == index of min face...
        var i = low
        while (i < ls.size) {
            val face = ls[i]
            if (face.level > levelMax) return

            action(face)

            i++
        }
    }

    private fun constructAntiHoles() {
        for (face in ls) {
            face.antiHoles = face.holes.constructCollisions()
        }
    }

    fun finish() {
        constructAntiHoles()

        var i = 0
        while (i < ls.size) {
            val f = ls[i]
            f.idx = i
            f.id = faceID(axis, i)
            i++
        }
    }

    companion object {
        private const val FACE_EPSILON = 1e-8f
    }
}