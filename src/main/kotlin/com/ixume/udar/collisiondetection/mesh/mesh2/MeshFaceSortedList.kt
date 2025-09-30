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
    val ls = mutableListOf<MeshFace>() // TODO: CHANGE TO BINARY HEAP!

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

    fun getFaceAt(level: Double): MeshFace? {
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

    fun facesIn(bb: AABB, out: MutableList<MeshFace>): List<MeshFace> {
        when (axis) {
            LocalMesher.AxisD.X -> {
                var i = 0
                while (i < ls.size) {
                    val f = ls[i]
                    if (f.level >= bb.minX && f.level <= bb.maxX) {
                        out += f
                    }

                    i++
                }
            }

            LocalMesher.AxisD.Y -> {
                var i = 0
                while (i < ls.size) {
                    val f = ls[i]
                    if (f.level >= bb.minY && f.level <= bb.maxY) {
                        out += f
                    }

                    i++
                }
            }

            LocalMesher.AxisD.Z -> {
                var i = 0
                while (i < ls.size) {
                    val f = ls[i]
                    if (f.level >= bb.minZ && f.level <= bb.maxZ) {
                        out += f
                    }

                    i++
                }
            }
        }

        return out
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