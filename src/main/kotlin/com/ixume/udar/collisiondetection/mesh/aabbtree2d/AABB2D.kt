package com.ixume.udar.collisiondetection.mesh.aabbtree2d

import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.testing.debugConnect
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d
import java.lang.Double.max
import java.lang.Double.min

class AABB2D(
    val data: DoubleArray,
    var node: AABBNode2D? = null,
) {
    var inDir: Boolean = false

    val volume: Double
        get() {
            return (data[MAX_A] - data[MIN_A]) * (data[MAX_B] - data[MIN_B])
        }

    fun setUnion(a: AABB2D, b: AABB2D) {
        data[MIN_A] = min(a.data[MIN_A], b.data[MIN_A])
        data[MIN_B] = min(a.data[MIN_B], b.data[MIN_B])
        data[MAX_A] = max(a.data[MAX_A], b.data[MAX_A])
        data[MAX_B] = max(a.data[MAX_B], b.data[MAX_B])
    }

    fun refit(b: AABB2D) {
        data[MIN_A] = min(data[MIN_A], b.data[MIN_A])
        data[MIN_B] = min(data[MIN_B], b.data[MIN_B])
        data[MAX_A] = max(data[MAX_A], b.data[MAX_A])
        data[MAX_B] = max(data[MAX_B], b.data[MAX_B])
    }

    fun overlaps(other: AABB2D): Boolean {
        return overlaps(other.data[MIN_A], other.data[MIN_B], other.data[MAX_A], other.data[MAX_B])
    }

    fun calcOverlap(other: AABB2D): AABB2D {
        val arr = DoubleArray(4)
        arr[MIN_A] = max(data[MIN_A], other.data[MIN_A])
        arr[MIN_B] = max(data[MIN_B], other.data[MIN_B])
        arr[MAX_A] = min(data[MAX_A], other.data[MAX_A])
        arr[MAX_B] = min(data[MAX_B], other.data[MAX_B])

        return AABB2D(arr)
    }

    fun overlaps(minX: Double, minY: Double, maxX: Double, maxY: Double): Boolean {
        return data[MIN_A] < maxX && data[MAX_A] > minX
                && data[MIN_B] < maxY && data[MAX_B] > minY
    }

    fun contains(other: AABB2D): Boolean {
        return contains(other.data[MIN_A], other.data[MIN_B], other.data[MAX_A], other.data[MAX_B])
    }

    fun contains(minX: Double, minY: Double, maxX: Double, maxY: Double): Boolean {
        return data[MIN_A] <= minX && data[MAX_A] >= maxX && data[MIN_B] <= minY && data[MAX_A] >= maxY
    }

    fun contains(a: Double, b: Double): Boolean {
        return data[MIN_A] <= a && data[MAX_A] >= a && data[MIN_B] <= b && data[MAX_B] >= b
    }

    fun updateTree(tree: AABBTree2D) {
        node?.let {
            tree.remove(it)
        }

        tree.insert(this)
    }

    fun visualize(world: World, depth: Int, level: Double, axis: LocalMesher.AxisD, isHole: Boolean) {
        val options = if (isHole) Particle.DustOptions(Color.BLUE, 0.25f) else Particle.DustOptions(Color.RED, 0.25f)

        world.debugConnect(
            start = withLevel(axis, data[MIN_A], data[MIN_B], level),
            end = withLevel(axis, data[MAX_A], data[MIN_B], level),
            options = options,
        )
        world.debugConnect(
            start = withLevel(axis, data[MAX_A], data[MIN_B], level),
            end = withLevel(axis, data[MAX_A], data[MAX_B], level),
            options = options,
        )
        world.debugConnect(
            start = withLevel(axis, data[MAX_A], data[MAX_B], level),
            end = withLevel(axis, data[MIN_A], data[MAX_B], level),
            options = options,
        )
        world.debugConnect(
            start = withLevel(axis, data[MIN_A], data[MAX_B], level),
            end = withLevel(axis, data[MIN_A], data[MIN_B], level),
            options = options,
        )
    }

    override fun toString(): String {
        return "{[${data[MIN_A]}, ${data[MIN_B]}] -> [${data[MAX_A]}, ${data[MAX_B]}]}"
    }

    companion object {
        internal const val MIN_A = 0
        internal const val MIN_B = 1
        internal const val MAX_A = 2
        internal const val MAX_B = 3

        fun union(a: AABB2D, b: AABB2D): AABB2D {
            val arr = DoubleArray(6)
            arr[MIN_A] = min(a.data[MIN_A], b.data[MIN_A])
            arr[MIN_B] = min(a.data[MIN_B], b.data[MIN_B])
            arr[MAX_A] = max(a.data[MAX_A], b.data[MAX_A])
            arr[MAX_B] = max(a.data[MAX_B], b.data[MAX_B])
            return AABB2D(data = arr)
        }

        fun unifiedCost(a: AABB2D, b: AABB2D): Double {
            val minX = min(a.data[MIN_A], b.data[MIN_A])
            val minY = min(a.data[MIN_B], b.data[MIN_B])
            val maxX = max(a.data[MAX_A], b.data[MAX_A])
            val maxY = max(a.data[MAX_B], b.data[MAX_B])

            return (maxX - minX) * (maxY - minY)
        }

        fun withLevel(axis: LocalMesher.AxisD, a: Double, b: Double, level: Double): Vector3d {
            return when (axis) {
                LocalMesher.AxisD.X -> Vector3d(level, a, b)
                LocalMesher.AxisD.Y -> Vector3d(a, level, b)
                LocalMesher.AxisD.Z -> Vector3d(a, b, level)
            }
        }
    }
}