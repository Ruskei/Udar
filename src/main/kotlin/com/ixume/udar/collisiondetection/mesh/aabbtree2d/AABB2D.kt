package com.ixume.udar.collisiondetection.mesh.aabbtree2d

import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.testing.debugConnect
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d
import java.lang.Double.max
import java.lang.Double.min

@JvmInline
value class AABB2D(val data: DoubleArray) {
    fun overlaps(other: AABB2D): Boolean {
        return overlaps(other.data[MIN_A], other.data[MIN_B], other.data[MAX_A], other.data[MAX_B])
    }

    fun overlaps(minX: Double, minY: Double, maxX: Double, maxY: Double): Boolean {
        return data[MIN_A] < maxX && data[MAX_A] > minX &&
               data[MIN_B] < maxY && data[MAX_B] > minY
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

    fun minA(): Double {
        return data[MIN_A]
    }

    fun minB(): Double {
        return data[MIN_B]
    }

    fun maxA(): Double {
        return data[MAX_A]
    }

    fun maxB(): Double {
        return data[MAX_B]
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
        
        fun withLevel(axis: LocalMesher.AxisD, a: Double, b: Double, level: Double): Vector3d {
            return when (axis) {
                LocalMesher.AxisD.X -> Vector3d(level, a, b)
                LocalMesher.AxisD.Y -> Vector3d(a, level, b)
                LocalMesher.AxisD.Z -> Vector3d(a, b, level)
            }
        }
    }
}