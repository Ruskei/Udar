package com.ixume.udar.dynamicaabb

import com.ixume.udar.testing.debugConnect
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d

open class AABB(
    var minX: Double,
    var minY: Double,
    var minZ: Double,
    var maxX: Double,
    var maxY: Double,
    var maxZ: Double,
) {
    fun overlaps(other: AABB): Boolean {
        return overlaps(other.minX, other.minY, other.minZ, other.maxX, other.maxY, other.maxZ)
    }

    fun overlaps(minX: Double, minY: Double, minZ: Double, maxX: Double, maxY: Double, maxZ: Double): Boolean {
        return this.minX < maxX && this.maxX > minX &&
               this.minY < maxY && this.maxY > minY &&
               this.minZ < maxZ && this.maxZ > minZ
    }

    fun contains(other: AABB): Boolean {
        return contains(other.minX, other.minY, other.minZ, other.maxX, other.maxY, other.maxZ)
    }

    fun contains(minX: Double, minY: Double, minZ: Double, maxX: Double, maxY: Double, maxZ: Double): Boolean {
        return this.minX <= minX && this.maxX >= maxX && this.minY <= minY && this.maxY >= maxY && this.minZ <= minZ && this.maxZ >= maxZ
    }

    fun contains(x: Double, y: Double, z: Double): Boolean {
        return x >= minX && x <= maxX &&
               y >= minY && y <= maxY &&
               z >= minZ && z <= maxZ
    }

    fun writeTo(other: AABB) {
        other.minX = minX
        other.minY = minY
        other.minZ = minZ
        other.maxX = maxX
        other.maxY = maxY
        other.maxZ = maxZ
    }

    override fun toString(): String {
        return "{min: ($minX $minY $minZ), max: ($maxX $maxY $maxZ)}"
    }
        
    fun visualize(world: World, depth: Int) {
        val options = options(depth)

        world.debugConnect(
            start = Vector3d(minX, minY, minZ),
            end = Vector3d(maxX, minY, minZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(minX, maxY, minZ),
            end = Vector3d(maxX, maxY, minZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(minX, maxY, maxZ),
            end = Vector3d(maxX, maxY, maxZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(minX, minY, maxZ),
            end = Vector3d(maxX, minY, maxZ),
            options = options,
        )

        world.debugConnect(
            start = Vector3d(minX, minY, minZ),
            end = Vector3d(minX, maxY, minZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(maxX, minY, minZ),
            end = Vector3d(maxX, maxY, minZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(maxX, minY, maxZ),
            end = Vector3d(maxX, maxY, maxZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(minX, minY, maxZ),
            end = Vector3d(minX, maxY, maxZ),
            options = options,
        )

        world.debugConnect(
            start = Vector3d(minX, minY, minZ),
            end = Vector3d(minX, minY, maxZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(minX, maxY, minZ),
            end = Vector3d(minX, maxY, maxZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(maxX, maxY, minZ),
            end = Vector3d(maxX, maxY, maxZ),
            options = options,
        )
        world.debugConnect(
            start = Vector3d(maxX, minY, minZ),
            end = Vector3d(maxX, minY, maxZ),
            options = options,
        )
    }

    companion object {
        const val FAT_MARGIN = 0.1
        private val colors = listOf(
            Color.WHITE,
            Color.SILVER,
            Color.GRAY,
            Color.BLACK,
            Color.RED,
            Color.MAROON,
            Color.YELLOW,
            Color.OLIVE,
            Color.LIME,
            Color.GREEN,
            Color.AQUA,
            Color.TEAL,
            Color.BLUE,
            Color.NAVY,
            Color.FUCHSIA,
            Color.PURPLE,
            Color.ORANGE,
        )

        private fun options(depth: Int): Particle.DustOptions {
            return Particle.DustOptions(colors[depth % colors.size], 0.25f)
        }
    }
}