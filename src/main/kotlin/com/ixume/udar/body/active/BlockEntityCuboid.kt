package com.ixume.udar.body.active

import com.ixume.udar.collisiondetection.capability.GJKCapable
import com.ixume.udar.collisiondetection.capability.SDFCapable
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.util.Transformation
import org.joml.Quaternionf
import org.joml.Vector2d
import org.joml.Vector3d
import org.joml.Vector3f

class BlockEntityCuboid(
    val cuboid: Cuboid,
    material: Material,
) : ActiveBody by cuboid, GJKCapable by cuboid, SDFCapable by cuboid {
    private val display: BlockDisplay = world.spawnEntity(
        Location(world, pos.x, pos.y, pos.z),
        EntityType.BLOCK_DISPLAY
    ) as BlockDisplay

    init {
        display.block = material.createBlockData()

        display.transformation = createTransformation()
        display.interpolationDuration = 1
        display.interpolationDelay = 0
        display.teleportDuration = 1
    }

    private fun createTransformation(): Transformation {
        val scale = Vector3f(cuboid.scale.x.toFloat(), cuboid.scale.y.toFloat(), cuboid.scale.z.toFloat())
        val rot = Quaternionf(q.x.toFloat(), q.y.toFloat(), q.z.toFloat(), q.w.toFloat())
        return Transformation(
            Vector3f(-0.5f).mul(scale).rotate(rot),
            rot,
            scale,
            Quaternionf(),
        )
    }

    override fun visualize() {
        display.transformation = createTransformation()
        display.teleport(Location(world, pos.x, pos.y, pos.z))
    }

    override fun project(axis: Vector3d): Vector2d {
        return cuboid.project(axis)
    }

    override fun onKill() {
        cuboid.onKill()
        display.remove()
    }
}

fun Cuboid.blockEntity(material: Material): BlockEntityCuboid {
    return BlockEntityCuboid(this, material)
}