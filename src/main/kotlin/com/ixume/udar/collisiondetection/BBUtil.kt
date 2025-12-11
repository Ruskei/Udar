package com.ixume.udar.collisiondetection

import org.bukkit.World
import org.bukkit.util.BoundingBox
import org.joml.Vector3d
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

fun boundingBox(
    vertices: List<Vector3d>,
): BoundingBox? {
    if (vertices.isEmpty()) return null
    val min = Vector3d(Double.MAX_VALUE)
    val max = Vector3d(-Double.MAX_VALUE)

    for (vertex in vertices) {
        min.set(
            min(min.x, vertex.x),
            min(min.y, vertex.y),
            min(min.z, vertex.z),
        )

        max.set(
            max(max.x, vertex.x),
            max(max.y, vertex.y),
            max(max.z, vertex.z),
        )
    }

    return BoundingBox(
        min.x, min.y, min.z,
        max.x, max.y, max.z
    )
}

/**
 * Returns list of all the BLOCK bounding boxes that overlap with this bounding box
 */
fun BoundingBox.overlappingBlocks(
    world: World,
): List<BoundingBox> {
    val bbs = mutableListOf<BoundingBox>()

    var x = minX
    while (x < maxX + 1.0) {
        var y = minY
        while (y < maxY + 1.0) {
            var z = minZ
            while (z < maxZ + 1.0) {
                val block = world.getBlockAt(
                    floor(x).toInt(),
                    floor(y).toInt(),
                    floor(z).toInt()
                )

                if (block.isPassable) {
                    z++
                    continue
                }

                if (block.boundingBox.overlaps(this)) {
                    val local = this.clone().shift(-block.location.x, -block.location.y, -block.location.z)
                    if (block.collisionShape.overlaps(local)) {
                        for (bb in block.collisionShape.boundingBoxes) {
                            if (bb.overlaps(local)) {
                                val bb2 = bb.clone()
                                bb2.shift(block.location)
                                bbs += bb2
                            }
                        }
                    }
                }

                z++
            }
            y++
        }
        x++
    }

    return bbs
}