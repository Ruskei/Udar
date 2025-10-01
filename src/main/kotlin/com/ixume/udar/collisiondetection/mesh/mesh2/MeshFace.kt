package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.FlattenedAABBTree2D
import com.ixume.udar.collisiondetection.mesh.quadtree.EdgeConnection
import com.ixume.udar.dynamicaabb.AABB
import org.bukkit.Color
import org.bukkit.World
import kotlin.math.sqrt
import kotlin.random.Random

class MeshFace(
    val axis: LocalMesher.AxisD,
    val level: Double,

    val holes: FlattenedAABBTree2D,
) : Comparable<MeshFace> {
    private val color = run {
        var r = Random.nextDouble()
        var g = Random.nextDouble()
        var b = Random.nextDouble()
        val n = sqrt(r * r + g * g + b * b)
        r /= n
        g /= n
        b /= n
        Color.fromRGB(
            (r * 255.0).toInt().coerceIn(0, 255),
            (g * 255.0).toInt().coerceIn(0, 255),
            (b * 255.0).toInt().coerceIn(0, 255),
        )
    }
    
    lateinit var antiHoles: FlattenedAABBTree2D
    val edgeConnections = mutableListOf<EdgeConnection>()

    var idx = -1
    var id: Long = -1

    fun visualize(world: World) {
        holes.visualize(world, color)
//        antiHoles.visualize(world, false)
    }

    override fun compareTo(other: MeshFace): Int {
        return if (level > other.level) {
            1
        } else if (level == other.level) {
            0
        } else {
            -1
        }
    }
    
    fun overlaps(bb: AABB): Boolean {
        return when (axis) {
            LocalMesher.AxisD.X -> level in bb.minX..bb.maxX
            LocalMesher.AxisD.Y -> level in bb.minY..bb.maxY
            LocalMesher.AxisD.Z -> level in bb.minZ..bb.maxZ
        }
    }
}