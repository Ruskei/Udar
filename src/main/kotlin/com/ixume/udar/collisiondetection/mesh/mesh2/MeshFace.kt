package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.FlattenedAABBTree2D
import com.ixume.udar.collisiondetection.mesh.quadtree.EdgeConnection
import com.ixume.udar.dynamicaabb.AABB
import org.bukkit.World

class MeshFace(
    val axis: LocalMesher.AxisD,
    val level: Double,

    val holes: FlattenedAABBTree2D,
) : Comparable<MeshFace> {
    lateinit var antiHoles: FlattenedAABBTree2D
    val edgeConnections = mutableListOf<EdgeConnection>()

    var idx = -1
    var id: Long = -1

    fun visualize(world: World) {
        holes.visualize(world, true)
        antiHoles.visualize(world, false)
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