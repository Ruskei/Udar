package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.FlattenedAABBTree2D
import com.ixume.udar.collisiondetection.mesh.quadtree.QuadtreeEdge
import org.bukkit.World

class MeshFace(
    val axis: LocalMesher.AxisD,
    val level: Double,

    val holes: FlattenedAABBTree2D,
) : Comparable<MeshFace> {
    lateinit var antiHoles: FlattenedAABBTree2D
    val edges = ArrayList<QuadtreeEdge>()

    fun visualize(world: World) {
        if (axis == LocalMesher.AxisD.X) {
            holes.visualize(world, true)
            antiHoles.visualize(world, false)
        }
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
}