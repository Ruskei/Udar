package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABBTree2D
import com.ixume.udar.collisiondetection.mesh.quadtree.QuadtreeEdge
import org.bukkit.World

class MeshFace(
    val axis: LocalMesher.AxisD,
    val level: Double,

    val holes: AABBTree2D,
) : Comparable<MeshFace> {
    lateinit var antiHoles: AABBTree2D
    val edges = ArrayList<QuadtreeEdge>()

    fun overlappingHoles(bb: AABB2D): List<AABB2D> {
        return holes.overlaps(bb)
    }

    fun overlappingAntiHoles(bb: AABB2D): List<AABB2D> {
        return antiHoles.overlaps(bb)
    }

    fun visualize(world: World) {
//        holes.visualize(world, level, axis, true)
//        antiHoles.visualize(world, level, axis, false)
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