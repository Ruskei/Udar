package com.ixume.udar.collisiondetection.mesh.quadtree

import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaces
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.dynamicaabb.AABBTree
import org.bukkit.World
import org.joml.Vector2d

/**
 * Stores DoubleArray's representing valid edge levels on axiss
 */
class EdgeQuadtree(
    val axis: LocalMesher.AxisD,
    val min: Vector2d,
    val max: Vector2d,
) {
    val root = EdgeQuadtreeNode(min, max)

    fun insertEdge(x: Double, y: Double, start: Double, end: Double, meshFaces: MeshFaces) {
        check(root.insertEdge(x, y, start, end - ASYMMETRY_EPSILON, axis, meshFaces)) { "Failed to insert ($x, $y) , $start -> $end "}
    }

    fun fixUp(bbTree: AABBTree) {
        root.fixUp(bbTree, axis)
    }

    fun overlaps(bb: AABB): List<QuadtreeEdge> {
        val ls = mutableListOf<QuadtreeEdge>()

        root.overlaps(bb, axis, ls)

        return ls
    }

    fun visualize(world: World) {
        root.visualize(world, axis)
    }
}

const val ASYMMETRY_EPSILON = 1e-12
const val ABOVE_ASYMMETRY_EPSILON = 1e-11
