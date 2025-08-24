package com.ixume.udar.collisiondetection.mesh.quadtree

import com.ixume.udar.collisiondetection.mesh.LocalMesher
import com.ixume.udar.dynamicaabb.AABBTree
import org.bukkit.World
import org.joml.Vector2d

/**
 * Stores DoubleArray's representing valid edge levels on axiss
 */
class EdgeQuadtree(
    val min: Vector2d,
    val max: Vector2d,
) {
    val root = EdgeQuadtreeNode(min, max)

    fun insertEdge(x: Double, y: Double, start: Double, end: Double, axis: LocalMesher.AxisD, meshFaces: LocalMesher.MeshFaces) {
        check(root.insertEdge(x, y, start, end - ASYMMETRY_EPSILON, axis, meshFaces)) { "Failed to insert ($x, $y) , $start -> $end "}
    }

    fun fixUp(bbTree: AABBTree, axis: LocalMesher.AxisD) {
        root.clearConcave(bbTree, axis)
    }

    fun visualize(world: World, axis: LocalMesher.AxisD) {
        root.visualize(world, axis)
    }
}

private const val ASYMMETRY_EPSILON = 1e-10
