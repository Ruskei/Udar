package com.ixume.udar.collisiondetection.mesh.quadtree

class EdgeConnection(
    val tree: FlattenedEdgeQuadtree,
    val otherFaceID: Long,
    val min: Double, val max: Double,
)