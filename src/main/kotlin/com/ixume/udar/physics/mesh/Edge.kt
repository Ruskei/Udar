package com.ixume.udar.physics.mesh

import org.joml.Vector3d

data class Edge(
    val start: Vector3d,
    val end: Vector3d,
    val mount: EdgeMount? = null,
    val axis: Axis? = null,
) {
    val vec: Vector3d = Vector3d(end).sub(start)
}