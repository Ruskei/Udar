package com.ixume.udar.collisiondetection.mesh

import org.joml.Vector2d
import org.joml.Vector3d

data class MeshFace(
    val axis: Axis,
    val start: Vector3d,
    val end: Vector3d,
    val valid: MutableList<MeshFacePass>,
    val invalid: MutableList<Pair<Vector2d, Vector2d>>,
    val level: Double,
)

data class MeshFacePass(
    val start: Vector2d,
    val end: Vector2d,
    val inAxisDir: Boolean,
)
