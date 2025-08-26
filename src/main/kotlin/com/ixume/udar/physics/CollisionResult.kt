package com.ixume.udar.physics

import org.joml.Vector3d

data class CollisionResult (
    val pointA: Vector3d,
    val pointB: Vector3d,
    val norm: Vector3d,
    val depth: Double,
)