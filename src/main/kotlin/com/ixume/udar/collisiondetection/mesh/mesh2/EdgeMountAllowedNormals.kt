package com.ixume.udar.collisiondetection.mesh.mesh2

import org.joml.Vector2d

object EdgeMountAllowedNormals {
    val allowedNormals = arrayOf(
        arrayOf(Vector2d(-1.0, 0.0), Vector2d(0.0, -1.0)),
        arrayOf(Vector2d(-1.0, 0.0), Vector2d(0.0, 1.0)),
        arrayOf(Vector2d(1.0, 0.0), Vector2d(0.0, 1.0)),
        arrayOf(Vector2d(1.0, 0.0), Vector2d(0.0, -1.0)),
    )
}