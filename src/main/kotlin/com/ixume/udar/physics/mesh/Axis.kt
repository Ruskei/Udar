package com.ixume.udar.physics.mesh

import org.joml.Vector3d

enum class Axis(
    val vec: Vector3d
) {
    X(Vector3d(1.0, 0.0, 0.0)), Y(Vector3d(0.0, 1.0, 0.0)), Z(Vector3d(0.0, 0.0, 1.0))
}
