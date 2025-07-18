package com.ixume.udar.physics

import com.ixume.udar.body.ActiveBody
import org.joml.Vector3d

data class Contact(
    val first: ActiveBody,
    val second: ActiveBody,
    val result: CollisionResult,
    var lambdaSum: Double = 0.0
) {
    val t1: Vector3d = Vector3d(1.0).orthogonalizeUnit(result.norm)
    var t1Sum: Double = 0.0
    val t2: Vector3d = Vector3d(t1).cross(result.norm).normalize()
    var t2Sum: Double = 0.0
}