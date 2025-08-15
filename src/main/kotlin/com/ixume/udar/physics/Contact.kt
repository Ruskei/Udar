package com.ixume.udar.physics

import com.ixume.udar.body.Body
import org.joml.Vector3d

class Contact(
    val first: Body,
    val second: Body,
    val result: CollisionResult,
    var lambdaSum: Double = 0.0
) {
    val t1: Vector3d = Vector3d(1.0).orthogonalizeUnit(result.norm)
    var t1Sum: Double = 0.0
    val t2: Vector3d = Vector3d(t1).cross(result.norm).normalize()
    var t2Sum: Double = 0.0
}