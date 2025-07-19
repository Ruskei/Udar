package com.ixume.udar.physics

import com.ixume.udar.body.Body
import org.joml.Vector3d

interface IContact {
    val first: Body
    val second: Body
    val result: CollisionResult
    var lambdaSum: Double
    val t1: Vector3d
    var t1Sum: Double
    val t2: Vector3d
    var t2Sum: Double
}

class Contact(
    override val first: Body,
    override val second: Body,
    override val result: CollisionResult,
    override var lambdaSum: Double = 0.0
) : IContact {
    override val t1: Vector3d = Vector3d(1.0).orthogonalizeUnit(result.norm)
    override var t1Sum: Double = 0.0
    override val t2: Vector3d = Vector3d(t1).cross(result.norm).normalize()
    override var t2Sum: Double = 0.0
}