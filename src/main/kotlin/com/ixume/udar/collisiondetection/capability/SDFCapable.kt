package com.ixume.udar.collisiondetection.capability

import org.joml.Vector3d

interface SDFCapable : Projectable {
    fun distance(p: Vector3d): Double
    fun gradient(p: Vector3d): Vector3d

    val startPoints: List<Vector3d>
}