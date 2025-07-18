package com.ixume.udar.collisiondetection.capability

import org.joml.Vector3d

interface SDFCapable {
    fun distance(p: Vector3d): Double
}