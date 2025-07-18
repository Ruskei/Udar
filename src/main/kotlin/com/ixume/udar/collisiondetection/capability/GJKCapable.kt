package com.ixume.udar.collisiondetection.capability

import org.joml.Vector3d

interface GJKCapable {
    fun support(dir: Vector3d): Vector3d
}