package com.ixume.udar.collisiondetection.capability

import org.joml.Vector3d

interface Projectable {
    fun project(axis: Vector3d): Pair<Double, Double>
}