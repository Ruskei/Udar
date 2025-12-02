package com.ixume.udar.physics.position

import com.ixume.udar.body.active.ActiveBody

data class PointConstraint(
    val b1: ActiveBody,
    val b2: ActiveBody,

    val r1x: Float,
    val r1y: Float,
    val r1z: Float,

    val r2x: Float,
    val r2y: Float,
    val r2z: Float,
)