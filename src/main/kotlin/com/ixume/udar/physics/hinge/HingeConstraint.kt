package com.ixume.udar.physics.hinge

import com.ixume.udar.body.active.ActiveBody

data class HingeConstraint(
    val b1: ActiveBody,
    val b2: ActiveBody,

    val a1x: Float,
    val a1y: Float,
    val a1z: Float,

    val a2x: Float,
    val a2y: Float,
    val a2z: Float,

    val n1x: Float,
    val n1y: Float,
    val n1z: Float,

    val n2x: Float,
    val n2y: Float,
    val n2z: Float,

    val min: Float,
    val max: Float,

    val p1x: Float,
    val p1y: Float,
    val p1z: Float,

    val p2x: Float,
    val p2y: Float,
    val p2z: Float,
)