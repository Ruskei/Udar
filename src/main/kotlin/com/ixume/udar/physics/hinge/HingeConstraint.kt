package com.ixume.udar.physics.hinge

import com.ixume.udar.body.active.ActiveBody

/**
 * Limits rotation between bodies b1 and b2 to be only around a1, a2
 * n1 and n2 are reference axes for measuring hinge angle limits
 */
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

    val r1x: Float,
    val r1y: Float,
    val r1z: Float,

    val r2x: Float,
    val r2y: Float,
    val r2z: Float,
)