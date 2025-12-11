package com.ixume.udar.physics.cone

import com.ixume.udar.body.active.ActiveBody

/**
 * b1 is parent
 * b2 is child
 * @param r1 The relative pivot point on body 1
 * @param r2 The relative pivot point on body 2
 * @param x1 The reference axis for body 1 that will be used to calculate θ
 * @param y1 The reference axis for body 1 that will be used to calculate φ
 * @param z2 The reference axis for body 2 that will be contained inside the cone
 * @param x2 The reference axis for body 2 that will be used to calculate twist relative to x1
 */
data class ConeConstraint(
    val b1: ActiveBody,
    val b2: ActiveBody,

    val r1x: Float,
    val r1y: Float,
    val r1z: Float,

    val r2x: Float,
    val r2y: Float,
    val r2z: Float,

    val x1x: Float,
    val x1y: Float,
    val x1z: Float,

    val y1x: Float,
    val y1y: Float,
    val y1z: Float,

    val x2x: Float,
    val x2y: Float,
    val x2z: Float,

    val z2x: Float,
    val z2y: Float,
    val z2z: Float,

    val maxXAngle: Float,
    val maxYAngle: Float,

    val minTwistAngle: Float,
    val maxTwistAngle: Float,
) {
    var λ3x31 = 0f
    var λ3x32 = 0f
    var λ3x33 = 0f
}