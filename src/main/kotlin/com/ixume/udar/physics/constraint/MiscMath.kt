package com.ixume.udar.physics.constraint

import kotlin.math.PI

object MiscMath {
    fun Float.toDegrees(): Float = this * 180f / PI.toFloat()
    fun Float.toRadians(): Float = this * PI.toFloat() / 180f
}