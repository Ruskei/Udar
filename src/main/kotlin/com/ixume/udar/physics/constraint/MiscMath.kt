package com.ixume.udar.physics.constraint

import kotlin.math.PI
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sqrt

object MiscMath {
    fun Float.toDegrees(): Float = this * 180f / PI.toFloat()
    fun Float.toRadians(): Float = this * PI.toFloat() / 180f
    val Float.biasedSign: Float
        get() = if (sign == 0f) 1f else sign

    fun sqrte(f: Float, e: Float = 1e-4f): Float {
        return if (f > -e) sqrt(max(0f, f))
        else Float.NaN
    }
}