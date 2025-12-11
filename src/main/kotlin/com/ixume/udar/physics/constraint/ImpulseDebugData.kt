package com.ixume.udar.physics.constraint

import com.ixume.udar.Udar
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.roundToInt

class ImpulseDebugData {
    var arr = FloatArray(Udar.CONFIG.collision.normalIterations)
    var iteration = 0
    fun setup() {
        iteration = 0
        if (arr.size != Udar.CONFIG.collision.normalIterations) {
            arr = FloatArray(Udar.CONFIG.collision.normalIterations)
        } else {
            arr.fill(0f)
        }
    }

    fun reportLambdas() {
        val iterations = Udar.CONFIG.collision.normalIterations
        if (iterations <= 0) return
        for (i in 0..<arr.size) {
            arr[i]
        }

        val first = arr[0]
        println("∑(Δλ):")
        println("  %4d: %8.2e ".format(1, first) + "*".repeat(10))
        for (i in 2..arr.size) {
            val v = arr[i - 1]
            if (first == 0f) {
                println("  %4d: %8.2e ".format(i, v))
            } else {
                val num = abs(v / first * 10.0).roundToInt()
                val clamped = min(200, num)
                print("  %4d: %8.2e ".format(i, v) + "*".repeat(clamped))
                if (clamped < num) {
                    print("($num)")
                }
                println()
            }
        }
    }

    operator fun plusAssign(lambda: Float) {
        arr[iteration - 1] += abs(lambda)
    }
}