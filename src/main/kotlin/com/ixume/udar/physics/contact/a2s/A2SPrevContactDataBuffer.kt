package com.ixume.udar.physics.contact.a2s

import it.unimi.dsi.fastutil.floats.FloatArrayList

class A2SPrevContactDataBuffer {
    private val ls = FloatArrayList()

    fun clear() {
        ls.clear()
    }

    fun add(
        x: Float, y: Float, z: Float,
        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    ) {
        ls.add(x)
        ls.add(y)
        ls.add(z)

        ls.add(normalLambda)
        ls.add(t1Lambda)
        ls.add(t2Lambda)
    }

    fun x(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + X_OFFSET)
    }

    fun y(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + Y_OFFSET)
    }

    fun z(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + Z_OFFSET)
    }

    fun normalLambda(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + NORMAL_LAMBDA_OFFSET)
    }

    fun t1Lambda(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + T1_LAMBDA_OFFSET)
    }

    fun t2Lambda(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + T2_LAMBDA_OFFSET)
    }
}

private const val DATA_SIZE = 6

private const val X_OFFSET = 0
private const val Y_OFFSET = 1
private const val Z_OFFSET = 2

private const val NORMAL_LAMBDA_OFFSET = 3
private const val T1_LAMBDA_OFFSET = 4
private const val T2_LAMBDA_OFFSET = 5
