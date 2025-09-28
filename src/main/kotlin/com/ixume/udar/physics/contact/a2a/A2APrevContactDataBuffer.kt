package com.ixume.udar.physics.contact.a2a

import it.unimi.dsi.fastutil.floats.FloatArrayList

class A2APrevContactDataBuffer {
    private val ls = FloatArrayList()

    fun clear() {
        ls.clear()
    }

    fun add(
        ax: Float, ay: Float, az: Float,
        bx: Float, by: Float, bz: Float,
        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    ) {
        ls.add(ax)
        ls.add(ay)
        ls.add(az)

        ls.add(bx)
        ls.add(by)
        ls.add(bz)

        ls.add(normalLambda)
        ls.add(t1Lambda)
        ls.add(t2Lambda)
    }

    fun ax(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + POINT_A_X_OFFSET)
    }

    fun ay(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + POINT_A_Y_OFFSET)
    }

    fun az(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + POINT_A_Z_OFFSET)
    }

    fun bx(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + POINT_B_X_OFFSET)
    }

    fun by(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + POINT_B_Y_OFFSET)
    }

    fun bz(idx: Int): Float {
        return ls.getFloat(idx * DATA_SIZE + POINT_B_Z_OFFSET)
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

private const val DATA_SIZE = 9

private const val POINT_A_X_OFFSET = 0
private const val POINT_A_Y_OFFSET = 1
private const val POINT_A_Z_OFFSET = 2

private const val POINT_B_X_OFFSET = 3
private const val POINT_B_Y_OFFSET = 4
private const val POINT_B_Z_OFFSET = 5

private const val NORMAL_LAMBDA_OFFSET = 6
private const val T1_LAMBDA_OFFSET = 7
private const val T2_LAMBDA_OFFSET = 8