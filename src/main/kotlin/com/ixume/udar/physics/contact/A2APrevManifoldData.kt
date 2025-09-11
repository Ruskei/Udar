package com.ixume.udar.physics.contact

import it.unimi.dsi.fastutil.floats.FloatArrayList
import java.lang.Math.fma

class A2APrevManifoldData {
    private val ls = FloatArrayList()

    fun clear() {
        ls.clear()
    }

    fun add(
        numContacts: Int,
        buf: A2APrevContactDataBuffer
    ): Int {
        val idx = ls.size

        ls.add(Float.fromBits(numContacts))

        var i = 0
        while (i < numContacts) {
            ls.add(buf.ax(i))
            ls.add(buf.ay(i))
            ls.add(buf.az(i))

            ls.add(buf.bx(i))
            ls.add(buf.by(i))
            ls.add(buf.bz(i))

            ls.add(buf.normalLambda(i))
            ls.add(buf.t1Lambda(i))
            ls.add(buf.t2Lambda(i))

            i++
        }

        return idx
    }

    fun numContacts(rawManifoldIdx: Int, default: Int): Int {
        if (rawManifoldIdx == -1) return default
        return ls.getFloat(rawManifoldIdx + NUM_CONTACTS_OFFSET).toRawBits()
    }

    private fun ax(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + POINT_A_X_OFFSET)
    }

    private fun ay(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + POINT_A_Y_OFFSET)
    }

    private fun az(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + POINT_A_Z_OFFSET)
    }

    private fun bx(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + POINT_B_X_OFFSET)
    }

    private fun by(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + POINT_B_Y_OFFSET)
    }

    private fun bz(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + POINT_B_Z_OFFSET)
    }

    fun normalLambda(rawManifoldIdx: Int, contactIdx: Int): Float {
        if (rawManifoldIdx == -1) return 0f
        val idx = rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + NORMAL_LAMBDA_OFFSET
        val l = ls.getFloat(idx)
        return l
    }

    fun t1Lambda(rawManifoldIdx: Int, contactIdx: Int): Float {
        if (rawManifoldIdx == -1) return 0f
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + T1_LAMBDA_OFFSET)
    }

    fun t2Lambda(rawManifoldIdx: Int, contactIdx: Int): Float {
        if (rawManifoldIdx == -1) return 0f
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + T2_LAMBDA_OFFSET)
    }

    /**
     * @return index of the contact which is closest to given (x, y, z)
     */
    fun closestA(rawManifoldIdx: Int, x: Float, y: Float, z: Float): Int {
        if (rawManifoldIdx == -1) return -1

        val num = numContacts(rawManifoldIdx, -1)
        check(num > 0)

        var minDistance = Float.MAX_VALUE
        var minIdx = -1

        var i = 0
        while (i < num) {
            val cx = ax(rawManifoldIdx, i)
            val cy = ay(rawManifoldIdx, i)
            val cz = az(rawManifoldIdx, i)

            val d = fma(cx - x, cx - x, fma(cy - y, cy - y, (cz - z) * (cz - z)))

            if (d < minDistance) {
                minDistance = d
                minIdx = i
            }

            i++
        }

        check(minIdx != -1)

        return minIdx
    }

    /**
     * @return index of the contact which is closest to given (x, y, z)
     */
    fun closestB(rawManifoldIdx: Int, x: Float, y: Float, z: Float): Int {
        if (rawManifoldIdx == -1) return -1

        val num = numContacts(rawManifoldIdx, -1)
        check(num > 0)

        var minDistance = Float.MAX_VALUE
        var minIdx = -1

        var i = 0
        while (i < num) {
            val cx = bx(rawManifoldIdx, i)
            val cy = by(rawManifoldIdx, i)
            val cz = bz(rawManifoldIdx, i)

            val d = fma(cx - x, cx - x, fma(cy - y, cy - y, (cz - z) * (cz - z)))

            if (d < minDistance) {
                minDistance = d
                minIdx = i
            }

            i++
        }

        check(minIdx != -1)

        return minIdx
    }
}

private const val NUM_CONTACTS_OFFSET = 0
private const val CONTACT_DATA_OFFSET = 1

private const val CONTACT_DATA_SIZE = 9

private const val POINT_A_X_OFFSET = 0
private const val POINT_A_Y_OFFSET = 1
private const val POINT_A_Z_OFFSET = 2

private const val POINT_B_X_OFFSET = 3
private const val POINT_B_Y_OFFSET = 4
private const val POINT_B_Z_OFFSET = 5

private const val NORMAL_LAMBDA_OFFSET = 6
private const val T1_LAMBDA_OFFSET = 7
private const val T2_LAMBDA_OFFSET = 8
