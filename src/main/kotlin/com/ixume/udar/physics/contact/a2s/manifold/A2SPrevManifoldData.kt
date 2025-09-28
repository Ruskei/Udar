package com.ixume.udar.physics.contact.a2s.manifold

import com.ixume.udar.physics.contact.a2s.A2SPrevContactDataBuffer
import it.unimi.dsi.fastutil.floats.FloatArrayList
import java.lang.Math.fma

class A2SPrevManifoldData {
    private val ls = FloatArrayList()

    fun clear() {
        ls.clear()
    }

    fun add(
        numContacts: Int,
        buf: A2SPrevContactDataBuffer,
    ): Int {
        val idx = ls.size

        ls.add(Float.fromBits(numContacts))

        var i = 0
        while (i < numContacts) {
            ls.add(buf.x(i))
            ls.add(buf.y(i))
            ls.add(buf.z(i))

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

    private fun x(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + X_OFFSET)
    }

    private fun y(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + Y_OFFSET)
    }

    private fun z(rawManifoldIdx: Int, contactIdx: Int): Float {
        return ls.getFloat(rawManifoldIdx + CONTACT_DATA_OFFSET + contactIdx * CONTACT_DATA_SIZE + Z_OFFSET)
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
    fun closest(rawManifoldIdx: Int, x: Float, y: Float, z: Float): Int {
        if (rawManifoldIdx == -1) return -1

        val num = numContacts(rawManifoldIdx, -1)
        check(num > 0)

        var minDistance = Float.MAX_VALUE
        var minIdx = -1

        var i = 0
        while (i < num) {
            val cx = x(rawManifoldIdx, i)
            val cy = y(rawManifoldIdx, i)
            val cz = z(rawManifoldIdx, i)

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

private const val CONTACT_DATA_SIZE = 6

private const val X_OFFSET = 0
private const val Y_OFFSET = 1
private const val Z_OFFSET = 2

private const val NORMAL_LAMBDA_OFFSET = 3
private const val T1_LAMBDA_OFFSET = 4
private const val T2_LAMBDA_OFFSET = 5
