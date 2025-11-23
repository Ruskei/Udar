package com.ixume.udar.physics.contact.a2s.manifold

import com.ixume.udar.Udar
import it.unimi.dsi.fastutil.ints.IntArrayList
import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap
import java.lang.Math.fma
import kotlin.math.max

class A2SPrevManifoldData(maxNumContacts: Int) {
    private val totalManifoldSize = MANIFOLD_DATA_SIZE + maxNumContacts * CONTACT_DATA_SIZE
    var arr = FloatArray(0)
    val freeList = IntArrayList()

    fun ensureCapacity(numManifolds: Int) {
        if (freeList.size < numManifolds) {
            val c = arr.size
            val toAdd = max(freeList.size / 2, numManifolds)
            val newArraySize = arr.size + toAdd * totalManifoldSize
            arr = arr.copyOf(newArraySize)
            var i = newArraySize - totalManifoldSize
            while (i >= c) {
                free(i)
                i -= totalManifoldSize
            }
        }
    }

    fun popFree(): Int {
        return freeList.popInt()
    }

    fun free(rawManifoldIdx: Int) {
        freeList.add(rawManifoldIdx)
        arr[rawManifoldIdx + NUM_CONTACTS_OFFSET] = Float.fromBits(0)
    }

    fun start(
        cursor: Int,
        numContacts: Int,
        stay: Int,
        manifoldID: Long,
    ): Int {
        arr[cursor + NUM_CONTACTS_OFFSET] = Float.fromBits(numContacts)
        arr[cursor + STAY_OFFSET] = Float.fromBits(stay)
        arr[cursor + ID_OFFSET] = Float.fromBits(manifoldID.toInt())
        arr[cursor + ID_OFFSET + 1] = Float.fromBits((manifoldID ushr 32).toInt())
        return cursor + MANIFOLD_DATA_SIZE
    }

    fun add(
        cursor: Int,

        x: Float,
        y: Float,
        z: Float,

        nl: Float,
        t1l: Float,
        t2l: Float,
    ): Int {
        arr[cursor + 0] = x
        arr[cursor + 1] = y
        arr[cursor + 2] = z

        arr[cursor + 3] = nl
        arr[cursor + 4] = t1l
        arr[cursor + 5] = t2l

        return cursor + CONTACT_DATA_SIZE
    }

    fun tick(map: Long2IntOpenHashMap) {
        check(arr.size % totalManifoldSize == 0)
        var i = 0
        while (i < arr.size) {
            val num = arr[i + NUM_CONTACTS_OFFSET].toRawBits()
            if (num > 0) {
                arr[i + STAY_OFFSET] = Float.fromBits(arr[i + STAY_OFFSET].toRawBits() - 1)
                val s = arr[i + STAY_OFFSET].toRawBits()
                if (s <= 0) {
                    val id = manifoldID(i)
                    free(i)
                    map.remove(id)
                }
            }

            i += totalManifoldSize
        }
    }

    fun numContacts(rawManifoldIdx: Int): Int {
        return arr[rawManifoldIdx + NUM_CONTACTS_OFFSET].toRawBits()
    }

    fun stay(rawManifoldIdx: Int, value: Int) {
        arr[rawManifoldIdx + STAY_OFFSET] = Float.fromBits(value)
    }

    fun manifoldID(rawManifoldIdx: Int): Long {
        val low = arr[rawManifoldIdx + ID_OFFSET].toRawBits().toLong()
        val high = arr[rawManifoldIdx + ID_OFFSET + 1].toRawBits().toLong()
        return (high shl 32) or (low and 0xFFFFFFFFL)
    }

    private fun x(rawManifoldIdx: Int, contactIdx: Int): Float {
        return arr[rawManifoldIdx + MANIFOLD_DATA_SIZE + contactIdx * CONTACT_DATA_SIZE + X_OFFSET]
    }

    private fun y(rawManifoldIdx: Int, contactIdx: Int): Float {
        return arr[rawManifoldIdx + MANIFOLD_DATA_SIZE + contactIdx * CONTACT_DATA_SIZE + Y_OFFSET]
    }

    private fun z(rawManifoldIdx: Int, contactIdx: Int): Float {
        return arr[rawManifoldIdx + MANIFOLD_DATA_SIZE + contactIdx * CONTACT_DATA_SIZE + Z_OFFSET]
    }

    fun normalLambda(rawManifoldIdx: Int, contactIdx: Int): Float {
        if (rawManifoldIdx == -1) return 0f
        val idx = rawManifoldIdx + MANIFOLD_DATA_SIZE + contactIdx * CONTACT_DATA_SIZE + NORMAL_LAMBDA_OFFSET
        val l = arr[idx]
        return l
    }

    fun t1Lambda(rawManifoldIdx: Int, contactIdx: Int): Float {
        if (rawManifoldIdx == -1) return 0f
        return arr[rawManifoldIdx + MANIFOLD_DATA_SIZE + contactIdx * CONTACT_DATA_SIZE + T1_LAMBDA_OFFSET]
    }

    fun t2Lambda(rawManifoldIdx: Int, contactIdx: Int): Float {
        if (rawManifoldIdx == -1) return 0f
        return arr[rawManifoldIdx + MANIFOLD_DATA_SIZE + contactIdx * CONTACT_DATA_SIZE + T2_LAMBDA_OFFSET]
    }

    /**
     * @return index of the contact which is closest to given (x, y, z)
     */
    fun closest(rawManifoldIdx: Int, x: Float, y: Float, z: Float): Int {
        if (rawManifoldIdx == -1) return -1

        val num = numContacts(rawManifoldIdx)
        check(num > 0)

        var minDistance = Float.MAX_VALUE
        var minIdx = -1

        var i = 0
        while (i < num) {
            val cx = x(rawManifoldIdx, i)
            val cy = y(rawManifoldIdx, i)
            val cz = z(rawManifoldIdx, i)

            val d = fma(cx - x, cx - x, fma(cy - y, cy - y, (cz - z) * (cz - z)))

            if (d < Udar.CONFIG.collision.sameContactThreshold && d < minDistance) {
                minDistance = d
                minIdx = i
            }

            i++
        }

        return minIdx
    }
}

private const val NUM_CONTACTS_OFFSET = 0
private const val STAY_OFFSET = 1
private const val ID_OFFSET = 2

private const val MANIFOLD_DATA_SIZE = 4
private const val CONTACT_DATA_SIZE = 6

private const val X_OFFSET = 0
private const val Y_OFFSET = 1
private const val Z_OFFSET = 2

private const val NORMAL_LAMBDA_OFFSET = 3
private const val T1_LAMBDA_OFFSET = 4
private const val T2_LAMBDA_OFFSET = 5
