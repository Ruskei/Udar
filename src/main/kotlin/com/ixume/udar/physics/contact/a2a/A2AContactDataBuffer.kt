package com.ixume.udar.physics.contact.a2a

import com.ixume.udar.collisiondetection.local.LocalMathUtil
import org.joml.Vector3f
import java.lang.Math.fma
import kotlin.math.abs

class A2AContactDataBuffer(private val numContacts: Int) {
    val arr = FloatArray(CONTACT_DATA_SIZE * numContacts)
    var cursor = 0

    fun clear() {
        cursor = 0
    }

    fun size(): Int {
        return cursor
    }

    fun dataSize(): Int {
        return cursor * CONTACT_DATA_SIZE
    }

    fun loadInto(
        pointAX: Float, pointAY: Float, pointAZ: Float,
        pointBX: Float, pointBY: Float, pointBZ: Float,

        normX: Float, normY: Float, normZ: Float,

        depth: Float,
        math: LocalMathUtil,

        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    ) {
        val norm = math._n.set(normX, normY, normZ)
        val t1 = math._t1v.set(1f).orthogonalizeUnit(norm)
        val t2 = math._t2v.set(t1).cross(norm).normalize()

        _loadInto(
            pointAX = pointAX,
            pointAY = pointAY,
            pointAZ = pointAZ,

            pointBX = pointBX,
            pointBY = pointBY,
            pointBZ = pointBZ,

            normX = normX,
            normY = normY,
            normZ = normZ,

            t1X = t1.x,
            t1Y = t1.y,
            t1Z = t1.z,

            t2X = t2.x,
            t2Y = t2.y,
            t2Z = t2.z,

            depth = depth,

            normalLambda = normalLambda,
            t1Lambda = t1Lambda,
            t2Lambda = t2Lambda,
        )
    }

    fun _loadInto(
        pointAX: Float, pointAY: Float, pointAZ: Float,
        pointBX: Float, pointBY: Float, pointBZ: Float,

        normX: Float, normY: Float, normZ: Float,
        t1X: Float, t1Y: Float, t1Z: Float,
        t2X: Float, t2Y: Float, t2Z: Float,

        depth: Float,

        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    ) {
        require(cursor < numContacts)

        val contactArrIdx = cursor * CONTACT_DATA_SIZE

        arr[contactArrIdx + POINT_A_X_OFFSET] = pointAX
        arr[contactArrIdx + POINT_A_Y_OFFSET] = pointAY
        arr[contactArrIdx + POINT_A_Z_OFFSET] = pointAZ

        arr[contactArrIdx + POINT_B_X_OFFSET] = pointBX
        arr[contactArrIdx + POINT_B_Y_OFFSET] = pointBY
        arr[contactArrIdx + POINT_B_Z_OFFSET] = pointBZ

        arr[contactArrIdx + NORM_X_OFFSET] = normX
        arr[contactArrIdx + NORM_Y_OFFSET] = normY
        arr[contactArrIdx + NORM_Z_OFFSET] = normZ

        arr[contactArrIdx + T1_X_OFFSET] = t1X
        arr[contactArrIdx + T1_Y_OFFSET] = t1Y
        arr[contactArrIdx + T1_Z_OFFSET] = t1Z

        arr[contactArrIdx + T2_X_OFFSET] = t2X
        arr[contactArrIdx + T2_Y_OFFSET] = t2Y
        arr[contactArrIdx + T2_Z_OFFSET] = t2Z

        arr[contactArrIdx + DEPTH_OFFSET] = depth

//        if (normalLambda == 0f) println("LOADED COLD LAMBDA")

        arr[contactArrIdx + NORMAL_LAMBDA_OFFSET] = normalLambda
        arr[contactArrIdx + T1_LAMBDA_OFFSET] = t1Lambda
        arr[contactArrIdx + T2_LAMBDA_OFFSET] = t2Lambda

        cursor++
    }

    private val _vec = Vector3f()
    private val _temp = FloatArray(4 * CONTACT_DATA_SIZE)

    fun reduceTo4() {
        if (cursor <= 4) return

        var aIdx = -1
        var maxDepth = -Float.MAX_VALUE
        for (i in 0..<cursor) {
            val d = depth(i)
            if (d > maxDepth) {
                maxDepth = d
                aIdx = i
            }
        }

        check(aIdx != -1)

        val aNx = nx(aIdx)
        val aNy = ny(aIdx)
        val aNz = nz(aIdx)

        val aDot = abs(fma(aNx, midx(aIdx), fma(aNy, midy(aIdx), aNz * midz(aIdx))))

        var bIdx = -1
        var maxDot = -Float.MAX_VALUE
        for (i in 0..<cursor) {
            val dot = abs(fma(aNx, midx(i), fma(aNy, midy(i), aNz * midz(i))) - aDot)
            if (dot > maxDot) {
                bIdx = i
                maxDot = dot
            }
        }

        check(bIdx != -1)

        var cIdx = -1
        var maxArea = -Float.MAX_VALUE
        for (i in 0..<cursor) {
            val area = _vec.set(
                midx(aIdx) - midx(i),
                midy(aIdx) - midy(i),
                midz(aIdx) - midz(i),
            ).cross(
                midx(bIdx) - midx(i),
                midy(bIdx) - midy(i),
                midz(bIdx) - midz(i),
            ).length() * 0.5f

            if (area > maxArea) {
                cIdx = i
                maxArea = area
            }
        }

        check(cIdx != -1)

        var dIdx = -1
        var maxVolume = -Float.MAX_VALUE
        for (i in 0..<cursor) {
            val volume = abs(
                midx(aIdx) * (midy(bIdx) * midz(cIdx) - midz(bIdx) * midy(cIdx)) -
                midy(aIdx) * (midx(bIdx) * midz(cIdx) - midz(bIdx) * midx(cIdx)) +
                midz(aIdx) * (midx(bIdx) * midy(cIdx) - midy(bIdx) * midx(cIdx))
            ) / 6f

            if (volume > maxVolume) {
                maxVolume = volume
                dIdx = i
            }
        }

        check(dIdx != -1)

        _temp

        System.arraycopy(arr, aIdx * CONTACT_DATA_SIZE, _temp, 0, CONTACT_DATA_SIZE)
        System.arraycopy(arr, bIdx * CONTACT_DATA_SIZE, _temp, CONTACT_DATA_SIZE, CONTACT_DATA_SIZE)
        System.arraycopy(arr, cIdx * CONTACT_DATA_SIZE, _temp, CONTACT_DATA_SIZE * 2, CONTACT_DATA_SIZE)
        System.arraycopy(arr, dIdx * CONTACT_DATA_SIZE, _temp, CONTACT_DATA_SIZE * 3, CONTACT_DATA_SIZE)

        clear()

        System.arraycopy(_temp, 0, arr, 0, CONTACT_DATA_SIZE * 4)
        cursor = 4
    }

    private fun depth(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + DEPTH_OFFSET]
    }

    private fun nx(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + NORM_X_OFFSET]
    }

    private fun ny(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + NORM_Y_OFFSET]
    }

    private fun nz(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + NORM_Z_OFFSET]
    }

    private fun midx(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + POINT_A_X_OFFSET] * 0.5f + arr[contactIdx * CONTACT_DATA_SIZE + POINT_B_X_OFFSET] * 0.5f
    }

    private fun midy(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + POINT_A_Y_OFFSET] * 0.5f + arr[contactIdx * CONTACT_DATA_SIZE + POINT_B_Y_OFFSET] * 0.5f
    }

    private fun midz(contactIdx: Int): Float {
        return arr[contactIdx * CONTACT_DATA_SIZE + POINT_A_Z_OFFSET] * 0.5f + arr[contactIdx * CONTACT_DATA_SIZE + POINT_B_Z_OFFSET] * 0.5f
    }
}

private const val CONTACT_DATA_SIZE = 19

// Contact data layout within each contact (16 floats per contact)
private const val POINT_A_X_OFFSET = 0 // 3 floats
private const val POINT_A_Y_OFFSET = 1
private const val POINT_A_Z_OFFSET = 2

private const val POINT_B_X_OFFSET = 3 // 3 floats
private const val POINT_B_Y_OFFSET = 4
private const val POINT_B_Z_OFFSET = 5

private const val NORM_X_OFFSET = 6 // 3 floats
private const val NORM_Y_OFFSET = 7
private const val NORM_Z_OFFSET = 8

private const val T1_X_OFFSET = 9 // 3 floats
private const val T1_Y_OFFSET = 10
private const val T1_Z_OFFSET = 11

private const val T2_X_OFFSET = 12 // 3 floats
private const val T2_Y_OFFSET = 13
private const val T2_Z_OFFSET = 14

private const val DEPTH_OFFSET = 15 // 1 float

private const val NORMAL_LAMBDA_OFFSET = 16 // 3 floats
private const val T1_LAMBDA_OFFSET = 17
private const val T2_LAMBDA_OFFSET = 18
