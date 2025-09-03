package com.ixume.udar

import java.lang.String
import java.util.*
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.max

/*
technically we don't need to store contact points for constraint solving but it's REALLY helpful for debugging
 */

class ContactBuffer {
    @Volatile
    private var arr = DoubleArray(0)

    private val cursor = AtomicInteger(0)

    private var status = ContactBufferStatus.READING

    fun reset() {
        check(status == ContactBufferStatus.READING)
        cursor.set(0)
    }

    fun setStatus(status: ContactBufferStatus) {
        this.status = status
    }

    fun add(
        aID: UUID,
        bID: UUID,

        contactID: Long,

        pointAX: Double,
        pointAY: Double,
        pointAZ: Double,

        pointBX: Double,
        pointBY: Double,
        pointBZ: Double,

        normX: Double,
        normY: Double,
        normZ: Double,

        t1X: Double,
        t1Y: Double,
        t1Z: Double,

        t2X: Double,
        t2Y: Double,
        t2Z: Double,

        depth: Double,

        normalLambda: Double,
        t1Lambda: Double,
        t2Lambda: Double,
    ) {
        check(status == ContactBufferStatus.ADDING)
        val idx = cursor.getAndIncrement() * CONTACT_DATA_SIZE
        grow(idx + CONTACT_DATA_SIZE)

        arr[idx + AID_OFFSET] = aID.mostSignificantBits.toDouble()
        arr[idx + AID_OFFSET + 1] = aID.leastSignificantBits.toDouble()
        arr[idx + BID_OFFSET] = bID.mostSignificantBits.toDouble()
        arr[idx + BID_OFFSET + 1] = bID.leastSignificantBits.toDouble()
        arr[idx + CONTACT_ID_OFFSET] = contactID.toDouble()

        arr[idx + POINT_A_X_OFFSET] = pointAX
        arr[idx + POINT_A_Y_OFFSET] = pointAY
        arr[idx + POINT_A_Z_OFFSET] = pointAZ

        arr[idx + POINT_B_X_OFFSET] = pointBX
        arr[idx + POINT_B_Y_OFFSET] = pointBY
        arr[idx + POINT_B_Z_OFFSET] = pointBZ

        arr[idx + NORM_X_OFFSET] = normX
        arr[idx + NORM_Y_OFFSET] = normY
        arr[idx + NORM_Z_OFFSET] = normZ

        arr[idx + T1_X_OFFSET] = t1X
        arr[idx + T1_Y_OFFSET] = t1Y
        arr[idx + T1_Z_OFFSET] = t1Z

        arr[idx + T2_X_OFFSET] = t2X
        arr[idx + T2_Y_OFFSET] = t2Y
        arr[idx + T2_Z_OFFSET] = t2Z

        arr[idx + DEPTH_OFFSET] = depth

        arr[idx + NORMAL_LAMBDA_OFFSET] = normalLambda
        arr[idx + T1_LAMBDA_OFFSET] = t1Lambda
        arr[idx + T2_LAMBDA_OFFSET] = t2Lambda
    }

    fun clear() {
        check(status == ContactBufferStatus.READING)

        cursor.set(0)
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)

        arr = arr.copyOf(newSize)
    }
}

enum class ContactBufferStatus {
    ADDING, READING
}

private const val CONTACT_DATA_SIZE = 24

private const val AID_OFFSET = 0 // 2 doubles
private const val BID_OFFSET = 2 // 2 doubles
private const val CONTACT_ID_OFFSET = 4 // long

private const val POINT_A_X_OFFSET = 5
private const val POINT_A_Y_OFFSET = 6
private const val POINT_A_Z_OFFSET = 7

private const val POINT_B_X_OFFSET = 8
private const val POINT_B_Y_OFFSET = 9
private const val POINT_B_Z_OFFSET = 10

private const val NORM_X_OFFSET = 11
private const val NORM_Y_OFFSET = 12
private const val NORM_Z_OFFSET = 13

private const val T1_X_OFFSET = 14
private const val T1_Y_OFFSET = 15
private const val T1_Z_OFFSET = 16

private const val T2_X_OFFSET = 17
private const val T2_Y_OFFSET = 18
private const val T2_Z_OFFSET = 19

private const val DEPTH_OFFSET = 20

private const val NORMAL_LAMBDA_OFFSET = 21
private const val T1_LAMBDA_OFFSET = 22
private const val T2_LAMBDA_OFFSET = 23
