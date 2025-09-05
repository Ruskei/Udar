package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.LOWER_MASK
import org.joml.Matrix3f
import org.joml.Vector3d
import java.util.*
import kotlin.math.max

/*
Single-threaded contact storage for Active-to-Active contacts
Only stores data about both active bodies
*/

class A2AContactArray : A2AContactCollection {
    private var arr = FloatArray(0)
    private var cursor = 0

    fun add(
        a: ActiveBody,
        b: ActiveBody,

        contactID: Long,

        pointAX: Float,
        pointAY: Float,
        pointAZ: Float,

        pointBX: Float,
        pointBY: Float,
        pointBZ: Float,

        normX: Float,
        normY: Float,
        normZ: Float,

        t1X: Float,
        t1Y: Float,
        t1Z: Float,

        t2X: Float,
        t2Y: Float,
        t2Z: Float,

        depth: Float,

        normalLambda: Float,
        t1Lambda: Float,
        t2Lambda: Float,
    ) {
        val idx = cursor * CONTACT_DATA_SIZE
        cursor++
        grow(idx + CONTACT_DATA_SIZE)

        val aID = a.uuid
        val bID = b.uuid

        arr[idx + AID_OFFSET] = Float.fromBits(aID.mostSignificantBits.toInt())
        arr[idx + AID_OFFSET + 1] = Float.fromBits((aID.mostSignificantBits ushr 32).toInt())
        arr[idx + AID_OFFSET + 2] = Float.fromBits(aID.leastSignificantBits.toInt())
        arr[idx + AID_OFFSET + 3] = Float.fromBits((aID.leastSignificantBits ushr 32).toInt())
        arr[idx + BID_OFFSET] = Float.fromBits(bID.mostSignificantBits.toInt())
        arr[idx + BID_OFFSET + 1] = Float.fromBits((bID.mostSignificantBits ushr 32).toInt())
        arr[idx + BID_OFFSET + 2] = Float.fromBits(bID.leastSignificantBits.toInt())
        arr[idx + BID_OFFSET + 3] = Float.fromBits((bID.leastSignificantBits ushr 32).toInt())
        arr[idx + CONTACT_ID_OFFSET] = Float.fromBits(contactID.toInt())
        arr[idx + CONTACT_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

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

        // Store body A data
        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(a.id)
        arr[idx + BODY_A_POS_X_OFFSET] = a.pos.x.toFloat()
        arr[idx + BODY_A_POS_Y_OFFSET] = a.pos.y.toFloat()
        arr[idx + BODY_A_POS_Z_OFFSET] = a.pos.z.toFloat()
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = a.inverseMass.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_00_OFFSET] = a.inverseInertia.m00.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_01_OFFSET] = a.inverseInertia.m01.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_02_OFFSET] = a.inverseInertia.m02.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_10_OFFSET] = a.inverseInertia.m10.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_11_OFFSET] = a.inverseInertia.m11.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_12_OFFSET] = a.inverseInertia.m12.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_20_OFFSET] = a.inverseInertia.m20.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_21_OFFSET] = a.inverseInertia.m21.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_22_OFFSET] = a.inverseInertia.m22.toFloat()

        // Store body B data
        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(b.id)
        arr[idx + BODY_B_POS_X_OFFSET] = b.pos.x.toFloat()
        arr[idx + BODY_B_POS_Y_OFFSET] = b.pos.y.toFloat()
        arr[idx + BODY_B_POS_Z_OFFSET] = b.pos.z.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = b.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_00_OFFSET] = b.inverseInertia.m00.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_01_OFFSET] = b.inverseInertia.m01.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_02_OFFSET] = b.inverseInertia.m02.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_10_OFFSET] = b.inverseInertia.m10.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_11_OFFSET] = b.inverseInertia.m11.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_12_OFFSET] = b.inverseInertia.m12.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_20_OFFSET] = b.inverseInertia.m20.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_21_OFFSET] = b.inverseInertia.m21.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_22_OFFSET] = b.inverseInertia.m22.toFloat()
    }

    override fun addCollision(
        first: ActiveBody,
        second: ActiveBody,
        pointAX: Double, pointAY: Double, pointAZ: Double,
        pointBX: Double, pointBY: Double, pointBZ: Double,
        normX: Double, normY: Double, normZ: Double,
        depth: Double,
        contactID: Long,
    ) {
        // Calculate tangent vectors following Contact class logic
        val norm = Vector3d(normX, normY, normZ)
        val t1 = Vector3d(1.0).orthogonalizeUnit(norm)
        val t2 = Vector3d(t1).cross(norm).normalize()

        // Add contact with computed tangents and default lambda values
        add(
            a = first,
            b = second,
            contactID = contactID,
            pointAX = pointAX.toFloat(),
            pointAY = pointAY.toFloat(),
            pointAZ = pointAZ.toFloat(),
            pointBX = pointBX.toFloat(),
            pointBY = pointBY.toFloat(),
            pointBZ = pointBZ.toFloat(),
            normX = normX.toFloat(),
            normY = normY.toFloat(),
            normZ = normZ.toFloat(),
            t1X = t1.x.toFloat(),
            t1Y = t1.y.toFloat(),
            t1Z = t1.z.toFloat(),
            t2X = t2.x.toFloat(),
            t2Y = t2.y.toFloat(),
            t2Z = t2.z.toFloat(),
            depth = depth.toFloat(),
            normalLambda = 0.0f,  // Default as in Contact class
            t1Lambda = 0.0f,     // Default as in Contact class
            t2Lambda = 0.0f,     // Default as in Contact class
        )
    }

    fun clear() {
        cursor = 0
    }

    fun isEmpty(): Boolean {
        return cursor == 0
    }

    fun size(): Int {
        return cursor
    }

    fun aID(idx: Int): UUID {
        val baseIdx = idx * CONTACT_DATA_SIZE
        val msb1 = arr[baseIdx + AID_OFFSET].toRawBits().toLong()
        val msb2 = arr[baseIdx + AID_OFFSET + 1].toRawBits().toLong()
        val lsb1 = arr[baseIdx + AID_OFFSET + 2].toRawBits().toLong()
        val lsb2 = arr[baseIdx + AID_OFFSET + 3].toRawBits().toLong()
        val mostSignificant = (msb2 shl 32) or (msb1 and LOWER_MASK)
        val leastSignificant = (lsb2 shl 32) or (lsb1 and LOWER_MASK)
        return UUID(mostSignificant, leastSignificant)
    }

    fun aID(idx: Int, uuid: UUID) {
        val baseIdx = idx * CONTACT_DATA_SIZE
        arr[baseIdx + AID_OFFSET] = Float.fromBits(uuid.mostSignificantBits.toInt())
        arr[baseIdx + AID_OFFSET + 1] = Float.fromBits((uuid.mostSignificantBits ushr 32).toInt())
        arr[baseIdx + AID_OFFSET + 2] = Float.fromBits(uuid.leastSignificantBits.toInt())
        arr[baseIdx + AID_OFFSET + 3] = Float.fromBits((uuid.leastSignificantBits ushr 32).toInt())
    }

    fun bID(idx: Int): UUID {
        val baseIdx = idx * CONTACT_DATA_SIZE
        val msb1 = arr[baseIdx + BID_OFFSET].toRawBits().toLong()
        val msb2 = arr[baseIdx + BID_OFFSET + 1].toRawBits().toLong()
        val lsb1 = arr[baseIdx + BID_OFFSET + 2].toRawBits().toLong()
        val lsb2 = arr[baseIdx + BID_OFFSET + 3].toRawBits().toLong()
        val mostSignificant = (msb2 shl 32) or (msb1 and LOWER_MASK)
        val leastSignificant = (lsb2 shl 32) or (lsb1 and LOWER_MASK)
        return UUID(mostSignificant, leastSignificant)
    }

    fun bID(idx: Int, uuid: UUID) {
        val baseIdx = idx * CONTACT_DATA_SIZE
        arr[baseIdx + BID_OFFSET] = Float.fromBits(uuid.mostSignificantBits.toInt())
        arr[baseIdx + BID_OFFSET + 1] = Float.fromBits((uuid.mostSignificantBits ushr 32).toInt())
        arr[baseIdx + BID_OFFSET + 2] = Float.fromBits(uuid.leastSignificantBits.toInt())
        arr[baseIdx + BID_OFFSET + 3] = Float.fromBits((uuid.leastSignificantBits ushr 32).toInt())
    }

    fun contactID(idx: Int): Long {
        val baseIdx = idx * CONTACT_DATA_SIZE
        val low = arr[baseIdx + CONTACT_ID_OFFSET].toRawBits().toLong()
        val high = arr[baseIdx + CONTACT_ID_OFFSET + 1].toRawBits().toLong()
        return (high shl 32) or (low and LOWER_MASK)
    }

    fun pointAX(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + POINT_A_X_OFFSET]
    }

    fun pointAY(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + POINT_A_Y_OFFSET]
    }

    fun pointAZ(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + POINT_A_Z_OFFSET]
    }

    fun pointBX(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + POINT_B_X_OFFSET]
    }

    fun pointBY(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + POINT_B_Y_OFFSET]
    }

    fun pointBZ(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + POINT_B_Z_OFFSET]
    }

    fun normX(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + NORM_X_OFFSET]
    }

    fun normY(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + NORM_Y_OFFSET]
    }

    fun normZ(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + NORM_Z_OFFSET]
    }

    fun t1X(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T1_X_OFFSET]
    }

    fun t1Y(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T1_Y_OFFSET]
    }

    fun t1Z(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T1_Z_OFFSET]
    }

    fun t2X(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T2_X_OFFSET]
    }

    fun t2Y(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T2_Y_OFFSET]
    }

    fun t2Z(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T2_Z_OFFSET]
    }

    fun depth(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + DEPTH_OFFSET]
    }

    fun normalLambda(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + NORMAL_LAMBDA_OFFSET]
    }

    fun t1Lambda(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T1_LAMBDA_OFFSET]
    }

    fun t2Lambda(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + T2_LAMBDA_OFFSET]
    }

    // Body A index accessor
    fun bodyAIdx(idx: Int): Int {
        return arr[idx * CONTACT_DATA_SIZE + BODY_A_IDX_OFFSET].toBits()
    }

    // Body B index setter
    fun bodyAIdx(idx: Int, value: Int) {
        arr[idx * CONTACT_DATA_SIZE + BODY_A_IDX_OFFSET] = Float.fromBits(value)
    }

    // Body A position accessors
    fun bodyAPosX(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_A_POS_X_OFFSET]
    }

    fun bodyAPosY(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_A_POS_Y_OFFSET]
    }

    fun bodyAPosZ(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_A_POS_Z_OFFSET]
    }

    // Body A inverse mass accessor
    fun bodyAInverseMass(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_A_INVERSE_MASS_OFFSET]
    }

    // Body A inverse inertia accessor
    fun bodyAInverseInertia(idx: Int, out: Matrix3f): Matrix3f {
        val baseIdx = idx * CONTACT_DATA_SIZE
        return out.set(
            arr[baseIdx + BODY_A_INVERSE_INERTIA_00_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_01_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_02_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_10_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_11_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_12_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_20_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_21_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_22_OFFSET]
        )
    }

    // Body B index accessor
    fun bodyBIdx(idx: Int): Int {
        return arr[idx * CONTACT_DATA_SIZE + BODY_B_IDX_OFFSET].toBits()
    }

    // Body B index setter
    fun bodyBIdx(idx: Int, value: Int) {
        arr[idx * CONTACT_DATA_SIZE + BODY_B_IDX_OFFSET] = Float.fromBits(value)
    }

    // Body B position accessors
    fun bodyBPosX(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_B_POS_X_OFFSET]
    }

    fun bodyBPosY(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_B_POS_Y_OFFSET]
    }

    fun bodyBPosZ(idx: Int): Float {
        return arr[idx + BODY_B_POS_Z_OFFSET]
    }

    // Body B inverse mass accessor
    fun bodyBInverseMass(idx: Int): Float {
        return arr[idx * CONTACT_DATA_SIZE + BODY_B_INVERSE_MASS_OFFSET]
    }

    // Body B inverse inertia accessor
    fun bodyBInverseInertia(idx: Int, out: Matrix3f): Matrix3f {
        val baseIdx = idx * CONTACT_DATA_SIZE
        return out.set(
            arr[baseIdx + BODY_B_INVERSE_INERTIA_00_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_01_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_02_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_10_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_11_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_12_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_20_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_21_OFFSET],
            arr[baseIdx + BODY_B_INVERSE_INERTIA_22_OFFSET]
        )
    }

    // Lambda setters
    fun setNormalLambda(idx: Int, value: Float) {
        arr[idx * CONTACT_DATA_SIZE + NORMAL_LAMBDA_OFFSET] = value
    }

    fun setT1Lambda(idx: Int, value: Float) {
        arr[idx * CONTACT_DATA_SIZE + T1_LAMBDA_OFFSET] = value
    }

    fun setT2Lambda(idx: Int, value: Float) {
        arr[idx * CONTACT_DATA_SIZE + T2_LAMBDA_OFFSET] = value
    }

    fun transferToBuffer(contactIdx: Int, buffer: A2AContactBuffer, activeBodyA: ActiveBody, activeBodyB: ActiveBody) {
        val baseIdx = contactIdx * CONTACT_DATA_SIZE

        buffer.addCollision(
            first = activeBodyA,
            second = activeBodyB,
            pointAX = arr[baseIdx + POINT_A_X_OFFSET].toDouble(),
            pointAY = arr[baseIdx + POINT_A_Y_OFFSET].toDouble(),
            pointAZ = arr[baseIdx + POINT_A_Z_OFFSET].toDouble(),
            pointBX = arr[baseIdx + POINT_B_X_OFFSET].toDouble(),
            pointBY = arr[baseIdx + POINT_B_Y_OFFSET].toDouble(),
            pointBZ = arr[baseIdx + POINT_B_Z_OFFSET].toDouble(),
            normX = arr[baseIdx + NORM_X_OFFSET].toDouble(),
            normY = arr[baseIdx + NORM_Y_OFFSET].toDouble(),
            normZ = arr[baseIdx + NORM_Z_OFFSET].toDouble(),
            depth = arr[baseIdx + DEPTH_OFFSET].toDouble(),
            contactID = contactID(contactIdx)
        )
    }

    fun nextIdx(i: Int): Int {
        return if (i + CONTACT_DATA_SIZE >= size()) -1 else i + CONTACT_DATA_SIZE
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

private const val CONTACT_DATA_SIZE = 57

private const val AID_OFFSET = 0 // 4 floats (UUID as 4 ints)
private const val BID_OFFSET = 4 // 4 floats (UUID as 4 ints)
private const val CONTACT_ID_OFFSET = 8 // 2 floats (long as 2 ints)

private const val POINT_A_X_OFFSET = 10
private const val POINT_A_Y_OFFSET = 11
private const val POINT_A_Z_OFFSET = 12

private const val POINT_B_X_OFFSET = 13
private const val POINT_B_Y_OFFSET = 14
private const val POINT_B_Z_OFFSET = 15

private const val NORM_X_OFFSET = 16
private const val NORM_Y_OFFSET = 17
private const val NORM_Z_OFFSET = 18

private const val T1_X_OFFSET = 19
private const val T1_Y_OFFSET = 20
private const val T1_Z_OFFSET = 21

private const val T2_X_OFFSET = 22
private const val T2_Y_OFFSET = 23
private const val T2_Z_OFFSET = 24

private const val DEPTH_OFFSET = 25

private const val NORMAL_LAMBDA_OFFSET = 26
private const val T1_LAMBDA_OFFSET = 27
private const val T2_LAMBDA_OFFSET = 28

// Body A data
private const val BODY_A_IDX_OFFSET = 29
private const val BODY_A_POS_X_OFFSET = 30
private const val BODY_A_POS_Y_OFFSET = 31
private const val BODY_A_POS_Z_OFFSET = 32
private const val BODY_A_INVERSE_MASS_OFFSET = 33
private const val BODY_A_INVERSE_INERTIA_00_OFFSET = 34
private const val BODY_A_INVERSE_INERTIA_01_OFFSET = 35
private const val BODY_A_INVERSE_INERTIA_02_OFFSET = 36
private const val BODY_A_INVERSE_INERTIA_10_OFFSET = 37
private const val BODY_A_INVERSE_INERTIA_11_OFFSET = 38
private const val BODY_A_INVERSE_INERTIA_12_OFFSET = 39
private const val BODY_A_INVERSE_INERTIA_20_OFFSET = 40
private const val BODY_A_INVERSE_INERTIA_21_OFFSET = 41
private const val BODY_A_INVERSE_INERTIA_22_OFFSET = 42

// Body B data
private const val BODY_B_IDX_OFFSET = 43
private const val BODY_B_POS_X_OFFSET = 44
private const val BODY_B_POS_Y_OFFSET = 45
private const val BODY_B_POS_Z_OFFSET = 46
private const val BODY_B_INVERSE_MASS_OFFSET = 47
private const val BODY_B_INVERSE_INERTIA_00_OFFSET = 48
private const val BODY_B_INVERSE_INERTIA_01_OFFSET = 49
private const val BODY_B_INVERSE_INERTIA_02_OFFSET = 50
private const val BODY_B_INVERSE_INERTIA_10_OFFSET = 51
private const val BODY_B_INVERSE_INERTIA_11_OFFSET = 52
private const val BODY_B_INVERSE_INERTIA_12_OFFSET = 53
private const val BODY_B_INVERSE_INERTIA_20_OFFSET = 54
private const val BODY_B_INVERSE_INERTIA_21_OFFSET = 55
private const val BODY_B_INVERSE_INERTIA_22_OFFSET = 56