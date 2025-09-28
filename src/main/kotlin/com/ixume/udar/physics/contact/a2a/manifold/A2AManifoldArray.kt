package com.ixume.udar.physics.contact.a2a.manifold

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.a2a.A2AContactDataBuffer
import kotlin.math.max

class A2AManifoldArray(val maxContactNum: Int) : A2AManifoldCollection {
    override var arr = FloatArray(0)
    private var cursor = 0

    private val manifoldDataSize = MANIFOLD_PREFIX_SIZE + maxContactNum * CONTACT_DATA_SIZE

    val maxContactArrSize = maxContactNum * CONTACT_DATA_SIZE

    fun size(): Int {
        return cursor
    }

    override fun addManifold(
        first: ActiveBody,
        second: ActiveBody,
        contactID: Long,
        buf: A2AContactDataBuffer,
    ) {
        if (buf.size() == 0) return
        val idx = cursor * manifoldDataSize
        cursor++

        grow(idx + manifoldDataSize)

        // Store contact ID (2 floats)
        arr[idx + MANIFOLD_ID_OFFSET] = Float.fromBits(contactID.toInt())
        arr[idx + MANIFOLD_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

        arr[idx + CONTACT_NUM_OFFSET] = Float.fromBits(buf.cursor)

        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(first.idx)
        arr[idx + BODY_A_POS_X_OFFSET] = first.pos.x.toFloat()
        arr[idx + BODY_A_POS_Y_OFFSET] = first.pos.y.toFloat()
        arr[idx + BODY_A_POS_Z_OFFSET] = first.pos.z.toFloat()
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = first.inverseMass.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = first.inverseInertia.m00.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = first.inverseInertia.m01.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = first.inverseInertia.m02.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = first.inverseInertia.m10.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = first.inverseInertia.m11.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = first.inverseInertia.m12.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = first.inverseInertia.m20.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = first.inverseInertia.m21.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = first.inverseInertia.m22.toFloat()

        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(second.idx)
        arr[idx + BODY_B_POS_X_OFFSET] = second.pos.x.toFloat()
        arr[idx + BODY_B_POS_Y_OFFSET] = second.pos.y.toFloat()
        arr[idx + BODY_B_POS_Z_OFFSET] = second.pos.z.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = second.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XX_OFFSET] = second.inverseInertia.m00.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XY_OFFSET] = second.inverseInertia.m01.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XZ_OFFSET] = second.inverseInertia.m02.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YX_OFFSET] = second.inverseInertia.m10.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YY_OFFSET] = second.inverseInertia.m11.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YZ_OFFSET] = second.inverseInertia.m12.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZX_OFFSET] = second.inverseInertia.m20.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZY_OFFSET] = second.inverseInertia.m21.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZZ_OFFSET] = second.inverseInertia.m22.toFloat()

        check(buf.dataSize() <= maxContactArrSize)
        System.arraycopy(buf.arr, 0, arr, idx + CONTACTS_OFFSET, buf.dataSize())
    }

    override fun addSingleManifold(
        first: ActiveBody,
        second: ActiveBody,
        contactID: Long,

        pointAX: Float, pointAY: Float, pointAZ: Float,
        pointBX: Float, pointBY: Float, pointBZ: Float,

        normX: Float, normY: Float, normZ: Float,

        depth: Float,
        math: LocalMathUtil,

        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    ) {
        val idx = cursor * manifoldDataSize
        cursor++
        grow(idx + manifoldDataSize)

        val norm = math._n.set(normX, normY, normZ)
        val t1 = math._t1v.set(1f).orthogonalizeUnit(norm)
        val t2 = math._t2v.set(t1).cross(norm).normalize()

        arr[idx + MANIFOLD_ID_OFFSET] = Float.fromBits(contactID.toInt())
        arr[idx + MANIFOLD_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

        arr[idx + CONTACT_NUM_OFFSET] = Float.fromBits(1)

        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(first.idx)
        arr[idx + BODY_A_POS_X_OFFSET] = first.pos.x.toFloat()
        arr[idx + BODY_A_POS_Y_OFFSET] = first.pos.y.toFloat()
        arr[idx + BODY_A_POS_Z_OFFSET] = first.pos.z.toFloat()
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = first.inverseMass.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = first.inverseInertia.m00.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = first.inverseInertia.m01.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = first.inverseInertia.m02.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = first.inverseInertia.m10.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = first.inverseInertia.m11.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = first.inverseInertia.m12.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = first.inverseInertia.m20.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = first.inverseInertia.m21.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = first.inverseInertia.m22.toFloat()

        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(second.idx)
        arr[idx + BODY_B_POS_X_OFFSET] = second.pos.x.toFloat()
        arr[idx + BODY_B_POS_Y_OFFSET] = second.pos.y.toFloat()
        arr[idx + BODY_B_POS_Z_OFFSET] = second.pos.z.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = second.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XX_OFFSET] = second.inverseInertia.m00.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XY_OFFSET] = second.inverseInertia.m01.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XZ_OFFSET] = second.inverseInertia.m02.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YX_OFFSET] = second.inverseInertia.m10.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YY_OFFSET] = second.inverseInertia.m11.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YZ_OFFSET] = second.inverseInertia.m12.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZX_OFFSET] = second.inverseInertia.m20.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZY_OFFSET] = second.inverseInertia.m21.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZZ_OFFSET] = second.inverseInertia.m22.toFloat()

        val contactArrIdx = idx + CONTACTS_OFFSET

        arr[contactArrIdx + POINT_A_X_OFFSET] = pointAX
        arr[contactArrIdx + POINT_A_Y_OFFSET] = pointAY
        arr[contactArrIdx + POINT_A_Z_OFFSET] = pointAZ

        arr[contactArrIdx + POINT_B_X_OFFSET] = pointBX
        arr[contactArrIdx + POINT_B_Y_OFFSET] = pointBY
        arr[contactArrIdx + POINT_B_Z_OFFSET] = pointBZ

        arr[contactArrIdx + NORM_X_OFFSET] = normX
        arr[contactArrIdx + NORM_Y_OFFSET] = normY
        arr[contactArrIdx + NORM_Z_OFFSET] = normZ

        arr[contactArrIdx + T1_X_OFFSET] = t1.x
        arr[contactArrIdx + T1_Y_OFFSET] = t1.y
        arr[contactArrIdx + T1_Z_OFFSET] = t1.z

        arr[contactArrIdx + T2_X_OFFSET] = t2.x
        arr[contactArrIdx + T2_Y_OFFSET] = t2.y
        arr[contactArrIdx + T2_Z_OFFSET] = t2.z

        arr[contactArrIdx + DEPTH_OFFSET] = depth

        arr[contactArrIdx + NORMAL_LAMBDA_OFFSET] = normalLambda
        arr[contactArrIdx + T1_LAMBDA_OFFSET] = t1Lambda
        arr[contactArrIdx + T2_LAMBDA_OFFSET] = t2Lambda
    }

    override fun load(other: A2AManifoldCollection, otherManifoldIdx: Int) {
        val idx = cursor * manifoldDataSize
        cursor++
        grow(idx + manifoldDataSize)

        System.arraycopy(other.arr, otherManifoldIdx * manifoldDataSize, arr, idx, manifoldDataSize)
    }

    fun clear() {
        cursor = 0
    }

    fun isEmpty(): Boolean {
        return cursor == 0
    }

    override fun numContacts(manifoldIdx: Int): Int {
        return arr[manifoldIdx * manifoldDataSize + CONTACT_NUM_OFFSET].toRawBits()
    }

    fun setBodyData(
        manifoldIdx: Int,
        first: ActiveBody,
        second: ActiveBody,
    ) {
        val idx = manifoldIdx * manifoldDataSize

        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(first.idx)
        arr[idx + BODY_A_POS_X_OFFSET] = first.pos.x.toFloat()
        arr[idx + BODY_A_POS_Y_OFFSET] = first.pos.y.toFloat()
        arr[idx + BODY_A_POS_Z_OFFSET] = first.pos.z.toFloat()
        arr[idx + BODY_A_INVERSE_MASS_OFFSET] = first.inverseMass.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = first.inverseInertia.m00.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = first.inverseInertia.m01.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = first.inverseInertia.m02.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = first.inverseInertia.m10.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = first.inverseInertia.m11.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = first.inverseInertia.m12.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = first.inverseInertia.m20.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = first.inverseInertia.m21.toFloat()
        arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = first.inverseInertia.m22.toFloat()

        arr[idx + BODY_B_IDX_OFFSET] = Float.fromBits(second.idx)
        arr[idx + BODY_B_POS_X_OFFSET] = second.pos.x.toFloat()
        arr[idx + BODY_B_POS_Y_OFFSET] = second.pos.y.toFloat()
        arr[idx + BODY_B_POS_Z_OFFSET] = second.pos.z.toFloat()
        arr[idx + BODY_B_INVERSE_MASS_OFFSET] = second.inverseMass.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XX_OFFSET] = second.inverseInertia.m00.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XY_OFFSET] = second.inverseInertia.m01.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_XZ_OFFSET] = second.inverseInertia.m02.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YX_OFFSET] = second.inverseInertia.m10.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YY_OFFSET] = second.inverseInertia.m11.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_YZ_OFFSET] = second.inverseInertia.m12.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZX_OFFSET] = second.inverseInertia.m20.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZY_OFFSET] = second.inverseInertia.m21.toFloat()
        arr[idx + BODY_B_INVERSE_INERTIA_ZZ_OFFSET] = second.inverseInertia.m22.toFloat()
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

// Manifold data layout
private const val MANIFOLD_PREFIX_SIZE = 31
private const val CONTACT_DATA_SIZE = 19 // 16 floats per contact

private const val MANIFOLD_ID_OFFSET = 0 // 2 floats (long as 2 ints)

// Active body data (only body A - no body B data)
private const val CONTACT_NUM_OFFSET = 2

private const val BODY_A_IDX_OFFSET = 3 // 1 float
private const val BODY_A_POS_X_OFFSET = 4 // 3 floats
private const val BODY_A_POS_Y_OFFSET = 5
private const val BODY_A_POS_Z_OFFSET = 6
private const val BODY_A_INVERSE_MASS_OFFSET = 7 // 1 float
private const val BODY_A_INVERSE_INERTIA_XX_OFFSET = 8 // 9 floats (3x3 matrix)
private const val BODY_A_INVERSE_INERTIA_XY_OFFSET = 9
private const val BODY_A_INVERSE_INERTIA_XZ_OFFSET = 10
private const val BODY_A_INVERSE_INERTIA_YX_OFFSET = 11
private const val BODY_A_INVERSE_INERTIA_YY_OFFSET = 12
private const val BODY_A_INVERSE_INERTIA_YZ_OFFSET = 13
private const val BODY_A_INVERSE_INERTIA_ZX_OFFSET = 14
private const val BODY_A_INVERSE_INERTIA_ZY_OFFSET = 15
private const val BODY_A_INVERSE_INERTIA_ZZ_OFFSET = 16

private const val BODY_B_IDX_OFFSET = 17 // 1 float
private const val BODY_B_POS_X_OFFSET = 18 // 3 floats
private const val BODY_B_POS_Y_OFFSET = 19
private const val BODY_B_POS_Z_OFFSET = 20
private const val BODY_B_INVERSE_MASS_OFFSET = 21 // 1 float
private const val BODY_B_INVERSE_INERTIA_XX_OFFSET = 22 // 9 floats (3x3 matrix)
private const val BODY_B_INVERSE_INERTIA_XY_OFFSET = 23
private const val BODY_B_INVERSE_INERTIA_XZ_OFFSET = 24
private const val BODY_B_INVERSE_INERTIA_YX_OFFSET = 25
private const val BODY_B_INVERSE_INERTIA_YY_OFFSET = 26
private const val BODY_B_INVERSE_INERTIA_YZ_OFFSET = 27
private const val BODY_B_INVERSE_INERTIA_ZX_OFFSET = 28
private const val BODY_B_INVERSE_INERTIA_ZY_OFFSET = 29
private const val BODY_B_INVERSE_INERTIA_ZZ_OFFSET = 30

private const val CONTACTS_OFFSET = 31 // Start of contacts array

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