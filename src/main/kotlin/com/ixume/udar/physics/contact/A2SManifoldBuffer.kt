package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import org.joml.Matrix3f
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.locks.ReentrantReadWriteLock
import kotlin.concurrent.read
import kotlin.concurrent.write
import kotlin.math.max

class A2SManifoldBuffer(maxContactNum: Int) : A2SManifoldCollection {
    override var arr = FloatArray(0)
    private val lock = ReentrantReadWriteLock()
    private val cursor = AtomicInteger(0)
    
    val numContacts = AtomicInteger(0)

    private val manifoldDataSize = MANIFOLD_PREFIX_SIZE + maxContactNum * CONTACT_DATA_SIZE

    val maxContactArrSize = maxContactNum * CONTACT_DATA_SIZE
    
    fun size(): Int {
        return cursor.get()
    }

    override fun addManifold(
        activeBody: ActiveBody,
        contactID: Long,
        buf: A2SContactDataBuffer,
    ) {
        if (buf.size() == 0) return
        val idx = cursor.andIncrement * manifoldDataSize
        lock.read {
            if (idx + manifoldDataSize < arr.size) {
                // Store contact ID (2 floats)
                arr[idx + MANIFOLD_ID_OFFSET] = Float.fromBits(contactID.toInt())
                arr[idx + MANIFOLD_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

                arr[idx + CONTACT_NUM_OFFSET] = Float.fromBits(buf.cursor)
                numContacts.addAndGet(buf.cursor)

                arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(activeBody.idx)
                arr[idx + BODY_A_POS_X_OFFSET] = activeBody.pos.x.toFloat()
                arr[idx + BODY_A_POS_Y_OFFSET] = activeBody.pos.y.toFloat()
                arr[idx + BODY_A_POS_Z_OFFSET] = activeBody.pos.z.toFloat()
                arr[idx + BODY_A_INVERSE_MASS_OFFSET] = activeBody.inverseMass.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = activeBody.inverseInertia.m00.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = activeBody.inverseInertia.m01.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = activeBody.inverseInertia.m02.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = activeBody.inverseInertia.m10.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = activeBody.inverseInertia.m11.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = activeBody.inverseInertia.m12.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = activeBody.inverseInertia.m20.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = activeBody.inverseInertia.m21.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = activeBody.inverseInertia.m22.toFloat()

                check(buf.dataSize() <= maxContactArrSize)
                System.arraycopy(buf.arr, 0, arr, idx + CONTACTS_OFFSET, buf.dataSize())
                
                return
            }
        }
        
        lock.write {
            grow(idx + manifoldDataSize)

            // Store contact ID (2 floats)
            arr[idx + MANIFOLD_ID_OFFSET] = Float.fromBits(contactID.toInt())
            arr[idx + MANIFOLD_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

            arr[idx + CONTACT_NUM_OFFSET] = Float.fromBits(buf.cursor)
            numContacts.addAndGet(buf.cursor)

            arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(activeBody.idx)
            arr[idx + BODY_A_POS_X_OFFSET] = activeBody.pos.x.toFloat()
            arr[idx + BODY_A_POS_Y_OFFSET] = activeBody.pos.y.toFloat()
            arr[idx + BODY_A_POS_Z_OFFSET] = activeBody.pos.z.toFloat()
            arr[idx + BODY_A_INVERSE_MASS_OFFSET] = activeBody.inverseMass.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = activeBody.inverseInertia.m00.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = activeBody.inverseInertia.m01.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = activeBody.inverseInertia.m02.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = activeBody.inverseInertia.m10.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = activeBody.inverseInertia.m11.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = activeBody.inverseInertia.m12.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = activeBody.inverseInertia.m20.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = activeBody.inverseInertia.m21.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = activeBody.inverseInertia.m22.toFloat()

            check(buf.dataSize() <= maxContactArrSize)
            check(idx + CONTACTS_OFFSET + buf.arr.size <= arr.size) {"""
                idx: $idx, manifoldDataSize: $manifoldDataSize
                buf.arr.size: ${buf.arr.size}
            """.trimIndent()}
            System.arraycopy(buf.arr, 0, arr, idx + CONTACTS_OFFSET, buf.dataSize())
        }
    }

    override fun addSingleManifold(
        activeBody: ActiveBody,
        contactID: Long,

        pointAX: Float,
        pointAY: Float,
        pointAZ: Float,

        normX: Float,
        normY: Float,
        normZ: Float,

        depth: Float,
        math: LocalMathUtil,

        normalLambda: Float,
        t1Lambda: Float,
        t2Lambda: Float,
    ) {
        numContacts.incrementAndGet()
        val idx = cursor.andIncrement * manifoldDataSize
        lock.read {
            if (idx + manifoldDataSize < arr.size) {
                val norm = math._n.set(normX, normY, normZ)
                val t1 = math._t1v.set(1f).orthogonalizeUnit(norm)
                val t2 = math._t2v.set(t1).cross(norm).normalize()

                arr[idx + MANIFOLD_ID_OFFSET] = Float.fromBits(contactID.toInt())
                arr[idx + MANIFOLD_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

                arr[idx + CONTACT_NUM_OFFSET] = Float.fromBits(1)

                arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(activeBody.idx)
                arr[idx + BODY_A_POS_X_OFFSET] = activeBody.pos.x.toFloat()
                arr[idx + BODY_A_POS_Y_OFFSET] = activeBody.pos.y.toFloat()
                arr[idx + BODY_A_POS_Z_OFFSET] = activeBody.pos.z.toFloat()
                arr[idx + BODY_A_INVERSE_MASS_OFFSET] = activeBody.inverseMass.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = activeBody.inverseInertia.m00.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = activeBody.inverseInertia.m01.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = activeBody.inverseInertia.m02.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = activeBody.inverseInertia.m10.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = activeBody.inverseInertia.m11.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = activeBody.inverseInertia.m12.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = activeBody.inverseInertia.m20.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = activeBody.inverseInertia.m21.toFloat()
                arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = activeBody.inverseInertia.m22.toFloat()

                val contactArrIdx = idx + CONTACTS_OFFSET

                arr[contactArrIdx + POINT_A_X_OFFSET] = pointAX
                arr[contactArrIdx + POINT_A_Y_OFFSET] = pointAY
                arr[contactArrIdx + POINT_A_Z_OFFSET] = pointAZ

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
                
                return
            }
        }
        
        lock.write {
            grow(idx + manifoldDataSize)

            val norm = math._n.set(normX, normY, normZ)
            val t1 = math._t1v.set(1f).orthogonalizeUnit(norm)
            val t2 = math._t2v.set(t1).cross(norm).normalize()

            arr[idx + MANIFOLD_ID_OFFSET] = Float.fromBits(contactID.toInt())
            arr[idx + MANIFOLD_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

            arr[idx + CONTACT_NUM_OFFSET] = Float.fromBits(1)

            arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(activeBody.idx)
            arr[idx + BODY_A_POS_X_OFFSET] = activeBody.pos.x.toFloat()
            arr[idx + BODY_A_POS_Y_OFFSET] = activeBody.pos.y.toFloat()
            arr[idx + BODY_A_POS_Z_OFFSET] = activeBody.pos.z.toFloat()
            arr[idx + BODY_A_INVERSE_MASS_OFFSET] = activeBody.inverseMass.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_XX_OFFSET] = activeBody.inverseInertia.m00.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_XY_OFFSET] = activeBody.inverseInertia.m01.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_XZ_OFFSET] = activeBody.inverseInertia.m02.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_YX_OFFSET] = activeBody.inverseInertia.m10.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_YY_OFFSET] = activeBody.inverseInertia.m11.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_YZ_OFFSET] = activeBody.inverseInertia.m12.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_ZX_OFFSET] = activeBody.inverseInertia.m20.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_ZY_OFFSET] = activeBody.inverseInertia.m21.toFloat()
            arr[idx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET] = activeBody.inverseInertia.m22.toFloat()

            val contactArrIdx = idx + CONTACTS_OFFSET

            arr[contactArrIdx + POINT_A_X_OFFSET] = pointAX
            arr[contactArrIdx + POINT_A_Y_OFFSET] = pointAY
            arr[contactArrIdx + POINT_A_Z_OFFSET] = pointAZ

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

            arr[contactArrIdx + NORMAL_LAMBDA_OFFSET] = 0f
            arr[contactArrIdx + T1_LAMBDA_OFFSET] = 0f
            arr[contactArrIdx + T2_LAMBDA_OFFSET] = 0f
        }
    }

    override fun load(other: A2SManifoldCollection, otherManifoldIdx: Int) {
        numContacts.addAndGet(other.numContacts(otherManifoldIdx))
        val idx = cursor.andIncrement * manifoldDataSize

        lock.read {
            if (idx + manifoldDataSize < arr.size) {
                System.arraycopy(other.arr, otherManifoldIdx * manifoldDataSize, arr, idx, manifoldDataSize)
                return
            }
        }

        lock.write {
            grow(idx + manifoldDataSize)
            System.arraycopy(other.arr, otherManifoldIdx * manifoldDataSize, arr, idx, manifoldDataSize)
        }
    }

    fun clear() {
        cursor.set(0)
        numContacts.set(0)
    }

    fun isEmpty(): Boolean {
        return cursor.get() == 0
    }
    
    override fun numContacts(manifoldIdx: Int): Int {
        return arr[manifoldIdx * manifoldDataSize + CONTACT_NUM_OFFSET].toRawBits()
    }

    fun manifoldID(manifoldIdx: Int): Long {
        val baseIdx = manifoldIdx * manifoldDataSize
        val low = arr[baseIdx + MANIFOLD_ID_OFFSET].toRawBits().toLong()
        val high = arr[baseIdx + MANIFOLD_ID_OFFSET + 1].toRawBits().toLong()
        return (high shl 32) or (low and 0xFFFFFFFFL)
    }

    fun bodyIdx(manifoldIdx: Int): Int {
        return arr[manifoldIdx * manifoldDataSize + BODY_A_IDX_OFFSET].toRawBits()
    }

    fun bodyX(manifoldIdx: Int): Float {
        return arr[manifoldIdx * manifoldDataSize + BODY_A_POS_X_OFFSET]
    }

    fun bodyY(manifoldIdx: Int): Float {
        return arr[manifoldIdx * manifoldDataSize + BODY_A_POS_Y_OFFSET]
    }

    fun bodyZ(manifoldIdx: Int): Float {
        return arr[manifoldIdx * manifoldDataSize + BODY_A_POS_Z_OFFSET]
    }
    
    fun bodyIM(manifoldIdx: Int): Float {
        return arr[manifoldIdx * manifoldDataSize + BODY_A_INVERSE_MASS_OFFSET]
    }
    
    fun bodyII(manifoldIdx: Int, out: Matrix3f): Matrix3f {
        val baseIdx = manifoldIdx * manifoldDataSize
        return out.set(
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XX_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XY_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XZ_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YX_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YY_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YZ_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZX_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZY_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET]
        )
    }

    // Contact point accessors
    fun pointAX(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + POINT_A_X_OFFSET]
    }

    fun pointAY(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + POINT_A_Y_OFFSET]
    }

    fun pointAZ(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + POINT_A_Z_OFFSET]
    }

    // Normal vector accessors
    fun normX(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + NORM_X_OFFSET]
    }

    fun normY(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + NORM_Y_OFFSET]
    }

    fun normZ(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + NORM_Z_OFFSET]
    }

    // Tangent 1 vector accessors
    fun t1X(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T1_X_OFFSET]
    }

    fun t1Y(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T1_Y_OFFSET]
    }

    fun t1Z(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T1_Z_OFFSET]
    }

    // Tangent 2 vector accessors
    fun t2X(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T2_X_OFFSET]
    }

    fun t2Y(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T2_Y_OFFSET]
    }

    fun t2Z(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T2_Z_OFFSET]
    }

    // Depth accessor
    fun depth(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + DEPTH_OFFSET]
    }

    // Lambda accessors
    fun normalLambda(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + NORMAL_LAMBDA_OFFSET]
    }

    fun setNormalLambda(manifoldIdx: Int, contactIdx: Int, value: Float) {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        arr[baseIdx + NORMAL_LAMBDA_OFFSET] = value
    }

    fun t1Lambda(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T1_LAMBDA_OFFSET]
    }

    fun setT1Lambda(manifoldIdx: Int, contactIdx: Int, value: Float) {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        arr[baseIdx + T1_LAMBDA_OFFSET] = value
    }

    fun t2Lambda(manifoldIdx: Int, contactIdx: Int): Float {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        return arr[baseIdx + T2_LAMBDA_OFFSET]
    }

    fun setT2Lambda(manifoldIdx: Int, contactIdx: Int, value: Float) {
        val baseIdx = manifoldIdx * manifoldDataSize + CONTACTS_OFFSET + contactIdx * CONTACT_DATA_SIZE
        arr[baseIdx + T2_LAMBDA_OFFSET] = value
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

// Manifold data layout
private const val MANIFOLD_PREFIX_SIZE = 17 // 2 floats (long as 2 ints)
private const val CONTACT_DATA_SIZE = 16 // 16 floats per contact

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
private const val CONTACTS_OFFSET = 17 // Start of contacts array

// Contact data layout within each contact (16 floats per contact)
private const val POINT_A_X_OFFSET = 0 // 3 floats
private const val POINT_A_Y_OFFSET = 1
private const val POINT_A_Z_OFFSET = 2

private const val NORM_X_OFFSET = 3 // 3 floats
private const val NORM_Y_OFFSET = 4
private const val NORM_Z_OFFSET = 5

private const val T1_X_OFFSET = 6 // 3 floats
private const val T1_Y_OFFSET = 7
private const val T1_Z_OFFSET = 8

private const val T2_X_OFFSET = 9 // 3 floats
private const val T2_Y_OFFSET = 10
private const val T2_Z_OFFSET = 11

private const val DEPTH_OFFSET = 12 // 1 float

private const val NORMAL_LAMBDA_OFFSET = 13 // 3 floats
private const val T1_LAMBDA_OFFSET = 14
private const val T2_LAMBDA_OFFSET = 15