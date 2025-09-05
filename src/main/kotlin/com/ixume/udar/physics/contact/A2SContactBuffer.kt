package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.LOWER_MASK
import org.joml.Matrix3f
import org.joml.Vector3d
import java.util.*
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.locks.ReentrantReadWriteLock
import kotlin.concurrent.read
import kotlin.concurrent.write
import kotlin.math.max

/*
Thread-safe contact storage for Active-to-Static contacts
Only stores data about the active body, omitting all static body data
*/

class A2SContactBuffer {
    @Volatile
    internal var arr = FloatArray(0)
    private val lock = ReentrantReadWriteLock()

    private val cursor = AtomicInteger(0)

    fun add(
        activeBody: ActiveBody,
        contactID: Long,

        pointAX: Float,
        pointAY: Float,
        pointAZ: Float,

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
        val idx = cursor.getAndIncrement() * A2S_CONTACT_DATA_SIZE

        lock.read {
            if (idx + A2S_CONTACT_DATA_SIZE < arr.size) {
                _add(
                    idx,
                    activeBody,

                    contactID,

                    pointAX,
                    pointAY,
                    pointAZ,

                    normX,
                    normY,
                    normZ,

                    t1X,
                    t1Y,
                    t1Z,

                    t2X,
                    t2Y,
                    t2Z,

                    depth,

                    normalLambda,
                    t1Lambda,
                    t2Lambda,
                )

                return
            }
        }

        lock.write {
            grow(idx + A2S_CONTACT_DATA_SIZE)

            _add(
                idx,
                activeBody,

                contactID,

                pointAX,
                pointAY,
                pointAZ,

                normX,
                normY,
                normZ,

                t1X,
                t1Y,
                t1Z,

                t2X,
                t2Y,
                t2Z,

                depth,

                normalLambda,
                t1Lambda,
                t2Lambda,
            )
        }
    }

    private fun _add(
        idx: Int,
        activeBody: ActiveBody,

        contactID: Long,

        pointAX: Float,
        pointAY: Float,
        pointAZ: Float,

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
        val aID = activeBody.uuid
        
        // Store active body UUID (4 floats)
        arr[idx + AID_OFFSET] = Float.fromBits(aID.mostSignificantBits.toInt())
        arr[idx + AID_OFFSET + 1] = Float.fromBits((aID.mostSignificantBits ushr 32).toInt())
        arr[idx + AID_OFFSET + 2] = Float.fromBits(aID.leastSignificantBits.toInt())
        arr[idx + AID_OFFSET + 3] = Float.fromBits((aID.leastSignificantBits ushr 32).toInt())
        
        // Store contact ID (2 floats)
        arr[idx + CONTACT_ID_OFFSET] = Float.fromBits(contactID.toInt())
        arr[idx + CONTACT_ID_OFFSET + 1] = Float.fromBits((contactID ushr 32).toInt())

        // Store only point A (3 floats) - no point B for static contacts
        arr[idx + POINT_A_X_OFFSET] = pointAX
        arr[idx + POINT_A_Y_OFFSET] = pointAY
        arr[idx + POINT_A_Z_OFFSET] = pointAZ

        // Store normal and tangent vectors (9 floats)
        arr[idx + NORM_X_OFFSET] = normX
        arr[idx + NORM_Y_OFFSET] = normY
        arr[idx + NORM_Z_OFFSET] = normZ

        arr[idx + T1_X_OFFSET] = t1X
        arr[idx + T1_Y_OFFSET] = t1Y
        arr[idx + T1_Z_OFFSET] = t1Z

        arr[idx + T2_X_OFFSET] = t2X
        arr[idx + T2_Y_OFFSET] = t2Y
        arr[idx + T2_Z_OFFSET] = t2Z

        // Store depth and lambda values (4 floats)
        arr[idx + DEPTH_OFFSET] = depth
        arr[idx + NORMAL_LAMBDA_OFFSET] = normalLambda
        arr[idx + T1_LAMBDA_OFFSET] = t1Lambda
        arr[idx + T2_LAMBDA_OFFSET] = t2Lambda

        // Store active body data (17 floats: idx + pos + inverse mass + 3x3 inverse inertia)
        arr[idx + BODY_A_IDX_OFFSET] = Float.fromBits(activeBody.id)
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
    }

    fun addCollision(
        activeBody: ActiveBody,
        pointAX: Double, pointAY: Double, pointAZ: Double,
        normX: Double, normY: Double, normZ: Double,
        depth: Double,
        contactID: Long = 0L
    ) {
        // Calculate tangent vectors following Contact class logic
        val norm = Vector3d(normX, normY, normZ)
        val t1 = Vector3d(1.0).orthogonalizeUnit(norm)
        val t2 = Vector3d(t1).cross(norm).normalize()

        // Add contact with computed tangents and default lambda values
        add(
            activeBody = activeBody,
            contactID = contactID,
            pointAX = pointAX.toFloat(),
            pointAY = pointAY.toFloat(),
            pointAZ = pointAZ.toFloat(),
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
        cursor.set(0)
    }

    fun isEmpty(): Boolean {
        return cursor.get() == 0
    }

    fun size(): Int {
        return cursor.get()
    }

    // UUID accessor (only active body)
    fun aID(idx: Int): UUID {
        val baseIdx = idx * A2S_CONTACT_DATA_SIZE
        val msb1 = arr[baseIdx + AID_OFFSET].toRawBits().toLong()
        val msb2 = arr[baseIdx + AID_OFFSET + 1].toRawBits().toLong()
        val lsb1 = arr[baseIdx + AID_OFFSET + 2].toRawBits().toLong()
        val lsb2 = arr[baseIdx + AID_OFFSET + 3].toRawBits().toLong()
        val mostSignificant = (msb2 shl 32) or (msb1 and LOWER_MASK)
        val leastSignificant = (lsb2 shl 32) or (lsb1 and LOWER_MASK)
        return UUID(mostSignificant, leastSignificant)
    }

    fun contactID(idx: Int): Long {
        val baseIdx = idx * A2S_CONTACT_DATA_SIZE
        val low = arr[baseIdx + CONTACT_ID_OFFSET].toRawBits().toLong()
        val high = arr[baseIdx + CONTACT_ID_OFFSET + 1].toRawBits().toLong()
        return (high shl 32) or (low and LOWER_MASK)
    }

    // Point A accessors (no point B for static contacts)
    fun pointAX(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + POINT_A_X_OFFSET]
    }

    fun pointAY(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + POINT_A_Y_OFFSET]
    }

    fun pointAZ(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + POINT_A_Z_OFFSET]
    }

    // Normal and tangent vector accessors
    fun normX(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + NORM_X_OFFSET]
    }

    fun normY(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + NORM_Y_OFFSET]
    }

    fun normZ(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + NORM_Z_OFFSET]
    }

    fun t1X(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T1_X_OFFSET]
    }

    fun t1Y(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T1_Y_OFFSET]
    }

    fun t1Z(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T1_Z_OFFSET]
    }

    fun t2X(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T2_X_OFFSET]
    }

    fun t2Y(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T2_Y_OFFSET]
    }

    fun t2Z(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T2_Z_OFFSET]
    }

    // Depth and lambda accessors
    fun depth(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + DEPTH_OFFSET]
    }

    fun normalLambda(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + NORMAL_LAMBDA_OFFSET]
    }

    fun t1Lambda(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T1_LAMBDA_OFFSET]
    }

    fun t2Lambda(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + T2_LAMBDA_OFFSET]
    }

    // Active body data accessors
    fun bodyAIdx(idx: Int): Int {
        return arr[idx * A2S_CONTACT_DATA_SIZE + BODY_A_IDX_OFFSET].toBits()
    }

    fun bodyAPosX(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + BODY_A_POS_X_OFFSET]
    }

    fun bodyAPosY(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + BODY_A_POS_Y_OFFSET]
    }

    fun bodyAPosZ(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + BODY_A_POS_Z_OFFSET]
    }

    fun bodyAInverseMass(idx: Int): Float {
        return arr[idx * A2S_CONTACT_DATA_SIZE + BODY_A_INVERSE_MASS_OFFSET]
    }

    fun bodyAInverseInertia(idx: Int, out: Matrix3f): Matrix3f {
        val baseIdx = idx * A2S_CONTACT_DATA_SIZE
        return out.set(
            arr[baseIdx + BODY_A_INVERSE_INERTIA_XX_OFFSET], arr[baseIdx + BODY_A_INVERSE_INERTIA_XY_OFFSET], arr[baseIdx + BODY_A_INVERSE_INERTIA_XZ_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_YX_OFFSET], arr[baseIdx + BODY_A_INVERSE_INERTIA_YY_OFFSET], arr[baseIdx + BODY_A_INVERSE_INERTIA_YZ_OFFSET],
            arr[baseIdx + BODY_A_INVERSE_INERTIA_ZX_OFFSET], arr[baseIdx + BODY_A_INVERSE_INERTIA_ZY_OFFSET], arr[baseIdx + BODY_A_INVERSE_INERTIA_ZZ_OFFSET]
        )
    }

    // Lambda setters
    fun setNormalLambda(idx: Int, value: Float) {
        arr[idx * A2S_CONTACT_DATA_SIZE + NORMAL_LAMBDA_OFFSET] = value
    }

    fun setT1Lambda(idx: Int, value: Float) {
        arr[idx * A2S_CONTACT_DATA_SIZE + T1_LAMBDA_OFFSET] = value
    }

    fun setT2Lambda(idx: Int, value: Float) {
        arr[idx * A2S_CONTACT_DATA_SIZE + T2_LAMBDA_OFFSET] = value
    }

    private fun grow(required: Int) {
        if (arr.size >= required) return

        val newSize = max(required, arr.size * 3 / 2)
        arr = arr.copyOf(newSize)
    }
}

// A2S Contact data layout - optimized for Active-to-Static contacts (35 floats total)
private const val A2S_CONTACT_DATA_SIZE = 36

private const val AID_OFFSET = 0 // 4 floats (active body UUID as 4 ints)
private const val CONTACT_ID_OFFSET = 4 // 2 floats (long as 2 ints)

private const val POINT_A_X_OFFSET = 6 // 3 floats (only point A, no point B)
private const val POINT_A_Y_OFFSET = 7
private const val POINT_A_Z_OFFSET = 8

private const val NORM_X_OFFSET = 9 // 3 floats
private const val NORM_Y_OFFSET = 10
private const val NORM_Z_OFFSET = 11

private const val T1_X_OFFSET = 12 // 3 floats
private const val T1_Y_OFFSET = 13
private const val T1_Z_OFFSET = 14

private const val T2_X_OFFSET = 15 // 3 floats  
private const val T2_Y_OFFSET = 16
private const val T2_Z_OFFSET = 17

private const val DEPTH_OFFSET = 18 // 1 float

private const val NORMAL_LAMBDA_OFFSET = 19 // 3 floats
private const val T1_LAMBDA_OFFSET = 20
private const val T2_LAMBDA_OFFSET = 21

// Active body data (only body A - no body B data)
private const val BODY_A_IDX_OFFSET = 22 // 1 float
private const val BODY_A_POS_X_OFFSET = 23 // 3 floats
private const val BODY_A_POS_Y_OFFSET = 24
private const val BODY_A_POS_Z_OFFSET = 25
private const val BODY_A_INVERSE_MASS_OFFSET = 26 // 1 float
private const val BODY_A_INVERSE_INERTIA_XX_OFFSET = 27 // 9 floats (3x3 matrix)
private const val BODY_A_INVERSE_INERTIA_XY_OFFSET = 28
private const val BODY_A_INVERSE_INERTIA_XZ_OFFSET = 29
private const val BODY_A_INVERSE_INERTIA_YX_OFFSET = 30
private const val BODY_A_INVERSE_INERTIA_YY_OFFSET = 31
private const val BODY_A_INVERSE_INERTIA_YZ_OFFSET = 32
private const val BODY_A_INVERSE_INERTIA_ZX_OFFSET = 33
private const val BODY_A_INVERSE_INERTIA_ZY_OFFSET = 34
private const val BODY_A_INVERSE_INERTIA_ZZ_OFFSET = 35
