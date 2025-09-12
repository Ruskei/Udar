package com.ixume.udar.physics.contact

import com.ixume.udar.collisiondetection.local.LocalMathUtil

class A2AContactDataBuffer(private val numContacts: Int) {
    val arr = FloatArray(CONTACT_DATA_SIZE * numContacts)
    var cursor = 0
    
    private var maxDepth = -Float.MAX_VALUE

    fun clear() {
        maxDepth = -Float.MAX_VALUE
        cursor = 0
    }

    fun size(): Int {
        return cursor
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