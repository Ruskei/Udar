package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil

interface A2AManifoldCollection {
    val arr: FloatArray
    
    fun addManifold(
        first: ActiveBody,
        second: ActiveBody,
        contactID: Long,
        buf: A2AContactDataBuffer,
    )

    fun addSingleManifold(
        first: ActiveBody,
        second: ActiveBody,
        contactID: Long,

        pointAX: Float, pointAY: Float, pointAZ: Float,
        pointBX: Float, pointBY: Float, pointBZ: Float,
        normX: Float, normY: Float, normZ: Float,

        depth: Float,
        math: LocalMathUtil,

        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    )

    fun load(other: A2AManifoldCollection, otherManifoldIdx: Int)

    fun numContacts(manifoldIdx: Int): Int
}