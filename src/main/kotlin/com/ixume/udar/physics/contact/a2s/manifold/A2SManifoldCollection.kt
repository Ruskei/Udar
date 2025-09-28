package com.ixume.udar.physics.contact.a2s.manifold

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.a2s.A2SContactDataBuffer

interface A2SManifoldCollection {
    val arr: FloatArray

    fun addManifold(
        activeBody: ActiveBody,
        contactID: Long,
        buf: A2SContactDataBuffer,
    )

    fun addSingleManifold(
        activeBody: ActiveBody,
        contactID: Long,

        pointAX: Float, pointAY: Float, pointAZ: Float,

        normX: Float, normY: Float, normZ: Float,

        depth: Float,
        math: LocalMathUtil,

        normalLambda: Float, t1Lambda: Float, t2Lambda: Float,
    )

    fun load(other: A2SManifoldCollection, otherManifoldIdx: Int)

    fun numContacts(manifoldIdx: Int): Int
}