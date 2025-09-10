package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil

interface A2SManifoldCollection {
    fun addManifold(
        activeBody: ActiveBody,
        contactID: Long,
        buf: A2SContactDataBuffer,
    )
    
    fun addSingleManifold(
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
    )
}