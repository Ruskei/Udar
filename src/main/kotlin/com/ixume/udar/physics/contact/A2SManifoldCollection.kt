package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody

interface A2SManifoldCollection {
    fun addManifold(
        activeBody: ActiveBody,
        contactID: Long,
        buf: ContactDataBuffer,
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
    )
}