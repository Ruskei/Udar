package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody

interface A2SContactCollection : ContactOut {
    fun addCollision(
        activeBody: ActiveBody,
        pointAX: Double, pointAY: Double, pointAZ: Double,
        normX: Double, normY: Double, normZ: Double,
        depth: Double,
        contactID: Long = 0L,
    )
}