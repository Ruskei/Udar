package com.ixume.udar.physics.contact

import com.ixume.udar.body.active.ActiveBody

interface A2AContactCollection : ContactOut {
    fun addCollision(
        first: ActiveBody,
        second: ActiveBody,
        pointAX: Double, pointAY: Double, pointAZ: Double,
        pointBX: Double, pointBY: Double, pointBZ: Double,
        normX: Double, normY: Double, normZ: Double,
        depth: Double,
        contactID: Long = 0L,
    )
}