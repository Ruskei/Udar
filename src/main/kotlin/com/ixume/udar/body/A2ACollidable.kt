package com.ixume.udar.body

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.A2AManifoldCollection

interface A2ACollidable {
    /**
     * @return Can collide if returns >= 0
     */
    fun capableCollision(other: ActiveBody): Int
    fun collides(other: ActiveBody, math: LocalMathUtil, out: A2AManifoldCollection): Boolean
}