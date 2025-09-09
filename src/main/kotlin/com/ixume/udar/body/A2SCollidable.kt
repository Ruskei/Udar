package com.ixume.udar.body

import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.A2SManifoldCollection

interface A2SCollidable {
    /**
     * @return Can collide if returns >= 0
     */
    fun capableCollision(other: EnvironmentBody): Int
    fun collides(other: EnvironmentBody, math: LocalMathUtil, out: A2SManifoldCollection): Boolean
}