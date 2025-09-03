package com.ixume.udar.body

import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.Contact

interface Collidable {
    /**
     * @return Can collide if returns >= 0
     */
    fun capableCollision(other: Body): Int
    fun collides(other: Body, math: LocalMathUtil): List<Contact>
}