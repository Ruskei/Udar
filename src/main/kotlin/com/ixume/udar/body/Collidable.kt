package com.ixume.udar.body

import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.physics.Contact

interface Collidable {
    /**
     * @return true if this the object associated with this contact generator can collide with 'body'
     */
    fun capableCollision(other: Body): Capability
    fun collides(other: Body): List<Contact>
}