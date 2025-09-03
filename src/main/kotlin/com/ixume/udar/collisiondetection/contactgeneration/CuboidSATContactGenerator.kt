package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.capability.Projectable
import com.ixume.udar.physics.Contact

class CuboidSATContactGenerator(
    val activeBody: ActiveBody,
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if (other.isConvex && other is Projectable) 0 else -1
    }

    override fun collides(other: Body, math: LocalMathUtil): List<Contact> {
        require(other is ActiveBody)

        return math.cuboidSATContactUtil.collides(activeBody, other)
    }
}