package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.A2ACollidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.A2AContactCollection

class CuboidSATContactGenerator(
    val activeBody: ActiveBody,
) : A2ACollidable {
    override fun capableCollision(other: ActiveBody): Int {
        return if (other.isConvex) 0 else -1
    }

    override fun collides(other: ActiveBody, math: LocalMathUtil, out: A2AContactCollection): Boolean {
        return math.cuboidSATContactUtil.collides(activeBody, other, out)
    }
}