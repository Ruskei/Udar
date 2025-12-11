package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.A2ACollidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldCollection
import com.ixume.udar.physicsWorld

class CompositeCompositeContactGenerator(
    val composite: Composite,
) : A2ACollidable {
    override fun capableCollision(other: ActiveBody): Int {
        return if (other is Composite) 0 else -1
    }

    override fun collides(other: ActiveBody, math: LocalMathUtil, out: A2AManifoldCollection): Boolean {
        require(other is Composite)

        var collided = false

        for (myPart in composite.parts) {
            for (otherPart in other.parts) {
                check(myPart.idx != -1)
                check(otherPart.idx != -1)
                val can = myPart.capableCollision(otherPart)
                if (can < 0) continue

                if (!myPart.tightBB.overlaps(otherPart.tightBB)) continue

                val result = myPart.collides(otherPart, math, out)

                if (result) {
                    collided = true
                }
            }
        }

        return collided
    }
}