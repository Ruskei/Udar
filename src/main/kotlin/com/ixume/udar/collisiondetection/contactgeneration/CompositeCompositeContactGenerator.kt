package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.physics.IContact

class CompositeCompositeContactGenerator(
    val composite: Composite
) : Collidable {
    override fun capableCollision(other: Body): Capability {
        return Capability(other is Composite, 0)
    }

    override fun collides(other: Body): List<IContact> {
        require(other is Composite)

        val contacts = mutableListOf<IContact>()

        for (myPart in composite.parts) {
            for (otherPart in other.parts) {
                val can = myPart.capableCollision(otherPart)
                if (!can.capable) continue
                if (!myPart.boundingBox.overlaps(otherPart.boundingBox)) continue

                val result = myPart.collides(otherPart)

                contacts += result
            }
        }

        return contacts
    }
}