package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.physics.IContact
import com.ixume.udar.physicsWorld

class CompositeCompositeContactGenerator(
    val composite: Composite
) : Collidable {
    override fun capableCollision(other: Body): Capability {
        return Capability(other is Composite, 0)
    }

    override fun collides(other: Body): List<IContact> {
        require(other is Composite)

        val pw = other.world.physicsWorld!!

        val contacts = mutableListOf<IContact>()

        for (myPart in composite.parts) {
            for (otherPart in other.parts) {
                val can = myPart.capableCollision(otherPart)
                if (!can.capable) continue

                pw.debugData.totalPairs++
                pw.debugData.totalCompositePairs++

                if (!myPart.boundingBox.overlaps(otherPart.boundingBox)) continue

                val d = myPart.pos.distance(otherPart.pos)
                if (d > myPart.radius + otherPart.radius) {
                    pw.debugData.missedEarlies++
                    continue
                }

                pw.debugData.totalCompositeCollisionChecks++

                val result = myPart.collides(otherPart)

                if (result.isNotEmpty()) {
                    pw.debugData.compositeCollisions++
                }

                contacts += result
            }
        }

        return contacts
    }
}