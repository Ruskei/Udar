package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.Contact
import com.ixume.udar.physicsWorld

class CompositeCompositeContactGenerator(
    val composite: Composite
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if (other is Composite) 0 else -1
    }

    override fun collides(other: Body, math: LocalMathUtil): List<Contact> {
        require(other is Composite)

        val pw = other.world.physicsWorld!!

        val contacts = mutableListOf<Contact>()

        for (myPart in composite.parts) {
            for (otherPart in other.parts) {
                val can = myPart.capableCollision(otherPart)
                if (can < 0) continue

                pw.debugData.totalPairs++
                pw.debugData.totalCompositePairs++

                if (!myPart.fatBB.overlaps(otherPart.fatBB)) continue

                val d = myPart.pos.distance(otherPart.pos)
                if (d > myPart.radius + otherPart.radius) {
                    pw.debugData.missedEarlies++
                    continue
                }

                pw.debugData.totalCompositeCollisionChecks++

                val result = myPart.collides(otherPart, math)

                if (result.isNotEmpty()) {
                    pw.debugData.compositeCollisions++
                }

                contacts += result
            }
        }

        return contacts
    }
}