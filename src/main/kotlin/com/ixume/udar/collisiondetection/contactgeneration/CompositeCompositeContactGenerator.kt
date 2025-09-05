package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.A2ACollidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Composite
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.physics.contact.A2AContactCollection
import com.ixume.udar.physicsWorld

class CompositeCompositeContactGenerator(
    val composite: Composite,
) : A2ACollidable {
    override fun capableCollision(other: ActiveBody): Int {
        return if (other is Composite) 0 else -1
    }

    override fun collides(other: ActiveBody, math: LocalMathUtil, out: A2AContactCollection): Boolean {
        require(other is Composite)

        val pw = other.world.physicsWorld!!
        var collided = false

        val buf = math.compositeUtil.buf
        buf.clear()

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

                val result = myPart.collides(otherPart, math, buf)

                if (result) {
                    collided = true
                    pw.debugData.compositeCollisions++
                }
            }
        }

        var i = 0
        while (i != -1) {
            buf.aID(i, composite.uuid)
            buf.bodyAIdx(i, composite.id)

            i = buf.nextIdx(i)
        }

        return collided
    }
}