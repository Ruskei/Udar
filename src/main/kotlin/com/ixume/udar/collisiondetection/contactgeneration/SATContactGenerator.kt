package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.LocalMathUtil
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.capability.Projectable
import com.ixume.udar.collisiondetection.edgeCrosses
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.IContact
import org.joml.Vector3d

class SATContactGenerator(
    val activeBody: ActiveBody,
) : Collidable {
    override fun capableCollision(other: Body): Capability {
        return Capability(other.isConvex && other is Projectable, 0)
    }

    override fun collides(other: Body, math: LocalMathUtil): List<IContact> {
        require(other is ActiveBody)

        val myAxiss = listOf<Vector3d>(
            Vector3d(1.0, 0.0, 0.0).rotate(activeBody.q).normalize(),
            Vector3d(0.0, 1.0, 0.0).rotate(activeBody.q).normalize(),
            Vector3d(0.0, 0.0, 1.0).rotate(activeBody.q).normalize(),
        )

        val otherAxiss = listOf<Vector3d>(
            Vector3d(1.0, 0.0, 0.0).rotate(other.q).normalize(),
            Vector3d(0.0, 1.0, 0.0).rotate(other.q).normalize(),
            Vector3d(0.0, 0.0, 1.0).rotate(other.q).normalize(),
        )

        val edgeAxiss = edgeCrosses(myAxiss, otherAxiss)

        val axiss = ArrayList<Vector3d>(myAxiss.size + otherAxiss.size + edgeAxiss.size)
        axiss += myAxiss
        axiss += otherAxiss
        axiss += edgeAxiss

        val r = math.collidesSAT(
            activeBody = activeBody,
            otherVertices = other.vertices,
            otherAxiss = otherAxiss,
            otherEdges = otherAxiss,
            findAll = false,
            collideMyAxiss = true,
        ) ?: return emptyList()

        return r.map {
            Contact(
                first = activeBody,
                second = other,
                result = it,
            )
        }
    }
}