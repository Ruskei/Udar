package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.LocalMathUtil
import com.ixume.udar.collisiondetection.capability.Projectable
import com.ixume.udar.physics.Contact
import org.joml.Vector3d

class SATContactGenerator(
    val activeBody: ActiveBody,
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if(other.isConvex && other is Projectable) 0 else -1
    }

    private val _myEdgeX = Vector3d()
    private val _myEdgeY = Vector3d()
    private val _myEdgeZ = Vector3d()

    override fun collides(other: Body, math: LocalMathUtil): List<Contact> {
        require(other is ActiveBody)

        val otherAxiss = listOf<Vector3d>(
            _myEdgeX.set(1.0, 0.0, 0.0).rotate(other.q).normalize(),
            _myEdgeY.set(0.0, 1.0, 0.0).rotate(other.q).normalize(),
            _myEdgeZ.set(0.0, 0.0, 1.0).rotate(other.q).normalize(),
        )

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