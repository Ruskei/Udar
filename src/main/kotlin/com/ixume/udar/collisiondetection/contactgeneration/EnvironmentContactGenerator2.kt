package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaceSortedList
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.physics.Contact
import java.util.concurrent.atomic.AtomicReference

class EnvironmentContactGenerator2(
    val activeBody: ActiveBody,
) : Collidable {
    val prevBB: AABB = AABB(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    override fun capableCollision(other: Body): Int {
        return if (other is EnvironmentBody) 0 else -1
    }

    val meshFaces = AtomicReference<List<MeshFaceSortedList>>(emptyList())
    val meshEdges = AtomicReference<List<FlattenedEdgeQuadtree>>(emptyList())

    override fun collides(other: Body, math: LocalMathUtil): List<Contact> {
        return math.envContactUtil.collides(this, activeBody, other)
    }
    
    fun tick() {
        activeBody.tightBB.writeTo(prevBB)
    }
}