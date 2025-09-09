package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.A2SCollidable
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaceSortedList
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.physics.contact.A2SContactCollection
import com.ixume.udar.physics.contact.A2SManifoldCollection
import java.util.concurrent.atomic.AtomicReference

class EnvironmentContactGenerator2(
    val activeBody: ActiveBody,
) : A2SCollidable {
    val prevBB: AABB = AABB(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    override fun capableCollision(other: EnvironmentBody): Int {
        return 0
    }

    val meshFaces = AtomicReference<List<MeshFaceSortedList>>(emptyList())
    val meshEdges = AtomicReference<List<FlattenedEdgeQuadtree>>(emptyList())

    override fun collides(other: EnvironmentBody, math: LocalMathUtil, out: A2SManifoldCollection): Boolean {
        return math.envContactUtil.collides(this, activeBody, other, out)
    }

    fun tick() {
        activeBody.tightBB.writeTo(prevBB)
    }
}