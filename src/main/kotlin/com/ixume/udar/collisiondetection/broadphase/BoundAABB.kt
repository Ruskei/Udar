package com.ixume.udar.collisiondetection.broadphase

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.dynamicaabb.AABBNode

/**
 * AABB that is bound to an active body
 */
class BoundAABB(
    minX: Double,
    minY: Double,
    minZ: Double,
    maxX: Double,
    maxY: Double,
    maxZ: Double,
    node: AABBNode? = null,
) : AABB(
    minX,
    minY,
    minZ,
    maxX,
    maxY,
    maxZ,
    node,
) {
    var body: ActiveBody? = null
}