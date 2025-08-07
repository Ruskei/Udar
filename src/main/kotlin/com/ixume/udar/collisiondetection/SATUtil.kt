package com.ixume.udar.collisiondetection

import com.ixume.udar.collisiondetection.capability.Projectable
import org.bukkit.World
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Simple convex collision detection via SAT
 */
/**
 * True if the given mesh, defined by its vertices and face normals, collides with any nearby block bounding boxes
 * @param edges Do not provide duplicates with normals
 */
fun collidesBlocks(
    world: World,
    vertices: List<Vector3d>,
    normals: List<Vector3d>,
    edges: List<Vector3d>,
): Boolean {
    val bb = boundingBox(vertices)
    val bbs = bb?.overlappingBlocks(world) ?: return false

    val epsilon = 1e-11

    val ees = edges.filterNot {
        it.normalize()
        (abs(it.dot(1.0, 0.0, 0.0)) < epsilon) ||
                (abs(it.dot(0.0, 1.0, 0.0)) < epsilon) ||
                (abs(it.dot(0.0, 1.0, 0.0)) < epsilon)
    }

    val nns = normals.filterNot {
        it.normalize()
        (abs(it.dot(1.0, 0.0, 0.0)) < epsilon) ||
                (abs(it.dot(0.0, 1.0, 0.0)) < epsilon) ||
                (abs(it.dot(0.0, 1.0, 0.0)) < epsilon)
                && (it in ees)
    }

    val bbns = listOf(
        Vector3d(1.0, 0.0, 0.0),
        Vector3d(0.0, 1.0, 0.0),
        Vector3d(0.0, 0.0, 1.0),
    )

    for (bb2 in bbs) {
        val vs = listOf(
            Vector3d(bb2.minX, bb2.minY, bb2.minZ),
            Vector3d(bb2.minX, bb2.minY, bb2.maxZ),
            Vector3d(bb2.minX, bb2.maxY, bb2.minZ),
            Vector3d(bb2.minX, bb2.maxY, bb2.maxZ),
            Vector3d(bb2.maxX, bb2.minY, bb2.minZ),
            Vector3d(bb2.maxX, bb2.minY, bb2.maxZ),
            Vector3d(bb2.maxX, bb2.maxY, bb2.minZ),
            Vector3d(bb2.maxX, bb2.maxY, bb2.maxZ),
        )

        if (collides(
            verticesA = vertices,
            normalsA = nns,
            edgesA = ees,
            verticesB = vs,
            normalsB = bbns,
            edgesB = bbns,
        )) return true
    }

    return false
}

/**
 * SAT collision test
 */
fun collides(
    verticesA: List<Vector3d>,
    normalsA: List<Vector3d>,
    edgesA: List<Vector3d>,
    verticesB: List<Vector3d>,
    normalsB: List<Vector3d>,
    edgesB: List<Vector3d>
): Boolean {
    //no set because it's probably not faster at small scales due to memory usage
    val axiss =
        ArrayList<Vector3d>(normalsA.size + normalsB.size + edgesA.size + edgesB.size + edgesA.size * edgesB.size)
    axiss += normalsA
    axiss += normalsB
    axiss += edgesA
    axiss += edgesB
    axiss += edgeCrosses(edgesA, edgesB)

    for (axis in axiss) {
        var minA = Double.MAX_VALUE
        var maxA = -Double.MAX_VALUE

        var minB = Double.MAX_VALUE
        var maxB = -Double.MAX_VALUE

        for (a in verticesA) {
            val s = a.dot(axis)
            minB = min(minB, s)
            maxB = max(maxB, s)
        }

        for (b in verticesB) {
            val s = b.dot(axis)
            minA = min(minA, s)
            maxA = max(maxA, s)
        }

        if (!(minA < maxB && maxA > minB)) return false
    }

    return true
}

fun edgeCrosses(
    edgesA: List<Vector3d>,
    edgesB: List<Vector3d>,
): List<Vector3d> {
    if (edgesA.isEmpty() || edgesB.isEmpty()) return listOf()
    val ls = ArrayList<Vector3d>(edgesA.size * edgesB.size)
    for (edgeA in edgesA) {
        for (edgeB in edgesB) {
            ls += Vector3d(edgeA).cross(edgeB).normalize()
        }
    }

    return ls
}

data class SATCycle(
    val overlap: Double,
    val order: Boolean,
)

fun cycleSAT(
    axis: Vector3d,
    my: Projectable,
    other: Projectable,
): SATCycle? {
    val myminmax = my.project(axis)
    val myMin = myminmax.x
    val myMax = myminmax.y
    val otherminmax = other.project(axis)
    val otherMin = otherminmax.x
    val otherMax = otherminmax.y

    var order: Boolean? = null
    val overlap = if (myMin < otherMax && myMax > otherMin) {
        //overlapping
        order = (otherMax - myMin) < (myMax - otherMin)

        //check if contained or overlapping; if contained then choose smallest distance as overlap
        if (myMin < otherMin && myMax > otherMax) {
            //i contain other
            min(myMax - otherMin, otherMax - myMin)
        } else if (otherMin < myMin && otherMax > myMax) {
            //other contains me
            min(otherMax - myMin, myMax - otherMin)
        } else {
            //just overlapping
            if (myMax > otherMax) otherMax - myMin
            else myMax - otherMin
        }
    } else 0.0

    if (overlap <= 0.0) {
        return null
    }

    return SATCycle(overlap, order!!)
}


