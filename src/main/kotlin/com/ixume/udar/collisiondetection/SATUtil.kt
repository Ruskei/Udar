package com.ixume.udar.collisiondetection

import com.ixume.udar.applyIf
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.capability.Projectable
import com.ixume.udar.collisiondetection.contactgeneration.GJKEPAContactGenerator.Companion.closestPointsBetweenSegments
import com.ixume.udar.physics.CollisionResult
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
    val (myMin, myMax) = my.project(axis)
    val (otherMin, otherMax) = other.project(axis)

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

fun List<Vector3d>.projectable(): Projectable {
    return object : Projectable {
        override fun project(axis: Vector3d): Pair<Double, Double> {
            var min = Double.MAX_VALUE
            var max = -Double.MAX_VALUE

            if (isEmpty()) return 0.0 to 0.0

            for (v in this@projectable) {
                val s = v.dot(axis)
                min = min(min, s)
                max = max(max, s)
            }

            return min to max
        }
    }
}

fun collidesSAT(
    activeBody: ActiveBody,
    otherVertices: List<Vector3d>,
    otherAxiss: List<Vector3d>,
    otherEdges: List<Vector3d>,
    allowedNormals: List<Vector3d>? = null,
    findAll: Boolean = false,
    collideMyAxiss: Boolean = true,
): MutableList<CollisionResult>? {
    val q = activeBody.q

    val myAxiss = if (collideMyAxiss) listOf<Vector3d>(
        Vector3d(1.0, 0.0, 0.0).rotate(q).normalize(),
        Vector3d(0.0, 1.0, 0.0).rotate(q).normalize(),
        Vector3d(0.0, 0.0, 1.0).rotate(q).normalize(),
    ) else listOf()

    val myEdges = if (collideMyAxiss) listOf(
        Vector3d(1.0, 0.0, 0.0).rotate(q).normalize(),
        Vector3d(0.0, 1.0, 0.0).rotate(q).normalize(),
        Vector3d(0.0, 0.0, 1.0).rotate(q).normalize(),
    ) else listOf()

    val edgeAxiss = edgeCrosses(otherEdges, myEdges)

    val axiss = ArrayList<Vector3d>(otherAxiss.size + myAxiss.size + edgeAxiss.size)
    axiss += otherAxiss
    axiss += myAxiss
    axiss += edgeAxiss

    val myVertices = activeBody.vertices

    var minOrder: Boolean? = null
    var minOverlap = Double.MAX_VALUE
    var minAxis: Vector3d? = null

    for (axis in axiss) {
        val (overlap, order) = cycleSAT(
            axis = axis,
            activeBody, otherVertices.projectable()
        ) ?: return null

        if (overlap < minOverlap) {
            minAxis = axis
            minOverlap = overlap
            minOrder = order
        }
    }

    minAxis!!
    minOrder!!

    if (allowedNormals != null) {
        val efa = if (minOrder) minAxis else Vector3d(minAxis).negate()
        if (!allowedNormals.all { efa.dot(it) >= 0.0 }) return null
    }

    val epsilon = 1e-10

    if (minAxis in edgeAxiss) {
        //edge-edge
        //depending on the order, find most penetrating point(s) on each body, then choose the edges from that
//            if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                println("EDGE-EDGE")
//                println("  - AXIS: $minAxis")
//                println("  - OVERLAP: $minOverlap")
//                println("  - ORDER: $minOrder")
//            }

        var myDeepestVertices = mutableListOf<Vector3d>()
        var myDeepestDistance = -Double.MAX_VALUE
        val myOrderedAxis = if (minOrder) Vector3d(minAxis).negate() else minAxis

        for (vertex in myVertices) {
            val d = vertex.dot(myOrderedAxis)
//                println("my $vertex d: $d deepest: $myDeepestDistance")
            if (abs(d - myDeepestDistance) < epsilon) {
//                    println("MERGER!")
                myDeepestVertices += vertex
                continue
            } else if (d > myDeepestDistance) {
//                    println("NEW LARGEST!")
                myDeepestDistance = d
                myDeepestVertices = mutableListOf(vertex)
            }
        }

        var otherDeepestVertices = mutableListOf<Vector3d>()
        var otherDeepestDistance = -Double.MAX_VALUE
        val otherOrderedAxis = Vector3d(myOrderedAxis).negate()

        for (vertex in otherVertices) {
            val d = vertex.dot(otherOrderedAxis)
//                println("other $vertex d: $d deepest: $otherDeepestDistance")
            if (abs(d - otherDeepestDistance) < epsilon) {
                otherDeepestVertices += vertex
                continue
            } else if (d > otherDeepestDistance) {
                otherDeepestDistance = d
                otherDeepestVertices = mutableListOf(vertex)
            }
        }

//            println("myDeepestVertices : $myDeepestDistance : $myDeepestVertices")
//            println("otherDeepestVertices : $otherDeepestDistance : $otherDeepestVertices")

        check(myDeepestVertices.size % 2 == 0)
        check(otherDeepestVertices.size % 2 == 0)

        val r = if (myDeepestVertices.size > 2 || otherDeepestVertices.size > 2) {
            var closestResult = Triple(Vector3d(), Vector3d(), Double.MAX_VALUE)
            for (i in 0..<myDeepestVertices.size step 2) {
                for (j in 0..<otherDeepestVertices.size step 2) {
                    val r = closestPointsBetweenSegments(
                        myDeepestVertices[i],
                        myDeepestVertices[i + 1],
                        otherDeepestVertices[j],
                        otherDeepestVertices[j + 1]
                    )

                    if (r.third < closestResult.third) {
                        closestResult = r
                    }
                }
            }

            check(closestResult.third != Double.MAX_VALUE)
            closestResult
        } else {
            closestPointsBetweenSegments(
                myDeepestVertices[0],
                myDeepestVertices[1],
                otherDeepestVertices[0],
                otherDeepestVertices[1]
            )
        }

//            if (PhysicsCommand.DEBUG_SAT_LEVEL > 1) {
//                println("   * myDeepestVertices: ${myDeepestVertices[0]}")
//                println("   * myDeepestVertices: ${myDeepestVertices[1]}")
//                println("   * otherDeepestVertices: ${otherDeepestVertices[0]}")
//                println("   * otherDeepestVertices: ${otherDeepestVertices[1]}")
//                println("   * DISTANCE: ${r.third}")
//            }

        return mutableListOf(
            CollisionResult(
                Vector3d(r.first).mul(0.5).add(Vector3d(r.second).mul(0.5)),
                if (minOrder) minAxis else Vector3d(minAxis).negate(),
                r.third
            )
        )
    } else {
        //face-vertex
//            if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                println("FACE-VERTEX")
//                println("  - AXIS: $minAxis")
//                println("  - OVERLAP: $minOverlap")
//                println("  - minOrder: $minOrder")
//            }

        var furthestDistance = -Double.MAX_VALUE
        var furtherVertex: Vector3d? = null

        if (myAxiss.contains(minAxis)) {
            val antiNormal = if (!minOrder) Vector3d(minAxis).negate() else Vector3d(minAxis)
//                println("OTHER AXIS")
            //other has incident
            for (vertex in otherVertices) {
                val d = vertex.dot(antiNormal)
                if (d > furthestDistance) {
                    furthestDistance = d
                    furtherVertex = vertex
                }
            }
//                if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                    println("  - POINT: $furtherVertex")
//                }

            //happens to conicide with antinormal here, but keep them as separate variables to not cause confusion
            val trueAxis = Vector3d(minAxis).applyIf(!minOrder) { negate() }

            val r = mutableListOf(
                CollisionResult(
                    furtherVertex!!,
                    trueAxis,
                    minOverlap,
                )
            )

            if (findAll) {
                for (vertex in otherVertices) {
                    val d = vertex.dot(antiNormal)
                    if (abs(d - furthestDistance) <= minOverlap) {
                        r += CollisionResult(
                            vertex,
                            trueAxis,
                            minOverlap - abs(d - furthestDistance),
                        )
                    }
                }
            }

            return r
        } else {
            val antiNormal = if (minOrder) Vector3d(minAxis).negate() else Vector3d(minAxis)
//                println("MY AXIS")
            //i have incident
            for (vertex in myVertices) {
                val d = vertex.dot(antiNormal)
                if (d > furthestDistance) {
                    furthestDistance = d
                    furtherVertex = vertex
                }
            }

//                if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                    println("  - POINT: $furtherVertex")
//                }

            val trueAxis = Vector3d(minAxis).applyIf(!minOrder) { negate() }

            val r = mutableListOf(
                CollisionResult(
                    furtherVertex!!,
                    trueAxis,
                    minOverlap,
                )
            )

            if (findAll) {
                for (vertex in myVertices) {
                    val d = vertex.dot(antiNormal)
                    if (abs(d - furthestDistance) <= minOverlap) {
                        r += CollisionResult(
                            vertex,
                            trueAxis,
                            minOverlap - abs(d - furthestDistance),
                        )
                    }
                }
            }

            return r
        }
    }

}
