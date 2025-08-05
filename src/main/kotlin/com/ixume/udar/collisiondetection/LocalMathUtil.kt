package com.ixume.udar.collisiondetection

import com.ixume.udar.applyIf
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.CollisionResult
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class LocalMathUtil {
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
            val my = activeBody.project(axis)
            val other = otherVertices.project(axis)

            val (overlap, order) = cycleSAT(
                myMin = my.first, myMax = my.second,
                otherMin = other.first, otherMax = other.second,
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

    /**
     * Finds closest distance between two lines defined by a0, a and b0, b
     * @return closest point on segment a, closest point on segment b, distance, null if parallel
     */
    fun closestPointsBetweenSegments(
        a0: Vector3d,
        a1: Vector3d,
        b0: Vector3d,
        b1: Vector3d
    ): Triple<Vector3d, Vector3d, Double> {
        val epsilon = 1e-10

        val a = Vector3d(a1).sub(a0).normalize()
        val b = Vector3d(b1).sub(b0).normalize()

        val axRange = (min(a0.x, a1.x) - epsilon)..(max(a0.x, a1.x) + epsilon)
        val ayRange = (min(a0.y, a1.y) - epsilon)..(max(a0.y, a1.y) + epsilon)
        val azRange = (min(a0.z, a1.z) - epsilon)..(max(a0.z, a1.z) + epsilon)
        val bxRange = (min(b0.x, b1.x) - epsilon)..(max(b0.x, b1.x) + epsilon)
        val byRange = (min(b0.y, b1.y) - epsilon)..(max(b0.y, b1.y) + epsilon)
        val bzRange = (min(b0.z, b1.z) - epsilon)..(max(b0.z, b1.z) + epsilon)

        //closest points is either on the lines, on a vertex and a line, or between 2 vertices
        //if line-line is valid, then that's the answer
        //then check vertex-line
        //if vertex-line is valid then that's the answer
        //otherwise check vertex-vertex
        //check line-line
        val (onLLA, onLLB, llD) = closestPointsBetweenLines(a0, a, b0, b) ?: return let {
            val p1 = Vector3d(a0).mul(0.5).add(Vector3d(a1).mul(0.5))
            val p2 = Vector3d(b0).mul(0.5).add(Vector3d(b1).mul(0.5))
            Triple(
                p1,
                p2,
                p1.distance(p2),
            )
        }

        if (onLLA.inside(axRange, ayRange, azRange) && onLLB.inside(bxRange, byRange, bzRange)) return Triple(
            onLLA,
            onLLB,
            llD
        )

        //test vertex-line:
        //a0-b, a1-b, b0-a, b1-a
        //return closest valid, because if it's valid then it must be closer than it is to a vertex, otherwise try vertex-vertex
        val vls = mutableListOf<Triple<Vector3d, Vector3d, Double>>()

        closestPointOnLine(b0, b, a0).let {
            if (it.first.inside(bxRange, byRange, bzRange)) vls += Triple(
                a0,
                it.first,
                it.second
            )
        }
        closestPointOnLine(b0, b, a1).let {
            if (it.first.inside(bxRange, byRange, bzRange)) vls += Triple(
                a1,
                it.first,
                it.second
            )
        }
        closestPointOnLine(a0, a, b0).let {
            if (it.first.inside(axRange, ayRange, azRange)) vls += Triple(
                it.first,
                b0,
                it.second
            )
        }
        closestPointOnLine(a0, a, b1).let {
            if (it.first.inside(axRange, ayRange, azRange)) vls += Triple(
                it.first,
                b1,
                it.second
            )
        }

        val vlMin = vls.minByOrNull { it.third }

        val vvs = listOf(
            Triple(a0, b0, a0.distance(b0)),
            Triple(a0, b1, a0.distance(b1)),
            Triple(a1, b0, a1.distance(b0)),
            Triple(a1, b1, a1.distance(b1)),
        )

        val vvsMin = vvs.minBy { it.third }

        if (vlMin == null) return vvsMin
        return if (vvsMin.third < vlMin.third) vvsMin else vlMin
    }

    private val an = Vector3d()
    private val bn = Vector3d()
    private val cn = Vector3d()
    private val d = Vector3d()
    private val d1 = Vector3d()
    private val a1 = Vector3d()
    private val c1 = Vector3d()
    private val b2 = Vector3d()
    private val r1 = Vector3d()
    private val cn1 = Vector3d()
    private val d2 = Vector3d()
    private val a2 = Vector3d()

    /**
     * Finds closest distance between two lines defined by a0, a and b0, b
     * @return closest point on line a, closest point on line b, distance, null if parallel
     */
    private fun closestPointsBetweenLines(
        a0: Vector3d,
        a: Vector3d,
        b0: Vector3d,
        b: Vector3d
    ): Triple<Vector3d, Vector3d, Double>? {
        an.set(a).normalize()
        bn.set(b).normalize()
        cn.set(an).cross(bn).normalize()
        if (!cn.isFinite) return null

        d.set(b0).sub(a0)

        val r = d1.set(d).sub(a1.set(a).mul(d.dot(a))).sub(c1.set(cn).mul(d.dot(cn)))
        if (r.length() < 0.0000001) return null
        val bClosest = Vector3d(b0).sub(b2.set(b).mul(r.length() / b.dot(r1.set(r).normalize())))
        val c = cn1.set(cn).mul(d2.set(d).sub(a2.set(a).mul(d.dot(a))).dot(cn))

        return Triple(Vector3d(bClosest).sub(c), bClosest, c.length())
    }

    private val xn = Vector3d()
    private val x01 = Vector3d()
    private val px0 = Vector3d()

    /**
     * Closest point on a line to another point
     * @return the closest point, the distance
     */
    private fun closestPointOnLine(x0: Vector3d, x: Vector3d, p: Vector3d): Pair<Vector3d, Double> {
        xn.set(x).normalize()
        val closestPoint = Vector3d(x0).add(x01.set(x).mul(px0.set(p).sub(x0).dot(xn)))

        return closestPoint to closestPoint.distance(p)
    }

    fun Vector3d.inside(
        xRange: ClosedRange<Double>,
        yRange: ClosedRange<Double>,
        zRange: ClosedRange<Double>,
    ): Boolean {
        return x in xRange && y in yRange && z in zRange
    }
}