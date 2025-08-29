package com.ixume.udar.collisiondetection

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.pool.TrackingD3Pool
import com.ixume.udar.dynamicaabb.array.IntQueue
import com.ixume.udar.physics.CollisionResult
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class LocalMathUtil(
    val world: PhysicsWorld,
) {
    private val d3Pool = TrackingD3Pool(15)

    private val _efa = Vector3d()
    private val _myOrderedAxis = Vector3d()
    private val _otherOrderedAxis = Vector3d()

    private val _antiNormal = Vector3d()
    private val _trueAxis = Vector3d()

    private val _myAxiss = Array(3) { Vector3d() }
    private val _tv = Vector3d()
    
    val envOverlapQueue = IntQueue()
    val envEdgeOverlapQueue = IntQueue()

    fun collidesSAT(
        activeBody: ActiveBody,
        otherVertices: Array<Vector3d>,
        otherAxiss: List<Vector3d>,
        otherEdges: List<Vector3d>,
        allowedNormals: List<Vector3d>? = null,
        findAll: Boolean = false,
        collideMyAxiss: Boolean = true,
    ): MutableList<CollisionResult>? {
        val q = activeBody.q

        if (collideMyAxiss) {
            _myAxiss[0].set(1.0, 0.0, 0.0).rotate(q).normalize()
            _myAxiss[1].set(0.0, 1.0, 0.0).rotate(q).normalize()
            _myAxiss[2].set(0.0, 0.0, 1.0).rotate(q).normalize()
        }

        if (collideMyAxiss) _edgeCrosses(otherEdges, _myAxiss)

        val myVertices = activeBody.vertices

        var minOrder: Boolean? = null
        var minOverlap = Double.MAX_VALUE
        var minAxis: Vector3d? = null

        if (collideMyAxiss) {
            var i = 0
            while (i < _myAxiss.size) {
                val axis = _myAxiss[i]
                if (!axis.isFinite) {
                    ++i
                    continue
                }

                val my = activeBody.project(axis)
                otherVertices.project(axis)

                val cr = cycleSAT(
                    myMin = my.x, myMax = my.y,
                    otherMin = _pmin, otherMax = _pmax,
                )

                if (!cr) {
                    d3Pool.clearTracked()
                    return null
                }

                if (_overlap < minOverlap) {
                    minAxis = axis
                    minOverlap = _overlap
                    minOrder = _order
                }

                ++i
            }

            var k = 0
            while (k < _edgeArr.size) {
                val axis = _edgeArr[k]
                if (!axis.isFinite) {
                    ++k
                    continue
                }

                val my = activeBody.project(axis)
                otherVertices.project(axis)

                val cr = cycleSAT(
                    myMin = my.x, myMax = my.y,
                    otherMin = _pmin, otherMax = _pmax,
                )

                if (!cr) {
                    d3Pool.clearTracked()
                    return null
                }

                if (_overlap < minOverlap) {
                    minAxis = axis
                    minOverlap = _overlap
                    minOrder = _order
                }

                ++k
            }
        }

        var j = 0
        while (j < otherAxiss.size) {
            val axis = otherAxiss[j]
            if (!axis.isFinite) {
                ++j
                continue
            }

            val my = activeBody.project(axis)
            otherVertices.project(axis)

            val cr = cycleSAT(
                myMin = my.x, myMax = my.y,
                otherMin = _pmin, otherMax = _pmax,
            )

            if (!cr) {
                d3Pool.clearTracked()
                return null
            }

            if (_overlap < minOverlap) {
                minAxis = axis
                minOverlap = _overlap
                minOrder = _order
            }

            ++j
        }

        minAxis!!
        minOrder!!

        if (allowedNormals != null) {
            _efa.set(minAxis)
            if (!minOrder) {
                _efa.negate()
            }

            if (!allowedNormals.all { _efa.dot(it) >= 0.0 }) {
                d3Pool.clearTracked()
                return null
            }
        }

        val epsilon = 1e-10

        if (collideMyAxiss && minAxis in _edgeArr) {
            //edge-edge
            //depending on the order, find most penetrating point(s) on each body, then choose the edges from that
//            if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                println("EDGE-EDGE")
//                println("  - AXIS: $minAxis")
//                println("  - OVERLAP: $minOverlap")
//                println("  - ORDER: $minOrder")
//            }

            val myDeepestVertices = mutableListOf<Vector3d>()
            var myDeepestDistance = -Double.MAX_VALUE

            _myOrderedAxis.set(minAxis)
            if (minOrder) _myOrderedAxis.negate()

            for (vertex in myVertices) {
                val d = vertex.dot(_myOrderedAxis)
//                println("my $vertex d: $d deepest: $myDeepestDistance")
                if (abs(d - myDeepestDistance) < epsilon) {
//                    println("MERGER!")
                    myDeepestVertices += vertex
                    continue
                } else if (d > myDeepestDistance) {
//                    println("NEW LARGEST!")
                    myDeepestDistance = d
                    myDeepestVertices.clear()
                    myDeepestVertices += vertex
                }
            }

            val otherDeepestVertices = mutableListOf<Vector3d>()
            var otherDeepestDistance = -Double.MAX_VALUE
            _otherOrderedAxis.set(_myOrderedAxis).negate()

            for (vertex in otherVertices) {
                val d = vertex.dot(_otherOrderedAxis)
//                println("other $vertex d: $d deepest: $otherDeepestDistance")
                if (abs(d - otherDeepestDistance) < epsilon) {
                    otherDeepestVertices += vertex
                    continue
                } else if (d > otherDeepestDistance) {
                    otherDeepestDistance = d
                    otherDeepestVertices.clear()
                    otherDeepestVertices += vertex
                }
            }

//            println("myDeepestVertices : $myDeepestDistance : $myDeepestVertices")
//            println("otherDeepestVertices : $otherDeepestDistance : $otherDeepestVertices")

            if (myDeepestVertices.size % 2 != 0) return null
            if (otherDeepestVertices.size % 2 != 0) return null

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

            d3Pool.clearTracked()
            return mutableListOf(
                CollisionResult(
                    Vector3d(r.first),
                    Vector3d(r.second),
                    Vector3d(if (minOrder) minAxis else minAxis.negate()),
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

            if (collideMyAxiss && _myAxiss.contains(minAxis)) {
                _antiNormal.set(minAxis)
                if (!minOrder) {
                    _antiNormal.negate()
                }
//                println("OTHER AXIS")
                //other has incident
                for (vertex in otherVertices) {
                    val d = vertex.dot(_antiNormal)
                    if (d > furthestDistance) {
                        furthestDistance = d
                        furtherVertex = vertex
                    }
                }
//                if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                    println("  - POINT: $furtherVertex")
//                }

                //happens to conicide with antinormal here, but keep them as separate variables to not cause confusion
                _trueAxis.set(minAxis)
                if (!minOrder) {
                    _trueAxis.negate()
                }

                val r = mutableListOf(
                    CollisionResult(
                        furtherVertex!!,
                        Vector3d(furtherVertex).add(Vector3d(_trueAxis).mul(minOverlap)),
                        Vector3d(_trueAxis),
                        minOverlap,
                    )
                )

                if (findAll) {
                    for (vertex in otherVertices) {
                        val d = vertex.dot(_antiNormal)
                        if (abs(d - furthestDistance) <= minOverlap) {
                            r += CollisionResult(
                                vertex,
                                Vector3d(vertex).add(Vector3d(_trueAxis).mul(minOverlap - abs(d - furthestDistance))),
                                Vector3d(_trueAxis),
                                minOverlap - abs(d - furthestDistance),
                            )
                        }
                    }
                }

                d3Pool.clearTracked()
                return r
            } else {
                _antiNormal.set(minAxis)
                if (minOrder) {
                    _antiNormal.negate()
                }
//                println("MY AXIS")
                //i have incident
                for (vertex in myVertices) {
                    val d = vertex.dot(_antiNormal)
                    if (d > furthestDistance) {
                        furthestDistance = d
                        furtherVertex = vertex
                    }
                }

//                if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) {
//                    println("  - POINT: $furtherVertex")
//                }

                _trueAxis.set(minAxis)
                if (!minOrder) {
                    _trueAxis.negate()
                }

                val r = mutableListOf(
                    CollisionResult(
                        furtherVertex!!,
                        Vector3d(furtherVertex).add(Vector3d(_trueAxis).mul(minOverlap)),
                        Vector3d(_trueAxis),
                        minOverlap,
                    )
                )

                if (findAll) {
                    for (vertex in myVertices) {
                        val d = vertex.dot(_antiNormal)
                        if (abs(d - furthestDistance) <= minOverlap) {
                            r += CollisionResult(
                                vertex,
                                Vector3d(vertex).add(Vector3d(_trueAxis).mul(minOverlap - abs(d - furthestDistance))),
                                Vector3d(_trueAxis),
                                minOverlap - abs(d - furthestDistance),
                            )
                        }
                    }
                }

                d3Pool.clearTracked()
                return r
            }
        }
    }

    fun collidePlane(
        axis: LocalMesher.AxisD,
        level: Double,
        vertices: Array<Vector3d>,
    ): List<CollisionResult>? {
        val (min, max) = axis.project(vertices)

        if (level < min || level > max) {
            return null
        }

        /*
        now find the overlap and from which direction it's from.
        min, max, projected look something like this:
        |
        |
        |
        |
        |  .
        |

        find all penetrating points
         */

        val collisions = mutableListOf<CollisionResult>()

        if (level - min < max - level) {
            //approached from top, so find vertex with smallest projection
            var i = 0
            while (i < vertices.size) {
                val v = vertices[i]
                val p = axis.project(v)

                if (p < level) {
                    collisions += CollisionResult(
                        pointA = v,
                        pointB = _empty,
                        norm = axis.vec,
                        depth = level - p,
                    )
                }

                i++
            }
        } else {
            val negatedAxis = Vector3d(axis.vec).normalize()
            //approached from bottom
            var i = 0
            while (i < vertices.size) {
                val v = vertices[i]
                val p = axis.project(v)

                if (p > level) {
                    collisions += CollisionResult(
                        pointA = v,
                        pointB = _empty,
                        norm = negatedAxis,
                        depth = p - level,
                    )
                }

                i++
            }
        }

        return collisions
    }

    private val _empty = Vector3d()

    /**
     * MUST NOT BE PARALLEL TO EDGE!! USE DIFFERENT METHOD IF THEY ARE!
     */
    fun collideCuboidEdge(
        edgeStart: Vector3d,
        edgeEnd: Vector3d,

        bodyAxiss: Array<Vector3d>,

        crossAxiss: Array<Vector3d>,

        vertices: Array<Vector3d>,

        allowedNormals: Array<Vector3d>,
    ): CollisionResult? {
//        println("collideCuboidEdge")
        var minBodyOverlap = Double.MAX_VALUE
        var minBodyAxis: Vector3d? = null
        var minBodyInDirOfAxis = true

        var minCrossOverlap = Double.MAX_VALUE
        var minCrossAxis: Vector3d? = null
        var minCrossInDirOfAxis = true

        var i = 0
        while (i < bodyAxiss.size) {
            val axis = bodyAxiss[i]

//            println("TESTING BODY AXIS: $axis")

            var bodyMin = Double.MAX_VALUE
            var bodyMax = -Double.MAX_VALUE

            var j = 0
            while (j < vertices.size) {
                val v = vertices[j]
                val d = v.dot(axis)

                bodyMin = min(d, bodyMin)
                bodyMax = max(d, bodyMax)

                j++
            }

            val es1 = edgeStart.dot(axis)
            val es2 = edgeEnd.dot(axis)

            val edgeMin = min(es1, es2)
            val edgeMax = max(es1, es2)

            if (!checkOverlap(bodyMin, bodyMax, edgeMin, edgeMax)) {
                //if no overlap, we found a non-overlapping axis between the edge and the cuboid, meaning they don't collide
//                println(
//                    """
//                    NOT COLLIDING ON BOXY $axis
//                    | edgeStart: $edgeStart
//                    | edgeEnd: $edgeEnd
//                    | edgeMin: $edgeMin
//                    | edgeMax: $edgeMax
//                    | bodyMin: $bodyMin
//                    | bodyMax: $bodyMax
//                """.trimIndent()
//                )
                return null
            }

            /*
            we have 2 lines A and B that overlap in some way, let's consider the cases:

            1:   *     2:   *     3:     *
                 |          |            |
                 |          |  *      *  |
                 |  *       |  |      |  |
                 *  |       |  |      |  |
                    |       |  *      *  |
                    |       |            |
                    *       *            *

                 A  B       A  B      A  B

            case 1 is a proper collision; cases 2 and 3 are either due to extremely high velocity or numerical error; for let's not try any special handling
             */

            if ((edgeMin < bodyMin && edgeMax > bodyMax) || (bodyMin < edgeMin && bodyMax > edgeMax)) {
//                println("CONTAINMENT OF BODY AXIS!")
                i++
                continue
            }

            val overlapLow =
                edgeMax - bodyMin // if this is smaller, then edge must be under body, thus we should use positive axis as normal
            val overlapHigh =
                bodyMax - edgeMin // if this is smaller, then edge must be over body, thus we should use negative axis as normal

            if (overlapLow < overlapHigh) {
                if (overlapLow < minBodyOverlap) {
                    minBodyOverlap = overlapLow
                    minBodyAxis = axis
                    minBodyInDirOfAxis = true
                }
            } else {
                if (overlapHigh < minBodyOverlap) {
                    minBodyOverlap = overlapHigh
                    minBodyAxis = axis
                    minBodyInDirOfAxis = false
                }
            }

            i++
        }

        var j = 0
        while (j < crossAxiss.size) {
            val axis = crossAxiss[j]

            if (axis.length() < 1e-5) { // edge axis aligned with body axis; this can't be the minimum collision vector so just ignore it
                j++
                continue
            }

            check(abs(axis.length() - 1.0) < 1e-5) {
                """
                axis: $axis
                length: ${axis.length()}
            """.trimIndent()
            }

//            println("TESTING CROSS AXIS: $axis")
//
            var crossMin = Double.MAX_VALUE
            var crossMax = -Double.MAX_VALUE

            var k = 0
            while (k < vertices.size) {
                val v = vertices[k]
                val d = v.dot(axis)

                crossMin = min(d, crossMin)
                crossMax = max(d, crossMax)

                k++
            }

            val es =
                edgeStart.dot(axis) // since this is a cross between the edge normal and the body, min and max are equal

            if (es > crossMax || es < crossMin) {
//                println(
//                    """
//                    NOT COLLIDING ON BOXY $axis
//                    | edgeStart: $edgeStart
//                    | edgeEnd: $edgeEnd
//                    | es: $es
//                    | crossMin: $crossMin
//                    | crossMax: $crossMax
//                """.trimIndent()
//                )
                return null
            }

            /*
            since es is just a single number, then overlap is simply side closest to es. if closer from minimum, then body must be above and we should use positive axis for normal, otherwise, use negative axis
             */

            val overlapLow = es - crossMin
            val overlapHigh = crossMax - es

            if (overlapLow < overlapHigh) {
                if (overlapLow < minCrossOverlap) {
                    minCrossOverlap = overlapLow
                    minCrossAxis = axis
                    minCrossInDirOfAxis = true
                }
            } else {
                if (overlapHigh < minCrossOverlap) {
                    minCrossOverlap = overlapHigh
                    minCrossAxis = axis
                    minCrossInDirOfAxis = false
                }
            }

            j++
        }

//        check(abs(minCrossAxis!!.length() - 1.0) < 1e-5)

        /*
        check if the min axis is even allowed; if not, then we're on the wrong side of the edge

        check this for both edge-edge and face-vertex since we don't want edges acting as points from behind, even if in theory this should never happen
         */

        if (minCrossOverlap < minBodyOverlap) {
            minCrossAxis!!

            val norm = if (minCrossInDirOfAxis) {
                Vector3d(minCrossAxis).normalize()
            } else {
                Vector3d(minCrossAxis).negate().normalize()
            }

            check(abs(norm.length() - 1.0) < 1e-5)

            if (norm.dot(allowedNormals[0]) < 0.0) {
//                println(
//                    """
//                    DISALLOWED NORMAL
//                    | norm: $norm
//                    | failed allowed: ${allowedNormals[0]}
//                    | edgeStart: $edgeStart
//                    | edgeEnd: $edgeEnd
//                """.trimIndent()
//                )
                return null
            }
            if (norm.dot(allowedNormals[1]) < 0.0) {
//                println(
//                    """
//                    DISALLOWED NORMAL
//                    | norm: $norm
//                    | failed allowed: ${allowedNormals[1]}
//                    | edgeStart: $edgeStart
//                    | edgeEnd: $edgeEnd
//                """.trimIndent()
//                )
                return null
            }

            /*
            collision is edge-edge

            since this is along an axis which is perpendicular to both edges, we just find the closest points on both edges; use overlap as depth still;

            edge start and end are given,
            just go through all the edges on cuboid and choose the closest
             */

            var closestEdgeDistance = Double.MAX_VALUE
            val closestA = Vector3d()

            var l = 0
            while (l < vertices.size) {
                val v1 = vertices[l]
                var next = l + 1

                if (next >= vertices.size) next = 0

                val v2 = vertices[next]

                val (cA, _, distance) = closestPointsBetweenSegments(
                    a0 = v1,
                    a1 = v2,
                    b0 = edgeStart,
                    b1 = edgeEnd
                )

                if (distance < closestEdgeDistance) {
                    closestEdgeDistance = distance
                    closestA.set(cA)
                }

                l++
            }

            check(closestEdgeDistance != Double.MAX_VALUE)

            return CollisionResult(
                pointA = closestA,
                pointB = _empty,
                norm = norm,
                depth = minCrossOverlap,
            )
        } else {
            minBodyAxis!!

            val norm = if (minBodyInDirOfAxis) {
                Vector3d(minBodyAxis).normalize()
            } else {
                Vector3d(minBodyAxis).negate().normalize()
            }

            if (norm.dot(allowedNormals[0]) < 0.0) {
//                println(
//                    """
//                    DISALLOWED NORMAL
//                    | norm: $norm
//                    | failed allowed: ${allowedNormals[0]}
//                    | edgeStart: $edgeStart
//                    | edgeEnd: $edgeEnd
//                """.trimIndent()
//                )
                return null
            }
            if (norm.dot(allowedNormals[1]) < 0.0) {
//                println(
//                    """
//                    DISALLOWED NORMAL
//                    | norm: $norm
//                    | failed allowed: ${allowedNormals[1]}
//                    | edgeStart: $edgeStart
//                    | edgeEnd: $edgeEnd
//                """.trimIndent()
//                )
                return null
            }


            /*
            collision is face-vertex, except here the vertex is one of the edge points; this means point of deepest collision is simply the part of the edge closer to the cuboid

            so just project the edge again onto the axis, then depending on the order, choose one

            figure out which edge vertex is the min and max
             */

            val es1 = edgeStart.dot(minBodyAxis)
            val es2 = edgeEnd.dot(minBodyAxis)

            if (es1 < es2) {
                // edgeStart is min, edgeEnd is max
                return if (minBodyInDirOfAxis) {
                    check(abs(norm.length() - 1.0) < 1e-5)
                    CollisionResult(
                        pointA = Vector3d(edgeEnd).sub(
                            minBodyAxis.x * minBodyOverlap,
                            minBodyAxis.y * minBodyOverlap,
                            minBodyAxis.z * minBodyOverlap
                        ),
                        pointB = _empty, // since this is a collision with a static body, pointB doesn't even matter ( see LocalConstraintSolver )
                        norm = norm,
                        depth = minBodyOverlap,
                    )
                } else {
                    check(abs(norm.length() - 1.0) < 1e-5)
                    CollisionResult(
                        pointA = Vector3d(edgeStart).add(
                            minBodyAxis.x * minBodyOverlap,
                            minBodyAxis.y * minBodyOverlap,
                            minBodyAxis.z * minBodyOverlap
                        ),
                        pointB = _empty,
                        norm = norm,
                        depth = minBodyOverlap,
                    )
                }
            } else {
                // edgeStart is max, edgeEnd is min
                return if (minBodyInDirOfAxis) {
                    check(abs(norm.length() - 1.0) < 1e-5)
                    CollisionResult(
                        pointA = Vector3d(edgeStart).sub(
                            minBodyAxis.x * minBodyOverlap,
                            minBodyAxis.y * minBodyOverlap,
                            minBodyAxis.z * minBodyOverlap
                        ),
                        pointB = _empty,
                        norm = norm,
                        depth = minBodyOverlap,
                    )
                } else {
                    check(abs(norm.length() - 1.0) < 1e-5)
                    CollisionResult(
                        pointA = Vector3d(edgeEnd).add(
                            minBodyAxis.x * minBodyOverlap,
                            minBodyAxis.y * minBodyOverlap,
                            minBodyAxis.z * minBodyOverlap
                        ),
                        pointB = _empty,
                        norm = norm,
                        depth = minBodyOverlap,
                    )
                }
            }
        }
    }

    private inline fun checkOverlap(
        a: Double,
        b: Double,

        x: Double,
        y: Double,
    ): Boolean {
        return a < y && b > x
    }

    private var _pmin = 0.0
    private var _pmax = 0.0

    fun Array<Vector3d>.project(axis: Vector3d) {
        _pmin = Double.MAX_VALUE
        _pmax = -Double.MAX_VALUE

        var i = 0
        while (i < size) {
            val v = this[i]
            val s = v.dot(axis)
            _pmin = min(_pmin, s)
            _pmax = max(_pmax, s)

            i++
        }
    }


    private var _overlap = 0.0
    private var _order = false

    fun cycleSAT(
        myMin: Double,
        myMax: Double,
        otherMin: Double,
        otherMax: Double,
    ): Boolean {
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
            return false
        }

        _overlap = overlap
        _order = order!!
        return true
    }

    private val _edgeArr = ArrayList<Vector3d>()

    private fun _edgeCrosses(
        edgesA: Collection<Vector3d>,
        edgesB: Array<Vector3d>,
    ) {
        if (edgesA.isEmpty() || edgesB.isEmpty()) return
        _edgeArr.clear()
        for (edgeA in edgesA) {
            for (edgeB in edgesB) {
                _edgeArr += d3Pool.get().set(edgeA).cross(edgeB).normalize()
            }
        }
    }

    private val _onLLA = Vector3d()
    private val _onLLB = Vector3d()
    private val _an = Vector3d()
    private val _bn = Vector3d()

    /**
     * Finds closest distance between two lines defined by a0, a and b0, b
     * @return closest point on segment a, closest point on segment b, distance, null if parallel
     */
    fun closestPointsBetweenSegments(
        a0: Vector3d,
        a1: Vector3d,
        b0: Vector3d,
        b1: Vector3d,
    ): Triple<Vector3d, Vector3d, Double> {
        val epsilon = 1e-10

        _an.set(a1).sub(a0).normalize()
        _bn.set(b1).sub(b0).normalize()

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
        val llD = closestPointsBetweenLines(a0, _an, b0, _bn, _onLLA, _onLLB) ?: return let {
            val p1 = Vector3d(a0).mul(0.5).add(_an.set(a1).mul(0.5))
            val p2 = Vector3d(b0).mul(0.5).add(_bn.set(b1).mul(0.5))
            Triple(
                p1,
                p2,
                p1.distance(p2),
            )
        }

        if (_onLLA.inside(axRange, ayRange, azRange) && _onLLB.inside(bxRange, byRange, bzRange)) return Triple(
            Vector3d(_onLLA),
            Vector3d(_onLLB),
            llD
        )

        //test vertex-line:
        //a0-b, a1-b, b0-a, b1-a
        //return closest valid, because if it's valid then it must be closer than it is to a vertex, otherwise try vertex-vertex
        val vls = mutableListOf<Triple<Vector3d, Vector3d, Double>>()

        closestPointOnLine(b0, _bn, a0).let {
            if (it.first.inside(bxRange, byRange, bzRange)) vls += Triple(
                a0,
                it.first,
                it.second
            )
        }
        closestPointOnLine(b0, _bn, a1).let {
            if (it.first.inside(bxRange, byRange, bzRange)) vls += Triple(
                a1,
                it.first,
                it.second
            )
        }
        closestPointOnLine(a0, _an, b0).let {
            if (it.first.inside(axRange, ayRange, azRange)) vls += Triple(
                it.first,
                b0,
                it.second
            )
        }
        closestPointOnLine(a0, _an, b1).let {
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
        b: Vector3d,
        onLLA: Vector3d,
        onLLB: Vector3d,
    ): Double? {
        an.set(a).normalize()
        bn.set(b).normalize()
        cn.set(an).cross(bn).normalize()
        if (!cn.isFinite) return null

        d.set(b0).sub(a0)

        val r = d1.set(d).sub(a1.set(a).mul(d.dot(a))).sub(c1.set(cn).mul(d.dot(cn)))
        if (r.length() < 0.0000001) return null
        val bClosest = onLLB.set(b0).sub(b2.set(b).mul(r.length() / b.dot(r1.set(r).normalize())))
        val c = cn1.set(cn).mul(d2.set(d).sub(a2.set(a).mul(d.dot(a))).dot(cn))

        onLLA.set(bClosest).sub(c)

        return c.length()
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