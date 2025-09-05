package com.ixume.udar.collisiondetection.local

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Edge
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.pool.TrackingD3Pool
import com.ixume.udar.dynamicaabb.array.IntQueue
import com.ixume.udar.physics.contact.A2SContactArray
import com.ixume.udar.physics.contact.A2SContactCollection
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class LocalMathUtil(
    val world: PhysicsWorld,
) {
    private val d3Pool = TrackingD3Pool(15)

    val envContactUtil = LocalEnvContactUtil(this)
    val cuboidSATContactUtil = LocalCuboidSATContactUtil(this)
    val compositeUtil = LocalCompositeUtil()

    val envOverlapQueue = IntQueue()
    val envEdgeOverlapQueue = IntQueue()

    private val _mm = DoubleArray(2)
    
    private val _outCA = Vector3d()
    private val _outCB = Vector3d()

    fun collidePlane(
        first: ActiveBody,
        axis: LocalMesher.AxisD,
        level: Double,
        vertices: Array<Vector3d>,
        out: A2SContactArray,
    ): Boolean {
        axis.project(vertices, _mm)
        val min = _mm[0]
        val max = _mm[1]

        if (level < min || level > max) {
            return false
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

        if (level - min < max - level) {
            var i = 0
            while (i < vertices.size) {
                val v = vertices[i]
                val p = axis.project(v)

                if (p < level) {
                    out.addCollision(
                        activeBody = first,
                        pointAX = v.x,
                        pointAY = v.y,
                        pointAZ = v.z,
                        normX = axis.vec.x,
                        normY = axis.vec.y,
                        normZ = axis.vec.z,
                        depth = level - p,
                        contactID = 0L,
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
                    out.addCollision(
                        activeBody = first,
                        pointAX = v.x,
                        pointAY = v.y,
                        pointAZ = v.z,

                        normX = negatedAxis.x,
                        normY = negatedAxis.y,
                        normZ = negatedAxis.z,
                        depth = p - level,
                        contactID = 0L,
                    )
                }

                i++
            }
        }

        return true
    }

    private val _empty = Vector3d()

    /**
     * MUST NOT BE PARALLEL TO EDGE!! USE DIFFERENT METHOD IF THEY ARE!
     */
    fun collideCuboidEdge(
        activeBody: ActiveBody,

        edgeStart: Vector3d,
        edgeEnd: Vector3d,

        bodyAxiss: Array<Vector3d>,

        crossAxiss: Array<Vector3d>,

        vertices: Array<Vector3d>,
        edges: Array<Edge>,

        allowedNormals: Array<Vector3d>,
        out: A2SContactCollection,
    ) {
        var minBodyOverlap = Double.MAX_VALUE
        var minBodyAxis: Vector3d? = null
        var minBodyInDirOfAxis = true

        var minCrossOverlap = Double.MAX_VALUE
        var minCrossAxis: Vector3d? = null
        var minCrossInDirOfAxis = true

        var i = 0
        while (i < bodyAxiss.size) {
            val axis = bodyAxiss[i]

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
                return
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
                return
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
                return
            }
            if (norm.dot(allowedNormals[1]) < 0.0) {
                return
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
            while (l < edges.size) {
                val edge = edges[l]

                val distance = closestPointsBetweenSegments(
                    a0 = edge.start,
                    a1 = edge.end,
                    b0 = edgeStart,
                    b1 = edgeEnd,
                    outA = _outCA,
                    outB = _outCB,
                )

                if (distance < closestEdgeDistance) {
                    closestEdgeDistance = distance
                    closestA.set(_outCA)
                }

                l++
            }

            check(closestEdgeDistance != Double.MAX_VALUE)

            out.addCollision(
                activeBody = activeBody,

                pointAX = closestA.x,
                pointAY = closestA.y,
                pointAZ = closestA.z,

                normX = norm.x,
                normY = norm.y,
                normZ = norm.z,

                depth = minCrossOverlap,
                contactID = 0L
            )

            return
        } else {
            minBodyAxis!!

            val norm = if (minBodyInDirOfAxis) {
                Vector3d(minBodyAxis).normalize()
            } else {
                Vector3d(minBodyAxis).negate().normalize()
            }

            if (norm.dot(allowedNormals[0]) < 0.0) {
                return
            }
            if (norm.dot(allowedNormals[1]) < 0.0) {
                return
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
                if (minBodyInDirOfAxis) {
                    check(abs(norm.length() - 1.0) < 1e-5)
                    out.addCollision(
                        activeBody = activeBody,

                        pointAX = edgeEnd.x - minBodyAxis.x * minBodyOverlap,
                        pointAY = edgeEnd.y - minBodyAxis.y * minBodyOverlap,
                        pointAZ = edgeEnd.z - minBodyAxis.z * minBodyOverlap,

                        normX = norm.x,
                        normY = norm.y,
                        normZ = norm.z,

                        depth = minBodyOverlap,
                        contactID = 0L,
                    )

                    return
                } else {
                    check(abs(norm.length() - 1.0) < 1e-5)

                    out.addCollision(
                        activeBody = activeBody,

                        pointAX = edgeStart.x + minBodyAxis.x * minBodyOverlap,
                        pointAY = edgeStart.y + minBodyAxis.y * minBodyOverlap,
                        pointAZ = edgeStart.z + minBodyAxis.z * minBodyOverlap,

                        normX = norm.x,
                        normY = norm.y,
                        normZ = norm.z,

                        depth = minBodyOverlap,
                        contactID = 0L,
                    )
                }

                return
            } else {
                // edgeStart is max, edgeEnd is min
                if (minBodyInDirOfAxis) {
                    check(abs(norm.length() - 1.0) < 1e-5)

                    out.addCollision(
                        activeBody = activeBody,

                        pointAX = edgeStart.x - minBodyAxis.x * minBodyOverlap,
                        pointAY = edgeStart.y - minBodyAxis.y * minBodyOverlap,
                        pointAZ = edgeStart.z - minBodyAxis.z * minBodyOverlap,

                        normX = norm.x,
                        normY = norm.y,
                        normZ = norm.z,

                        depth = minBodyOverlap,
                        contactID = 0L,
                    )

                    return
                } else {
                    check(abs(norm.length() - 1.0) < 1e-5)

                    out.addCollision(
                        activeBody = activeBody,

                        pointAX = edgeEnd.x + minBodyAxis.x * minBodyOverlap,
                        pointAY = edgeEnd.y + minBodyAxis.y * minBodyOverlap,
                        pointAZ = edgeEnd.z + minBodyAxis.z * minBodyOverlap,

                        normX = norm.x,
                        normY = norm.y,
                        normZ = norm.z,

                        depth = minBodyOverlap,
                        contactID = 0L,
                    )

                    return
                }
            }
        }
    }

    inline fun checkOverlap(
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
    
    private val _outP0 = Vector3d()
    private val _outP1 = Vector3d()
    private val _outP2 = Vector3d()
    private val _outP3 = Vector3d()

    /**
     * Finds closest distance between two lines defined by a0, a and b0, b
     * @return closest point on segment a, closest point on segment b, distance, null if parallel
     */
    fun closestPointsBetweenSegments(
        a0: Vector3d,
        a1: Vector3d,
        b0: Vector3d,
        b1: Vector3d,
        outA: Vector3d,
        outB: Vector3d,
    ): Double {
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
        val llD = closestPointsBetweenLines(a0, _an, b0, _bn, _onLLA, _onLLB)
        
        if (llD == Double.MAX_VALUE) {
            val p1 = outA.set(a0).mul(0.5).add(_an.set(a1).mul(0.5))
            val p2 = outB.set(b0).mul(0.5).add(_bn.set(b1).mul(0.5))
            return p1.distance(p2)
        }

        if (_onLLA.inside(axRange, ayRange, azRange) && _onLLB.inside(bxRange, byRange, bzRange)) {
            outA.set(_onLLA)
            outB.set(_onLLB)
            return llD
        }
        
        var closestDistance = Double.MAX_VALUE

        //test vertex-line:
        //a0-b, a1-b, b0-a, b1-a
        //return closest valid, because if it's valid then it must be closer than it is to a vertex, otherwise try vertex-vertex
        val d0 = closestPointOnLine(b0, _bn, a0, _outP0)
        if (d0 < closestDistance && _outP0.inside(bxRange, byRange, bzRange)) {
            closestDistance = d0
            outA.set(_outP0)
            outB.set(a0)
        }

        val d1 = closestPointOnLine(b0, _bn, a1, _outP1)
        if (d1 < closestDistance && _outP1.inside(bxRange, byRange, bzRange)) {
            closestDistance = d1
            outA.set(_outP1)
            outB.set(a1)
        }

        val d2 = closestPointOnLine(a0, _an, b0, _outP2)
        if (d2 < closestDistance && _outP2.inside(axRange, ayRange, azRange)) {
            closestDistance = d2
            outA.set(_outP2)
            outB.set(b0)
        }

        val d3 = closestPointOnLine(a0, _an, b1, _outP3)
        if (d3 < closestDistance && _outP3.inside(axRange, ayRange, azRange)) {
            closestDistance = d3
            outA.set(_outP3)
            outB.set(b1)
        }
        
        val d00 = a0.distance(b0)
        if (d00 < closestDistance) {
            closestDistance = d00
            outA.set(a0)
            outB.set(b0)
        }

        val d01 = a0.distance(b1)
        if (d01 < closestDistance) {
            closestDistance = d01
            outA.set(a0)
            outB.set(b1)
        }

        val d02 = a1.distance(b0)
        if (d02 < closestDistance) {
            closestDistance = d02
            outA.set(a1)
            outB.set(b0)
        }

        val d03 = a1.distance(b1)
        if (d03 < closestDistance) {
            outA.set(a1)
            outB.set(b1)
        }
        
        check(closestDistance != Double.MAX_VALUE)
        
        return closestDistance
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
    ): Double {
        an.set(a).normalize()
        bn.set(b).normalize()
        cn.set(an).cross(bn).normalize()
        if (!cn.isFinite) return Double.MAX_VALUE

        d.set(b0).sub(a0)

        val r = d1.set(d).sub(a1.set(a).mul(d.dot(a))).sub(c1.set(cn).mul(d.dot(cn)))
        if (r.length() < 0.0000001) return Double.MAX_VALUE
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
    private fun closestPointOnLine(
        x0: Vector3d,
        x: Vector3d,
        p: Vector3d,
        outP: Vector3d,
    ): Double {
        xn.set(x).normalize()
        outP.set(x0).add(x01.set(x).mul(px0.set(p).sub(x0).dot(xn)))
        return outP.distance(p)
    }

    fun Vector3d.inside(
        xRange: ClosedRange<Double>,
        yRange: ClosedRange<Double>,
        zRange: ClosedRange<Double>,
    ): Boolean {
        return x in xRange && y in yRange && z in zRange
    }
}