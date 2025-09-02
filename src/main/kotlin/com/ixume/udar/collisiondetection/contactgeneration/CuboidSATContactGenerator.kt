package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Edge
import com.ixume.udar.body.active.Face
import com.ixume.udar.collisiondetection.LocalMathUtil
import com.ixume.udar.collisiondetection.capability.Projectable
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class CuboidSATContactGenerator(
    val activeBody: ActiveBody,
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if (other.isConvex && other is Projectable) 0 else -1
    }

    private val _myBodyAxiss = Array(3) { Vector3d() }
    private val _otherBodyAxiss = Array(3) { Vector3d() }
    private val _crossAxiss = Array(9) { Vector3d() }

    private val contacts = mutableListOf<Contact>()

    private val _incidentVertices = DoubleArrayList()
    private val _prevIncidentVertex = Vector3d()
    private val _currIncidentVertex = Vector3d()

    private val _myEdgeVertices = DoubleArrayList()
    private val _otherEdgeVertices = DoubleArrayList()

    private val _refCenter = Vector3d()
    private val _incidentPoint = Vector3d()
    private val _intersectionOut = Vector3d()
    private val _tp = Vector3d()
    private val _tpProjected = Vector3d()

    private val _edgeNormal = Vector3d()

    override fun collides(other: Body, math: LocalMathUtil): List<Contact> {
        require(other is ActiveBody)

        contacts.clear()

        if (!setupAxiss(other)) return contacts

        val myVertices = activeBody.vertices
        if (myVertices.isEmpty()) return contacts
        val otherVertices = other.vertices
        if (otherVertices.isEmpty()) return contacts

        var minMyBodyOverlap = Double.MAX_VALUE
        var minMyBodyAxis: Vector3d? = null
        var minMyBodyInDirOfAxis = true

        var minOtherBodyOverlap = Double.MAX_VALUE
        var minOtherBodyAxis: Vector3d? = null
        var minOtherBodyInDirOfAxis = true

        var minCrossOverlap = Double.MAX_VALUE
        var minCrossAxis: Vector3d? = null
        var minCrossAxisIdx: Int = -1
        var minCrossInDirOfAxis = true

        var i = 0
        while (i < _myBodyAxiss.size) {
            val axis = _myBodyAxiss[i]

            var myMin = Double.MAX_VALUE
            var myMax = -Double.MAX_VALUE

            for (v in myVertices) {
                val s = v.dot(axis)
                myMin = min(myMin, s)
                myMax = max(myMax, s)
            }

            var otherMin = Double.MAX_VALUE
            var otherMax = -Double.MAX_VALUE

            for (v in otherVertices) {
                val s = v.dot(axis)
                otherMin = min(otherMin, s)
                otherMax = max(otherMax, s)
            }

            /*
            possibilities ere are containment, overlap, or nothing
                containment cannot possibly be the best collision, so we can skip it
             */

            if (!math.checkOverlap(myMin, myMax, otherMin, otherMax)) {
                return contacts
            }

            if ((myMin < otherMin && myMax > otherMax) || (otherMin < myMin && otherMax > myMin)) {
                i++
                continue
            }

            val overlapLow = otherMax - myMin
            val overlapHigh = myMax - otherMin

            if (overlapLow < overlapHigh) {
                if (overlapLow < minMyBodyOverlap) {
                    minMyBodyOverlap = overlapLow
                    minMyBodyAxis = axis
                    minMyBodyInDirOfAxis = true
                }
            } else {
                if (overlapHigh < minMyBodyOverlap) {
                    minMyBodyOverlap = overlapHigh
                    minMyBodyAxis = axis
                    minMyBodyInDirOfAxis = false
                }
            }

            i++
        }

        var j = 0
        while (j < _otherBodyAxiss.size) {
            val axis = _otherBodyAxiss[j]

            var myMin = Double.MAX_VALUE
            var myMax = -Double.MAX_VALUE

            for (v in myVertices) {
                val s = v.dot(axis)
                myMin = min(myMin, s)
                myMax = max(myMax, s)
            }

            var otherMin = Double.MAX_VALUE
            var otherMax = -Double.MAX_VALUE

            for (v in otherVertices) {
                val s = v.dot(axis)
                otherMin = min(otherMin, s)
                otherMax = max(otherMax, s)
            }

            /*
            possibilities ere are containment, overlap, or nothing
                containment cannot possibly be the best collision, so we can skip it
             */

            if (!math.checkOverlap(myMin, myMax, otherMin, otherMax)) {
                return contacts
            }

            if ((myMin < otherMin && myMax > otherMax) || (otherMin < myMin && otherMax > myMin)) {
                j++
                continue
            }

            val overlapLow = otherMax - myMin
            val overlapHigh = myMax - otherMin

            if (overlapLow < overlapHigh) {
                if (overlapLow < minOtherBodyOverlap) {
                    minOtherBodyOverlap = overlapLow
                    minOtherBodyAxis = axis
                    minOtherBodyInDirOfAxis = true
                }
            } else {
                if (overlapHigh < minOtherBodyOverlap) {
                    minOtherBodyOverlap = overlapHigh
                    minOtherBodyAxis = axis
                    minOtherBodyInDirOfAxis = false
                }
            }

            j++
        }

        var k = 0
        while (k < _crossAxiss.size) {
            val axis = _crossAxiss[k]

            var myMin = Double.MAX_VALUE
            var myMax = -Double.MAX_VALUE

            for (v in myVertices) {
                val s = v.dot(axis)
                myMin = min(myMin, s)
                myMax = max(myMax, s)
            }

            var otherMin = Double.MAX_VALUE
            var otherMax = -Double.MAX_VALUE

            for (v in otherVertices) {
                val s = v.dot(axis)
                otherMin = min(otherMin, s)
                otherMax = max(otherMax, s)
            }

            /*
            possibilities ere are containment, overlap, or nothing
                containment cannot possibly be the best collision, so we can skip it
             */

            if (!math.checkOverlap(myMin, myMax, otherMin, otherMax)) {
                return contacts
            }

            if ((myMin < otherMin && myMax > otherMax) || (otherMin < myMin && otherMax > myMin)) {
                k++
                continue
            }

            val overlapLow = otherMax - myMin
            val overlapHigh = myMax - otherMin

            minCrossAxisIdx = k
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

            k++
        }

        if (minCrossOverlap < minMyBodyOverlap && minCrossOverlap < minOtherBodyOverlap) {
            // edge-edge!
            if (minCrossAxis == null) return emptyList()
            
            val norm = if (minCrossInDirOfAxis) {
                Vector3d(minCrossAxis).normalize()
            } else {
                Vector3d(minCrossAxis).negate().normalize()
            }

            check(abs(norm.length() - 1.0) < 1e-5)

            val myEdges = activeBody.edges
            val otherEdges = other.edges

            _otherEdgeVertices.clear()
            _myEdgeVertices.clear()

            var myMin = Double.MAX_VALUE
            var myEdge: Edge? = null

            var m = 0
            while (m < myEdges.size) {
                val edge = myEdges[m]

                val a = edge.start.dot(norm)
                val b = edge.end.dot(norm)

                if (abs(a - b) < EDGE_EPSILON && min(a, b) < myMin) {
                    myMin = min(a, b)
                    myEdge = edge
                }

                m++
            }

            checkNotNull(myEdge)

            var otherMax = -Double.MAX_VALUE
            var otherEdge: Edge? = null

            var n = 0
            while (n < otherEdges.size) {
                val edge = otherEdges[n]

                val a = edge.start.dot(norm)
                val b = edge.end.dot(norm)

                if (abs(a - b) < EDGE_EPSILON && max(a, b) > otherMax) {
                    otherMax = max(a, b)
                    otherEdge = edge
                }

                n++
            }

            checkNotNull(otherEdge)
            
            val (cA, cB, _) = math.closestPointsBetweenSegments(
                a0 = myEdge.start,
                a1 = myEdge.end,
                b0 = otherEdge.start,
                b1 = otherEdge.end,
            )

            return listOf(
                Contact(
                    first = activeBody,
                    second = other,
                    result = CollisionResult(
                        pointA = cA,
                        pointB = cB,
                        norm = norm,
                        depth = minCrossOverlap,
                    ),
                )
            )
        } else {
            /*
            face-face:
            - find reference and incident face and vertices
                reference is just the face which created the axis
            - clip
            - filter and return
             */

            val axis = if (minMyBodyOverlap < minOtherBodyOverlap) minMyBodyAxis else minOtherBodyAxis
            if (axis == null) return emptyList()

            val inDirOfAxis =
                if (minMyBodyOverlap < minOtherBodyOverlap) minMyBodyInDirOfAxis else minOtherBodyInDirOfAxis

            val norm = if (inDirOfAxis) {
                Vector3d(axis).normalize()
            } else {
                Vector3d(axis).negate().normalize()
            }

            val myFaces = activeBody.faces
            val otherFaces = other.faces

            var refMax = -Double.MAX_VALUE
            var refFace: Face? = null

            var q = 0
            while (q < myFaces.size) {
                val face = myFaces[q]
                val p = face.normal.dot(norm)

                if (p > refMax) {
                    refMax = p
                    refFace = face
                }

                q++
            }

            var incidentMin = Double.MAX_VALUE
            var incidentFace: Face? = null

            var l = 0
            while (l < otherFaces.size) {
                val face = otherFaces[l]
                val p = face.normal.dot(norm)

                if (p < incidentMin) {
                    incidentMin = p
                    incidentFace = face
                }

                l++
            }

            checkNotNull(refFace)
            checkNotNull(incidentFace)

            refFace.center(_refCenter)

            // clip incident vertices into ref face by projecting onto edge plane if outside

            val output = DoubleArrayList()
            incidentFace.populate(output)
            
            var o = 0
            while (o < refFace.vertices.size) {
                val v1 = refFace.vertices[o]
                val v2 = refFace.vertices[(o + 1).mod(refFace.vertices.size)]

                val edgeNormal = _edgeNormal.set(v2).sub(v1).cross(norm).normalize()
                
                check(edgeNormal.dot(Vector3d(_refCenter).sub(v1)) < 0)

                _incidentVertices.clear()
                _incidentVertices.addAll(output)
                output.clear()

                var p = 0
                while (p < _incidentVertices.size / 3) {
                    _prevIncidentVertex.set(
                        _incidentVertices.getDouble((p * 3 - 3).mod(_incidentVertices.size)),
                        _incidentVertices.getDouble((p * 3 - 3 + 1).mod(_incidentVertices.size)),
                        _incidentVertices.getDouble((p * 3 - 3 + 2).mod(_incidentVertices.size)),
                    )
                    
                    _currIncidentVertex.set(
                        _incidentVertices.getDouble(p * 3),
                        _incidentVertices.getDouble(p * 3 + 1),
                        _incidentVertices.getDouble(p * 3 + 2),
                    )

                    intersection(
                        planePoint = v1,
                        planeNormal = edgeNormal,
                        start = _prevIncidentVertex,
                        end = _currIncidentVertex,
                        out = _intersectionOut,
                    )
                    
                    val prevInside = _incidentPoint.set(_prevIncidentVertex).sub(v1).dot(edgeNormal) <= 0
                    val currInside = _incidentPoint.set(_currIncidentVertex).sub(v1).dot(edgeNormal) <= 0
                    
                    if (currInside) {
                        if (!prevInside) {
                            output.add(_intersectionOut.x)
                            output.add(_intersectionOut.y)
                            output.add(_intersectionOut.z)
                        }

                        output.add(_currIncidentVertex.x)
                        output.add(_currIncidentVertex.y)
                        output.add(_currIncidentVertex.z)
                    } else if (prevInside) {
                        output.add(_intersectionOut.x)
                        output.add(_intersectionOut.y)
                        output.add(_intersectionOut.z)
                    }

                    p++
                }

                o++
            }
            
            // now check if the incident vertices are behind the reference face

            val collisions = mutableListOf<Contact>()

            var p = 0
            while (p < output.size / 3) {
                _tp.set(
                    output.getDouble(p * 3),
                    output.getDouble(p * 3 + 1),
                    output.getDouble(p * 3 + 2),
                )

                val d = -_tpProjected.set(_tp).sub(refFace.point()).dot(refFace.normal)

                if (d < 0.0) {
                    val v = Vector3d(_tp).sub(d * refFace.normal.x, d * refFace.normal.y, d * refFace.normal.z)
                    collisions += Contact(
                        first = activeBody,
                        second = other,
                        result = CollisionResult(
                            pointA = v,
                            pointB = Vector3d(_tp),
                            norm = norm,
                            depth = -d,
                        )
                    )
                }

                p++
            }

            return collisions
        }
    }

    private val _lineDir = Vector3d()
    private val _w = Vector3d()

    private fun intersection(
        planePoint: Vector3d,
        planeNormal: Vector3d,
        start: Vector3d,
        end: Vector3d,
        out: Vector3d,
    ) {
        end.sub(start, _lineDir)

        val denominator = _lineDir.dot(planeNormal)

        start.sub(planePoint, _w)
        val numerator = -_w.dot(planeNormal)

        val t = numerator / denominator

        start.fma(t, _lineDir, out)
    }


    private fun setupAxiss(other: Body): Boolean {
        _myBodyAxiss[0].set(1.0, 0.0, 0.0).rotate(activeBody.q).normalize()
        _myBodyAxiss[1].set(0.0, 1.0, 0.0).rotate(activeBody.q).normalize()
        _myBodyAxiss[2].set(0.0, 0.0, 1.0).rotate(activeBody.q).normalize()

        _otherBodyAxiss[0].set(1.0, 0.0, 0.0).rotate(other.q).normalize()
        _otherBodyAxiss[1].set(0.0, 1.0, 0.0).rotate(other.q).normalize()
        _otherBodyAxiss[2].set(0.0, 0.0, 1.0).rotate(other.q).normalize()

        _crossAxiss[0].set(_myBodyAxiss[0]).cross(_otherBodyAxiss[0]).normalize()
        if (!_crossAxiss[0].isFinite) return false
        _crossAxiss[1].set(_myBodyAxiss[0]).cross(_otherBodyAxiss[1]).normalize()
        if (!_crossAxiss[1].isFinite) return false
        _crossAxiss[2].set(_myBodyAxiss[0]).cross(_otherBodyAxiss[2]).normalize()
        if (!_crossAxiss[2].isFinite) return false
        _crossAxiss[3].set(_myBodyAxiss[1]).cross(_otherBodyAxiss[0]).normalize()
        if (!_crossAxiss[3].isFinite) return false
        _crossAxiss[4].set(_myBodyAxiss[1]).cross(_otherBodyAxiss[1]).normalize()
        if (!_crossAxiss[4].isFinite) return false
        _crossAxiss[5].set(_myBodyAxiss[1]).cross(_otherBodyAxiss[2]).normalize()
        if (!_crossAxiss[5].isFinite) return false
        _crossAxiss[6].set(_myBodyAxiss[2]).cross(_otherBodyAxiss[0]).normalize()
        if (!_crossAxiss[6].isFinite) return false
        _crossAxiss[7].set(_myBodyAxiss[2]).cross(_otherBodyAxiss[1]).normalize()
        if (!_crossAxiss[7].isFinite) return false
        _crossAxiss[8].set(_myBodyAxiss[2]).cross(_otherBodyAxiss[2]).normalize()
        if (!_crossAxiss[8].isFinite) return false

        return true
    }
}

private const val EDGE_EPSILON = 1e-11