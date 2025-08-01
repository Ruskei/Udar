package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.capability.GJKCapable
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import org.joml.Vector3d
import kotlin.math.*

class GJKEPAContactGenerator<T>(
    val activeBody: T
) : Collidable where T : Body, T : GJKCapable {
    override fun capableCollision(other: Body): Capability {
        return Capability(other.isConvex && other is GJKCapable, 0)
    }

    override fun collides(other: Body): List<Contact> {
        require(other.isConvex)
        require(other is GJKCapable)
        // map from difference to MINE to OTHER
        val originals = mutableMapOf<Vector3d, Pair<Vector3d, Vector3d>>()

        fun minkowski(dir: Vector3d): Vector3d {
            val a = activeBody.support(dir)
            val b = other.support(Vector3d(dir).negate())
            val diff = Vector3d(a).sub(b)
            originals[diff] = a to b

//            println("DIFF: $diff FROM: $a $b")

            return diff
        }

        val initialAxis = Vector3d(other.pos).sub(activeBody.pos)
        var a = minkowski(initialAxis)
        val s = mutableListOf(a)
        val d = Vector3d(a).negate()

        var i = 0
        repeat(64) {
            i++
            a = minkowski(d)
            if (a.dot(d) < 0.0) {
                return emptyList()
            }

            s += a
            if (nearestSimplex(s, d)) {
//                println("COLLISION after $i iterations")

                val aP = s[2]
                val bP = s[1]
                val cP = s[0]
                val dP = s[3]

//                println("tetrahedron: a: $aP b: $bP c: $cP d: $dP")

                //verify that norms are pointing outside
                val da = Vector3d(aP).sub(dP)
                val db = Vector3d(bP).sub(dP)
                val dc = Vector3d(cP).sub(dP)

                val ab = Vector3d(bP).sub(aP)
                val ac = Vector3d(cP).sub(aP)

                val dab = Vector3d(da).cross(db) //SHOULD FACE AWAY FROM C
                val dbc = Vector3d(db).cross(dc) //SHOULD FACE AWAY FROM A
                val dca = Vector3d(dc).cross(da) //SHOULD FACE AWAY FROM B
                val abc = Vector3d(ac).cross(ab) //SHOULD FACE AWAY FROM D

                val winding = abc.dot(dP) < 0.0 //TRUE: COUNTERCLOCKWISE, FALSE: CLOCKWISE
//                println("WINDING: ${if (winding) "COUNTERCLOCKWISE" else "CLOCKWISE"}")
                check(winding == (dab.dot(cP) < 0.0) == (dbc.dot(aP) < 0.0) == (dca.dot(bP) < 0.0)) //EVERYTHING MUST BE WOUND THE SAME WAY

                val shape = mutableListOf(
                    if (winding) Triple(aP, cP, bP) else Triple(aP, bP, cP),
                    if (winding) Triple(aP, bP, dP) else Triple(aP, dP, bP),
                    if (winding) Triple(bP, cP, dP) else Triple(bP, dP, cP),
                    if (winding) Triple(cP, aP, dP) else Triple(cP, dP, aP),
                )

                repeat(64) {
                    val dn = closestNormal(Vector3d(), shape)
                    if (dn != null) {
                        val (dis, norm, faces) = dn

                        val sup = minkowski(norm)

                        val dis2 = sup.dot(norm)
                        if (dis2 - dis < 0.0000001) {
                            var coefficients: Vector3d? = null
                            var closestFace: Triple<Vector3d, Vector3d, Vector3d>? = null
//                            println("faces.size: ${faces.size}")
                            for (face in faces) {
                                val r = toBarycentric(Vector3d(), face.first, face.second, face.third, true)
                                if (r != null) {
                                    coefficients = r
                                    closestFace = face
                                }
                            }

                            if (coefficients == null) {
                                throw IllegalStateException("No coefficients found")
//                                AbstractParticleEmitter.INSTANCE.logger.warning("No coefficients found")
//                                coefficients = toBarycentric(Vector3d(), faces[0].first, faces[0].second, faces[0].third, false)!!
//                                closestFace = faces[0]
                            }

                            val (a3, b3, c3) = closestFace!!
                            val (a3mine, a3other) = originals[a3]!!
                            val (b3mine, b3other) = originals[b3]!!
                            val (c3mine, c3other) = originals[c3]!!

                            val aC = coefficients.x
                            val bC = coefficients.y
                            val cC = coefficients.z
                            val doDebug = aC + bC + cC > 1.001
                            val myInt =
                                Vector3d(a3mine).mul(aC).add(Vector3d(b3mine).mul(bC)).add(Vector3d(c3mine).mul(cC))
                            val otherInt =
                                Vector3d(a3other).mul(aC).add(Vector3d(b3other).mul(bC))
                                    .add(Vector3d(c3other).mul(cC))
                            if (doDebug) {
                                println(
                                    """
                                    minkowski:
                                     - a: $a3
                                     - b: $b3
                                     - c: $c3
                                    mine: $myInt
                                     - a: $a3mine C: $aC
                                     - b: $b3mine C: $bC
                                     - c: $c3mine C: $cC
                                    other: $otherInt
                                     - a: $a3other C: $aC
                                     - b: $b3other C: $bC
                                     - c: $c3other C: $cC
                                    shape:${"\n"}${shape.joinToString(separator = "FACE:\n") { " - a: ${it.first}\n - b: ${it.second}\n - c: ${it.third}\n" }}
                                """.trimIndent()
                                )
                            }

                            return listOf(
                                Contact(
                                    activeBody, other, CollisionResult(
                                        myInt,
                                        Vector3d(norm).negate(),
                                        dis2,
//                                shape,
//                                closestFace,
//                                originals,
                                    )
                                )
                            )
                        } else {
                            addPoint(shape, sup)
                        }
                    } else {
                        //1d epa
                        var dClosest = Double.MAX_VALUE
                        var dNorm = Vector3d()
                        var vertexClosest: Vector3d? = null

                        for ((a2, b2, c2) in shape) {
                            val a2l = a2.length()
                            val b2l = b2.length()
                            val c2l = c2.length()

                            if (a2l < dClosest || b2l < dClosest || c2l < dClosest)
                                if (a2l < b2l) {
                                    // a < b
                                    if (c2l < a2l) {
                                        // c < a < b

                                        dClosest = c2l
                                        dNorm = Vector3d(c2).normalize()
                                        vertexClosest = c2
                                    } else {
                                        dClosest = a2l
                                        dNorm = Vector3d(a2).normalize()
                                        vertexClosest = a2
                                    }
                                } else {
                                    // b < a
                                    if (c2l < b2l) {
                                        // c < b < a
                                        dClosest = c2l
                                        dNorm = Vector3d(c2).normalize()
                                        vertexClosest = c2
                                    } else {
                                        dClosest = b2l
                                        dNorm = Vector3d(b2).normalize()
                                        vertexClosest = b2
                                    }
                                }
                        }

                        if (dNorm == Vector3d() || !dNorm.isFinite) return emptyList()
                        val (closestMine, _) = originals[vertexClosest!!]!!

                        return listOf(
                            Contact(
                                activeBody, other, CollisionResult(
                                    closestMine,
                                    Vector3d(dNorm).negate(),
                                    dClosest,
//                            shape,
//                            Triple(vertexClosest, vertexClosest, vertexClosest),
//                            originals,
                                )
                            )
                        )
                    }
                }

                return emptyList()

            }
        }

        return emptyList()
    }

    private fun nearestSimplex(s: MutableList<Vector3d>, dir: Vector3d): Boolean {
        when (s.size) {
            2 -> {
                val a = s[1]
                val b = s[0]
                val ab = Vector3d(b).sub(a)
                val ao = Vector3d(a).negate()

                dir.set(Vector3d(ab).cross(ao).cross(ab))

                return false
            }

            3 -> {
                val a = s[2]
                val b = s[1]
                val c = s[0]

                val ab = Vector3d(b).sub(a)
                val ac = Vector3d(c).sub(a)
                val ao = Vector3d(a).negate()

                val abc = Vector3d(ab).cross(ac)

                dir.set(Vector3d(abc).mul(if (abc.dot(ao) > 0.0) 1.0 else -1.0))

                return false
            }

            4 -> {
                val a = s[2]
                val b = s[1]
                val c = s[0]
                val d = s[3]

                val da = Vector3d(a).sub(d)
                val db = Vector3d(b).sub(d)
                val dc = Vector3d(c).sub(d)

                val d0 = Vector3d(d).negate()

                val dab = Vector3d(da).cross(db)
                val dbc = Vector3d(db).cross(dc)
                val dca = Vector3d(dc).cross(da)

                if (dab.dot(d0) > 0.0) {
                    s.remove(c)
                    dir.set(dab)

                    return false
                } else if (dbc.dot(d0) > 0.0) {
                    s.remove(a)
                    dir.set(dbc)

                    return false
                } else if (dca.dot(d0) > 0.0) {
                    s.remove(b)
                    dir.set(dca)

                    return false
                } else {
                    return true
                }
            }

            else -> throw IllegalArgumentException("Simplex not in R^(1..3)")
        }
    }


    data class ClosestResult(
        val distance: Double,
        val normal: Vector3d,
        val faces: List<Triple<Vector3d, Vector3d, Vector3d>>,
    )

    private fun addPoint(shape: MutableList<Triple<Vector3d, Vector3d, Vector3d>>, point: Vector3d) {
        val epsilon = 1e-11
        // holds vertices associated with edges
        // non uniques will always be wound 2 ways, and will be removed
        // so unique leftovers are all wound correctly
        val uniqueEdges = mutableListOf<Pair<Vector3d, Vector3d>>()
        val facesToRemove = mutableListOf<Triple<Vector3d, Vector3d, Vector3d>>()

        for (face in shape) {
            val (a, b, c) = face
            val ab = Vector3d(b).sub(a)
            val ac = Vector3d(c).sub(a)

            val n = Vector3d(ab).cross(ac)
            val relP = Vector3d(point).sub(a)
            if (relP.dot(n) > 0.0) {
                //if any edge in 'uniqueEdges' is the same or the reverse of ab, ac, or cb, remove it
                //otherwise, add edge in wound order
                var foundAB = false
                var foundBC = false
                var foundCA = false

                val edgesToRemove = mutableListOf<Pair<Vector3d, Vector3d>>()
                for (uniqueEdge in uniqueEdges) {
                    if (
                        (uniqueEdge.first.distanceSquared(a) < epsilon && uniqueEdge.second.distanceSquared(b) < epsilon)
                        || (uniqueEdge.first.distanceSquared(b) < epsilon && uniqueEdge.second.distanceSquared(a) < epsilon)
                    ) {
                        foundAB = true
                        edgesToRemove += uniqueEdge
                        continue
                    }

                    if (
                        (uniqueEdge.first.distanceSquared(b) < epsilon && uniqueEdge.second.distanceSquared(c) < epsilon)
                        || (uniqueEdge.first.distanceSquared(c) < epsilon && uniqueEdge.second.distanceSquared(b) < epsilon)
                    ) {
                        foundBC = true
                        edgesToRemove += uniqueEdge
                        continue
                    }

                    if (
                        (uniqueEdge.first.distanceSquared(c) < epsilon && uniqueEdge.second.distanceSquared(a) < epsilon)
                        || (uniqueEdge.first.distanceSquared(a) < epsilon && uniqueEdge.second.distanceSquared(c) < epsilon)
                    ) {
                        foundCA = true
                        edgesToRemove += uniqueEdge
                        continue
                    }
                }

                uniqueEdges.removeAll(edgesToRemove)

                if (!foundAB) {
                    uniqueEdges += a to b
                }

                if (!foundBC) {
                    uniqueEdges += b to c
                }

                if (!foundCA) {
                    uniqueEdges += c to a
                }

                facesToRemove += face

            }
        }

        shape.removeAll(facesToRemove)
        facesToRemove.clear()

        for ((uniqueStart, uniqueEnd) in uniqueEdges) {
            shape += Triple(uniqueStart, uniqueEnd, point)
        }

//        if (!shape.isConvex()) {
//            println("CONCAVE SHAPE")
//        }
    }

    companion object {

        fun toBarycentric(
            point: Vector3d,
            a: Vector3d,
            b: Vector3d,
            c: Vector3d,
            failhard: Boolean = false,
        ): Vector3d? {
            //first make sure point is on same plane as a,b,c
            val ab = Vector3d(b).sub(a)
            val bc = Vector3d(c).sub(b)
            val ca = Vector3d(a).sub(c)

            val arp = Vector3d(point).sub(a)
            val n = Vector3d(ab).cross(ca).normalize()
//            val fac = if (n.dot(point))
            val d = Vector3d(arp).sub(Vector3d(n).mul(n.dot(arp))).add(a)

            val da = Vector3d(d).sub(a)
            val db = Vector3d(d).sub(b)
            val dc = Vector3d(d).sub(c)

            val total = 0.5 * Vector3d(ab).cross(ca).length()

            val dabA = 0.5 * Vector3d(da).cross(ab).length()
            val cA = dabA / total
            val dbcA = 0.5 * Vector3d(db).cross(bc).length()
            val cB = dbcA / total
            val dcaA = 0.5 * Vector3d(dc).cross(ca).length()
            val cC = dcaA / total

            if (cA + cB + cC > 1.001) {
//                println("WARNING: cA: $cA cB: $cB cC: $cC SUM: ${cA + cB + cC}\ntotal area: $total dab: $dabA dbc: $dbcA dca: $dcaA")
                if (failhard) return null
            }
//
            return Vector3d(cB, cC, cA)
        }

        fun closestNormal(point: Vector3d, shape: List<Triple<Vector3d, Vector3d, Vector3d>>): ClosestResult? {
//            println("CLOSEST NORMAL")
            var dClosest = Double.MAX_VALUE
            var nClosest = Vector3d()
            var closestFaces: MutableList<Triple<Vector3d, Vector3d, Vector3d>> = mutableListOf()

            var winding: Double? = null

            for ((a, b, c) in shape) {
                val relPoint = Vector3d(point).sub(a)
                val ab = Vector3d(b).sub(a)
                val ac = Vector3d(c).sub(a)

                val n = Vector3d(ab).cross(ac).normalize()
                if (!n.isFinite) {
                    //a,b,c are collinear, resolution vector is just 1d epa aka whichever direction is closest to origin
                    return null
                }

                val d = Vector3d(n).dot(relPoint)
//                println("d: $d winding: $winding")
                if (winding == null) winding = sign(d)
                else if (sign(d) != winding) println("WINDING MISMATCH")
//                println("  - a: $a b: $b c: $c n: $n d: $d")
//                println("d: $d")
//            println("  - a: $a b: $b c: $c n: $n d: $d")

                if (abs(d.absoluteValue - dClosest) < 0.0000001) {
                    closestFaces += Triple(a, b, c)
                } else if (d.absoluteValue < dClosest) {
                    dClosest = d.absoluteValue
                    nClosest = n
                    closestFaces = mutableListOf(Triple(a, b, c))
                }
            }

            check(nClosest.lengthSquared() > 0.0)

            return ClosestResult(dClosest, nClosest, closestFaces)
        }

        private fun Vector3d.inside(
            xRange: ClosedRange<Double>,
            yRange: ClosedRange<Double>,
            zRange: ClosedRange<Double>,
        ): Boolean {
            return x in xRange && y in yRange && z in zRange
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
            val an = Vector3d(a).normalize()
            val bn = Vector3d(b).normalize()
            val cn = Vector3d(an).cross(bn).normalize()
            if (!cn.isFinite) return null

            val d = Vector3d(b0).sub(a0)

            val r = Vector3d(d).sub(Vector3d(a).mul(d.dot(a))).sub(Vector3d(cn).mul(d.dot(cn)))
            if (r.length() < 0.0000001) return null
            val bClosest = Vector3d(b0).sub(Vector3d(b).mul(r.length() / b.dot(Vector3d(r).normalize())))
            val c = Vector3d(cn).mul(Vector3d(d).sub(Vector3d(a).mul(d.dot(a))).dot(cn))

            return Triple(Vector3d(bClosest).sub(c), bClosest, c.length())
        }

        /**
         * Closest point on a line to another point
         * @return the closest point, the distance
         */
        private fun closestPointOnLine(x0: Vector3d, x: Vector3d, p: Vector3d): Pair<Vector3d, Double> {
            val xn = Vector3d(x).normalize()
            val closestPoint = Vector3d(x0).add(Vector3d(x).mul(Vector3d(p).sub(x0).dot(xn)))

            return closestPoint to closestPoint.distance(p)
        }
    }
}