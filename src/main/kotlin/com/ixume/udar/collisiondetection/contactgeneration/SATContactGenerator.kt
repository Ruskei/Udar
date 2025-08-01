package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.Udar
import com.ixume.udar.applyIf
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.contactgeneration.GJKEPAContactGenerator.Companion.closestPointsBetweenSegments
import com.ixume.udar.collisiondetection.cycleSAT
import com.ixume.udar.collisiondetection.edgeCrosses
import com.ixume.udar.collisiondetection.mesh.Axis
import com.ixume.udar.collisiondetection.mesh.Edge
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.collisiondetection.mesh.MeshFace
import com.ixume.udar.collisiondetection.projectable
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import com.ixume.udar.testing.debugConnect
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Particle
import org.joml.Vector3d
import org.joml.Vector3i
import kotlin.collections.plusAssign
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

class SATContactGenerator(
    val activeBody: ActiveBody
) : Collidable {
    override fun capableCollision(other: Body): Capability {
        return Capability(other is EnvironmentBody, 0)
    }

    private var cachedMesh: Mesh = Mesh.Companion.mesh(world = activeBody.world, boundingBox = activeBody.boundingBox)
    private fun getMesh(): Mesh {
        val bb = activeBody.boundingBox

        val meshStart = Vector3i(
            floor(bb.minX).toInt() - 1,
            floor(bb.minY).toInt() - 1,
            floor(bb.minZ).toInt() - 1,
        )
        val meshEnd = Vector3i(
            floor(bb.maxX).toInt() + 1,
            floor(bb.maxY).toInt() + 1,
            floor(bb.maxZ).toInt() + 1,
        )

        if (meshStart.x < cachedMesh.start.x || meshStart.y < cachedMesh.start.y || meshStart.z < cachedMesh.start.z
            || meshEnd.x > cachedMesh.end.x || meshEnd.y > cachedMesh.end.y || meshEnd.z > cachedMesh.end.z
        ) {
//            println("NEW MESH!")
//            println("  - meshStart: $meshStart")
//            println("  - meshEnd: $meshEnd")
//            println("  - cachedMesh.start: ${cachedMesh.start}")
//            println("  - cachedMesh.end: ${cachedMesh.end}")

            cachedMesh = Mesh.Companion.mesh(activeBody.world, bb)
            return cachedMesh
        } else {
            return cachedMesh
        }
    }

    private fun maxChange(): Double {
        val bb = activeBody.boundingBox
        val p = activeBody.pos
        val fp = Vector3d(
            max(abs(bb.maxX - p.x), abs(bb.minX - p.x)),
            max(abs(bb.maxY - p.y), abs(bb.minY - p.y)),
            max(abs(bb.maxZ - p.z), abs(bb.minZ - p.z)),
        )

        return activeBody.velocity.length() * Udar.Companion.CONFIG.timeStep +
                Vector3d(fp).rotate(activeBody.q)
                    .distance(Vector3d(fp).rotate(activeBody.prevQ)) + Udar.Companion.CONFIG.collision.passiveSlop
    }

    private var prevMaxDepth = Udar.Companion.CONFIG.collision.passiveSlop
    override fun collides(other: Body): List<Contact> {
        require(capableCollision(other).capable)

        val mesh = getMesh()
//        if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) println("COLLIDES MESH!")

        val collisions = mutableListOf<CollisionResult>()
        val maxDepth = prevMaxDepth + maxChange() * Udar.Companion.CONFIG.sat.fudge
//        println("prevMaxDepth: $prevMaxDepth v: $velocity maxChange: ${maxChange()} maxDepth: $maxDepth")
        prevMaxDepth = 0.0

        for (cheesyFace in mesh.faces) {
            val r = collidesFace(cheesyFace) ?: continue
            if (r.isEmpty()) continue

//            println("TC! axis: ${cheesyFace.axis}")
//            println("  * valid: ${cheesyFace.valid}")

            for (p in r) {
                val valid = run validate@{
                    when (cheesyFace.axis) {
                        Axis.X -> {
                            for (invalid in cheesyFace.invalid) {
                                if (p.point.y in invalid.first.x..invalid.second.x && p.point.z in invalid.first.y..invalid.second.y) {
//                                println("INVALID BY $invalid")
                                    return@validate false
                                }
                            }

                            val firstMatch =
                                cheesyFace.valid.firstOrNull { p.point.y in it.start.x..it.end.x && p.point.z in it.start.y..it.end.y }
                            if (firstMatch == null) {
//                                println("NOT IN ANY VALID")
                                return@validate false
                            }

                            val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                            val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
//                            println("m: $m d: $d")
                            return@validate d
                        }

                        Axis.Y -> {
                            for (invalid in cheesyFace.invalid) {
                                if (p.point.x in invalid.first.x..invalid.second.x && p.point.z in invalid.first.y..invalid.second.y) {
//                                println("INVALID BY $invalid")
                                    return@validate false
                                }
                            }

                            val firstMatch =
                                cheesyFace.valid.firstOrNull { p.point.x in it.start.x..it.end.x && p.point.z in it.start.y..it.end.y }
                            if (firstMatch == null) {
//                                println("NOT IN ANY VALID")
                                return@validate false
                            }
                            val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                            val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
//                            println("m: $m d: $d")
                            return@validate d
                        }

                        Axis.Z -> {
                            for (invalid in cheesyFace.invalid) {
                                if (p.point.x in invalid.first.x..invalid.second.x && p.point.y in invalid.first.y..invalid.second.y) {
//                                println("INVALID BY $invalid")
                                    return@validate false
                                }
                            }

                            val firstMatch =
                                cheesyFace.valid.firstOrNull { p.point.x in it.start.x..it.end.x && p.point.y in it.start.y..it.end.y }
                            if (firstMatch == null) {
//                                println("NOT IN ANY VALID")
                                return@validate false
                            }
                            val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                            val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
//                            println("m: $m d: $d")
                            return@validate d
                        }
                    }
                }

                if (valid) {
//                    println("FACE-VERTEX COLLISION!")
//                    println("  - depth: ${p.depth}")
//                    println("  - norm ${p.norm}")
//                    println("  - point: ${p.point}")

                    if (p.depth > maxDepth) {
//                        println("OVERSHOT, ${p.depth / maxDepth}x MAX DEPTH with ${p.depth}")
//                        println("p: $p")

                        activeBody.world.debugConnect(
                            p.point,
                            Vector3d(p.point).add(p.norm),
                            Particle.DustOptions(Color.WHITE, 0.2f)
                        )

                        activeBody.world.spawnParticle(
                            Particle.REDSTONE,
                            Location(
                                activeBody.world,
                                p.point.x, p.point.y, p.point.z,
                            ),
                            1, Particle.DustOptions(Color.BLACK, 0.4f)
                        )

                        continue
                    }

                    if (p.depth > prevMaxDepth) {
                        prevMaxDepth = p.depth
                    }
//
                    collisions += p
                }
            }
        }

        for (edge in mesh.edges) {
            val r = collidesEdge(edge) ?: continue

            if (r.depth > maxDepth) {
//                println("OVERSHOT, ${r.depth / maxDepth}x MAX DEPTH with ${r.depth}")
//                println("r: $r")
                continue
            }

            if (r.depth > prevMaxDepth) {
                prevMaxDepth = r.depth
            }

            collisions += r
        }

        return collisions.map { Contact(activeBody, other, it) }
    }

    private fun collidesFace(
        face: MeshFace,
        normal: Vector3d = face.axis.vec,
    ): List<CollisionResult>? {
        val start = Vector3d(face.start)
        val end = Vector3d(face.end)

        require(start.x <= end.x && start.y <= end.y && start.z <= end.z)

        val boundingBox = activeBody.boundingBox

        val large = 64.0
        val (otherEdges, otherVertices) = when (face.axis) {
            Axis.X -> {
                if (boundingBox.maxX < face.level || boundingBox.minX > face.level) return null

                listOf<Vector3d>() to listOf(
                    Vector3d(start.x, start.y - large, start.z - large),
                    Vector3d(start.x, end.y + large, start.z - large),
                    Vector3d(start.x, end.y + large, end.z + large),
                    Vector3d(start.x, start.y - large, end.z + large),
                )
            }

            Axis.Y -> {
                if (boundingBox.maxY < face.level || boundingBox.minY > face.level) return null

                listOf<Vector3d>() to listOf(
                    Vector3d(start.x - large, start.y, start.z - large),
                    Vector3d(end.x + large, start.y, start.z - large),
                    Vector3d(end.x + large, start.y, end.z + large),
                    Vector3d(start.x - large, start.y, end.z + large),
                )
            }

            Axis.Z -> {
                if (boundingBox.maxZ < face.level || boundingBox.minZ > face.level) return null

                listOf<Vector3d>() to listOf(
                    Vector3d(start.x - large, start.y - large, start.z),
                    Vector3d(end.x + large, start.y - large, start.z),
                    Vector3d(end.x + large, end.y + large, start.z),
                    Vector3d(start.x - large, end.y + large, start.z),
                )
            }
        }

        val otherAxiss = listOf(normal)

        // val allowedNormals = listOf(normal)

        val r =
            collidesSAT(otherVertices, otherAxiss, otherEdges, findAll = true, collideMyAxiss = false) ?: return null
        if (r.isEmpty()) return null
//        if (r.size > 1) println("found: ${r.size}")

        r.sortBy { -it.depth }

        return when (face.axis) {
            Axis.X -> {
                r.filter { it.point.y in start.y..end.y && it.point.z in start.z..end.z }
            }

            Axis.Y -> {
                r.filter { it.point.x in start.x..end.x && it.point.z in start.z..end.z }
            }

            Axis.Z -> {
                r.filter { it.point.y in start.y..end.y && it.point.x in start.x..end.x }
            }
        }
    }

    private fun collidesEdge(
        edge: Edge
    ): CollisionResult? {
//        if (!bb.contains(edge.start.x, edge.start.y, edge.start.z) && !bb.contains(
//                edge.end.x,
//                edge.end.y,
//                edge.end.z
//            )
//        ) {
//            return null
//        }

        edge.axis!!
        edge.mount!!
        val otherVertices = listOf(
            edge.start,
            edge.end,
        )

        val otherEdges = listOf(edge.vec)

        val boundingBox = activeBody.boundingBox

        val allowedNormals = when (edge.axis) {
            Axis.X -> {
                if (!(boundingBox.minY <= edge.start.y && boundingBox.maxY >= edge.start.y
                            && boundingBox.minZ <= edge.start.z && boundingBox.maxZ >= edge.start.z)
                ) return null
                listOf(
                    Vector3d(0.0, -edge.mount.a, 0.0),
                    Vector3d(0.0, 0.0, -edge.mount.b),
                )
            }

            Axis.Y -> {
                if (!(boundingBox.minX <= edge.start.x && boundingBox.maxX >= edge.start.x
                            && boundingBox.minZ <= edge.start.z && boundingBox.maxZ >= edge.start.z)
                ) return null
                listOf(
                    Vector3d(-edge.mount.a, 0.0, 0.0),
                    Vector3d(0.0, 0.0, -edge.mount.b),
                )
            }

            Axis.Z -> {
                if (!(boundingBox.minX <= edge.start.x && boundingBox.maxX >= edge.start.x
                            && boundingBox.minY <= edge.start.y && boundingBox.maxY >= edge.start.y)
                ) return null
                listOf(
                    Vector3d(-edge.mount.a, 0.0, 0.0),
                    Vector3d(0.0, -edge.mount.b, 0.0),
                )
            }
        }

        val r = collidesSAT(
            otherVertices = otherVertices,
            otherAxiss = listOf(),
            otherEdges = otherEdges,
            allowedNormals = allowedNormals,
        ) ?: return null

        if (r.isEmpty()) return null

        return r.first()
    }

    private fun collidesSAT(
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

        val axiss = mutableListOf<Vector3d>()
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
}