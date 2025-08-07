package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.Udar
import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.Axis
import com.ixume.udar.collisiondetection.mesh.Edge
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.collisiondetection.mesh.MeshFace
import com.ixume.udar.physics.BBRequest
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import com.ixume.udar.testing.debugConnect
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Particle
import org.joml.Vector3d
import org.joml.Vector3i
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max

class EnvironmentSATContactGenerator(
    val activeBody: ActiveBody
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if (other is EnvironmentBody) 0 else -1
    }

    private val cachedMesh: AtomicReference<Mesh> =
        AtomicReference(Mesh.Companion.mesh(world = activeBody.world, boundingBox = activeBody.boundingBox))

    private fun setMesh(value: Mesh) {
        do {
            val curr = cachedMesh.get()
        } while (!cachedMesh.compareAndSet(curr, value))
    }

    private fun getMesh(): Mesh {
        val bb = activeBody.boundingBox

        val cm = cachedMesh.get()

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

        if (meshStart.x < cm.start.x || meshStart.y < cm.start.y || meshStart.z < cm.start.z
            || meshEnd.x > cm.end.x || meshEnd.y > cm.end.y || meshEnd.z > cm.end.z
        ) {
            activeBody.physicsWorld.realWorldGetter.fetchBBs(BBRequest(bb) { setMesh(it) })

            return cm
        } else {
            return cm
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
    var facesChecked = 0
    override fun collides(other: Body, math: LocalMathUtil): List<Contact> {
        require(capableCollision(other) >= 0)

        val mesh = getMesh()
//        mesh.visualize(
//            world = activeBody.world,
//            visualizeFaces = true,
//            visualizeEdges = true,
//        )
//        if (PhysicsCommand.DEBUG_SAT_LEVEL > 0) println("COLLIDES MESH!")

        val collisions = mutableListOf<CollisionResult>()
        val maxDepth = prevMaxDepth + maxChange() * Udar.Companion.CONFIG.sat.fudge
        prevMaxDepth = 0.0

        for (cheesyFace in mesh.faces) {
            val r = collidesFace(cheesyFace, math = math) ?: continue
            if (r.isEmpty()) continue

            for (p in r) {
                val valid = run validate@{
                    when (cheesyFace.axis) {
                        Axis.X -> {
                            if (cheesyFace.invalid.size < cheesyFace.valid.size) {
                                for (invalid in cheesyFace.invalid) {
                                    if (p.point.y in invalid.first.x..invalid.second.x && p.point.z in invalid.first.y..invalid.second.y) {
                                        return@validate false
                                    }
                                }

                                val firstMatch =
                                    cheesyFace.valid.firstOrNull { p.point.y in it.start.x..it.end.x && p.point.z in it.start.y..it.end.y }
                                if (firstMatch == null) {
                                    return@validate false
                                }

                                val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                                val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
                                return@validate d
                            } else {
                                val firstMatch =
                                    cheesyFace.valid.firstOrNull { p.point.y in it.start.x..it.end.x && p.point.z in it.start.y..it.end.y }
                                if (firstMatch == null) {
                                    return@validate false
                                }

                                val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                                val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)

                                if (!d) return@validate false

                                for (invalid in cheesyFace.invalid) {
                                    if (p.point.y in invalid.first.x..invalid.second.x && p.point.z in invalid.first.y..invalid.second.y) {
                                        return@validate false
                                    }
                                }

                                return@validate true
                            }
                        }

                        Axis.Y -> {
                            if (cheesyFace.invalid.size < cheesyFace.valid.size) {
                                for (invalid in cheesyFace.invalid) {
                                    if (p.point.x in invalid.first.x..invalid.second.x && p.point.z in invalid.first.y..invalid.second.y) {
                                        return@validate false
                                    }
                                }

                                val firstMatch =
                                    cheesyFace.valid.firstOrNull { p.point.x in it.start.x..it.end.x && p.point.z in it.start.y..it.end.y }
                                if (firstMatch == null) {
                                    return@validate false
                                }
                                val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                                val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
                                return@validate d
                            } else {
                                val firstMatch =
                                    cheesyFace.valid.firstOrNull { p.point.x in it.start.x..it.end.x && p.point.z in it.start.y..it.end.y }
                                if (firstMatch == null) {
                                    return@validate false
                                }

                                val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                                val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
                                if (!d) return@validate false

                                for (invalid in cheesyFace.invalid) {
                                    if (p.point.x in invalid.first.x..invalid.second.x && p.point.z in invalid.first.y..invalid.second.y) {
                                        return@validate false
                                    }
                                }

                                return@validate true
                            }
                        }

                        Axis.Z -> {
                            if (cheesyFace.invalid.size < cheesyFace.valid.size) {
                                for (invalid in cheesyFace.invalid) {
                                    if (p.point.x in invalid.first.x..invalid.second.x && p.point.y in invalid.first.y..invalid.second.y) {
                                        return@validate false
                                    }
                                }

                                val firstMatch =
                                    cheesyFace.valid.firstOrNull { p.point.x in it.start.x..it.end.x && p.point.y in it.start.y..it.end.y }
                                if (firstMatch == null) {
                                    return@validate false
                                }
                                val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                                val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
                                return@validate d
                            } else {
                                val firstMatch =
                                    cheesyFace.valid.firstOrNull { p.point.x in it.start.x..it.end.x && p.point.y in it.start.y..it.end.y }
                                if (firstMatch == null) {
                                    return@validate false
                                }
                                val m = if (firstMatch.inAxisDir) 1.0 else -1.0
                                val d = (p.norm.dot(cheesyFace.axis.vec) * m >= 0)
                                if (!d) return@validate false

                                for (invalid in cheesyFace.invalid) {
                                    if (p.point.x in invalid.first.x..invalid.second.x && p.point.y in invalid.first.y..invalid.second.y) {
                                        return@validate false
                                    }
                                }

                                return@validate true
                            }
                        }
                    }
                }

                if (valid) {
                    if (p.depth > maxDepth) {
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

//                        continue
                    }

                    if (p.depth > prevMaxDepth) {
                        prevMaxDepth = p.depth
                    }
//
                    collisions += p
                }
            }
        }

        facesChecked = 0

        for (edge in mesh.edges) {
            val r = collidesEdge(edge, math = math) ?: continue

            if (r.depth > maxDepth) {
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
        math: LocalMathUtil,
    ): List<CollisionResult>? {
        val start = Vector3d(face.start)
        val end = Vector3d(face.end)

        require(start.x <= end.x && start.y <= end.y && start.z <= end.z)

        val boundingBox = activeBody.boundingBox

        val large = 64.0
        val (otherEdges, otherVertices) = when (face.axis) {
            Axis.X -> {
                if (boundingBox.maxX < face.level || boundingBox.minX > face.level) return null

                listOf<Vector3d>() to arrayOf(
                    Vector3d(start.x, start.y - large, start.z - large),
                    Vector3d(start.x, end.y + large, start.z - large),
                    Vector3d(start.x, end.y + large, end.z + large),
                    Vector3d(start.x, start.y - large, end.z + large),
                )
            }

            Axis.Y -> {
                if (boundingBox.maxY < face.level || boundingBox.minY > face.level) return null

                listOf<Vector3d>() to arrayOf(
                    Vector3d(start.x - large, start.y, start.z - large),
                    Vector3d(end.x + large, start.y, start.z - large),
                    Vector3d(end.x + large, start.y, end.z + large),
                    Vector3d(start.x - large, start.y, end.z + large),
                )
            }

            Axis.Z -> {
                if (boundingBox.maxZ < face.level || boundingBox.minZ > face.level) return null

                listOf<Vector3d>() to arrayOf(
                    Vector3d(start.x - large, start.y - large, start.z),
                    Vector3d(end.x + large, start.y - large, start.z),
                    Vector3d(end.x + large, end.y + large, start.z),
                    Vector3d(start.x - large, end.y + large, start.z),
                )
            }
        }

        val otherAxiss = listOf(normal)

        facesChecked++
        val r =
            math.collidesSAT(
                activeBody,
                otherVertices,
                otherAxiss,
                otherEdges,
                findAll = true,
                collideMyAxiss = false
            ) ?: return null
        if (r.isEmpty()) return null

        return (when (face.axis) {
            Axis.X -> {
                r.filter { it.point.y in start.y..end.y && it.point.z in start.z..end.z }
            }

            Axis.Y -> {
                r.filter { it.point.x in start.x..end.x && it.point.z in start.z..end.z }
            }

            Axis.Z -> {
                r.filter { it.point.y in start.y..end.y && it.point.x in start.x..end.x }
            }
        }).sortedBy { -it.depth }
    }

    private fun collidesEdge(
        edge: Edge, math: LocalMathUtil
    ): CollisionResult? {
        edge.axis!!
        edge.mount!!
        val otherVertices = arrayOf(
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

        val r = math.collidesSAT(
            activeBody,
            otherVertices = otherVertices,
            otherAxiss = listOf(),
            otherEdges = otherEdges,
            allowedNormals = allowedNormals,
        ) ?: return null

        if (r.isEmpty()) return null

        return r.first()
    }
}