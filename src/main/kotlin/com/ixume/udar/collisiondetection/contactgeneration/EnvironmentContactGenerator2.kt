package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.body.Body
import com.ixume.udar.body.Collidable
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D
import com.ixume.udar.collisiondetection.mesh.mesh2.EdgeMountAllowedNormals
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFace
import com.ixume.udar.collisiondetection.mesh.quadtree.EdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.MeshRequest
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicReference

class EnvironmentContactGenerator2(
    val activeBody: ActiveBody,
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if (other is EnvironmentBody) 0 else -1
    }

    private val mesher = LocalMesher()

    private val cachedMesh: AtomicReference<LocalMesher.Mesh2?> = AtomicReference(null)

    private fun getMesh(): LocalMesher.Mesh2? {
//        val bb = activeBody.fatBB
        val pos = activeBody.pos

        val meshBB = AABB(
            minX = pos.x - 8.0,
            minY = pos.y - 8.0,
            minZ = pos.z - 8.0,
            maxX = pos.x + 8.0,
            maxY = pos.y + 8.0,
            maxZ = pos.z + 8.0,
        )

        val cm = cachedMesh.get()
        if (cm == null) {
            activeBody.physicsWorld.realWorldHandler.getter2.fetchBBs(
                MeshRequest(
                    meshBB,
                    mesher
                ) { cachedMesh.set(it) })
        }

        return cm
    }

    private var testedFaces = 0

    override fun collides(
        other: Body,
        math: LocalMathUtil,
    ): List<Contact> {
//        println("COLLIDING ENV!")
        val mesh = getMesh() ?: return emptyList()
//        mesh.visualize(activeBody.world)
        val bb = activeBody.tightBB

        val contacts = mutableListOf<Contact>()

        testedFaces = 0

        mesh.faces?.xFaces?.facesIn(bb)?.let { collideFaces(it, LocalMesher.AxisD.X, math, other, contacts) }
        mesh.faces?.yFaces?.facesIn(bb)?.let { collideFaces(it, LocalMesher.AxisD.Y, math, other, contacts) }
        mesh.faces?.zFaces?.facesIn(bb)?.let { collideFaces(it, LocalMesher.AxisD.Z, math, other, contacts) }

        setupBodyAxiss()

//        println("X EDGES!")
        mesh.xEdges?.let {
            collideEdges(
                tree = it,
                math = math,
                other = other,
                out = contacts,
            )
        }

//        println("Y EDGES!")
        mesh.yEdges?.let {
            collideEdges(
                tree = it,
                math = math,
                other = other,
                out = contacts,
            )
        }

//        println("Z EDGES!")
        mesh.zEdges?.let {
            collideEdges(
                tree = it,
                math = math,
                other = other,
                out = contacts,
            )
        }

        return contacts
    }

    private fun collideFaces(
        faces: List<MeshFace>,
        axis: LocalMesher.AxisD,
        math: LocalMathUtil,
        other: Body,
        out: MutableList<Contact>,
    ) {
//        println("COLLIDING $axis FACES!")
        val bb = activeBody.tightBB
        val arr = DoubleArray(4)
        //faces are guaranteed to be inside BB
        val bb2d = when (axis) {
            LocalMesher.AxisD.X -> {
                arr[0] = bb.minY
                arr[1] = bb.minZ
                arr[2] = bb.maxY
                arr[3] = bb.maxZ
                AABB2D(arr)
            }

            LocalMesher.AxisD.Y -> {
                arr[0] = bb.minX
                arr[1] = bb.minZ
                arr[2] = bb.maxX
                arr[3] = bb.maxZ
                AABB2D(arr)
            }

            LocalMesher.AxisD.Z -> {
                arr[0] = bb.minX
                arr[1] = bb.minY
                arr[2] = bb.maxX
                arr[3] = bb.maxY
                AABB2D(arr)
            }
        }

        val vertices = activeBody.vertices

        for (face in faces) {
            testedFaces++
            val collisions = math.collidePlane(
                axis = axis,
                level = face.level,
                vertices = vertices
            ) ?: continue

            val overlappingAntiHoles = face.antiHoles.overlaps(bb2d)
            val overlappingHoles = face.holes.overlaps(bb2d)

            for (collision in collisions) {
                var valid = true
                for (antiHole in overlappingAntiHoles) {
                    if (antiHole.contains(collision.pointA.get(axis.aOffset), collision.pointA.get(axis.bOffset))) {
                        valid = false
                        break
                    }
                }

                if (!valid) continue

                valid = false

                for (hole in overlappingHoles) {
                    if (hole.contains(collision.pointA.get(axis.aOffset), collision.pointA.get(axis.bOffset))) {
                        valid = true
                        break
                    }
                }

                if (!valid) continue

                out += Contact(
                    first = activeBody,
                    second = other,
                    result = collision,
                )
            }
        }
    }

    private val _bodyAxiss = Array(3) { Vector3d() }
    private val _crossAxiss = Array(3) { Vector3d() }

    private fun setupBodyAxiss() {
        _bodyAxiss[0].set(1.0, 0.0, 0.0).rotate(activeBody.q).normalize()
        _bodyAxiss[1].set(0.0, 1.0, 0.0).rotate(activeBody.q).normalize()
        _bodyAxiss[2].set(0.0, 0.0, 1.0).rotate(activeBody.q).normalize()
    }

    private fun setupCrossAxiss(edge: Vector3d): Boolean {
        _crossAxiss[0].set(_bodyAxiss[0]).cross(edge).normalize()
        if (!_crossAxiss[0].isFinite) return false
        _crossAxiss[1].set(_bodyAxiss[1]).cross(edge).normalize()
        if (!_crossAxiss[1].isFinite) return false
        _crossAxiss[2].set(_bodyAxiss[2]).cross(edge).normalize()
        return _crossAxiss[2].isFinite
    }

    private val _edgeStart = Vector3d()
    private val _edgeEnd = Vector3d()
    private val _allowedNormals = arrayOf(Vector3d(), Vector3d())

    private fun collideEdges(
        tree: EdgeQuadtree,
        math: LocalMathUtil,
        other: Body,
        out: MutableList<Contact>,
    ) {
//        println("COLLIDING EDGES!")
        /*
        get all edges that we could possibly be colliding with; we could have a valid collision with each of these edges (not using discrete curvature graph rn)
         */

        val edges = tree.overlaps(activeBody.tightBB)
        if (edges.isEmpty()) return

        val vertices = activeBody.vertices

        val axis = tree.axis

        if (!setupCrossAxiss(axis.vec)) return

        for (edge in edges) {
            /*
            edge is represnted by a sorted list of doubles, so we jump 2 at a time through them and those are our edge starts and ends
                might eventually be worth it to convert this into a sorted set to do O(log n) traversal instead of O(n)
             */
            _edgeStart.setComponent(axis.aOffset, edge.a)
            _edgeStart.setComponent(axis.bOffset, edge.b)

            _edgeEnd.setComponent(axis.aOffset, edge.a)
            _edgeEnd.setComponent(axis.bOffset, edge.b)

//            println("POINTS: ${edge.points.joinToString { it.toString() }}")
            val itr = edge.points.iterator()
            var i = 0
            while (itr.hasNext()) {
                val d1 = itr.nextDouble()
                val d2 = itr.nextDouble()

                val mount = edge.pointMounts.getInt(i)

                _allowedNormals[0].setComponent(axis.levelOffset, 0.0)
                _allowedNormals[0].setComponent(axis.aOffset, EdgeMountAllowedNormals.allowedNormals[mount][0].x)
                _allowedNormals[0].setComponent(axis.bOffset, EdgeMountAllowedNormals.allowedNormals[mount][0].y)

                _allowedNormals[1].setComponent(axis.levelOffset, 0.0)
                _allowedNormals[1].setComponent(axis.aOffset, EdgeMountAllowedNormals.allowedNormals[mount][1].x)
                _allowedNormals[1].setComponent(axis.bOffset, EdgeMountAllowedNormals.allowedNormals[mount][1].y)

                _edgeStart.setComponent(axis.levelOffset, d1)
                _edgeEnd.setComponent(axis.levelOffset, d2)

                val r = math.collideCuboidEdge(
                    edgeStart = _edgeStart,
                    edgeEnd = _edgeEnd,
                    bodyAxiss = _bodyAxiss,
                    crossAxiss = _crossAxiss,
                    vertices = vertices,
                    allowedNormals = _allowedNormals,
                )

                if (r == null) {
                    i++
                    continue
                }

//                println("COLLIDED EDGE")
//                println("  - norm: ${r.norm}")
//                println("  - point: ${r.pointA}")
//                println("  - allowedNormals[0]: ${_allowedNormals[0]}")
//                println("  - allowedNormals[1]: ${_allowedNormals[1]}")
//                println("  - mount: $mount")

                out += Contact(
                    first = activeBody,
                    second = other,
                    result = r,
                )

                i++
            }
        }
    }
}