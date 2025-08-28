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
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaceSortedList
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.physics.Contact
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList
import org.joml.Vector3d
import java.util.concurrent.atomic.AtomicReference

class EnvironmentContactGenerator2(
    val activeBody: ActiveBody,
) : Collidable {
    override fun capableCollision(other: Body): Int {
        return if (other is EnvironmentBody) 0 else -1
    }

    val meshFaces = AtomicReference<List<MeshFaceSortedList>>(emptyList())
    val meshEdges = AtomicReference<List<FlattenedEdgeQuadtree>>(emptyList())

    private val prevBB: AABB = AABB(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    private var testedFaces = 0

    override fun collides(
        other: Body,
        math: LocalMathUtil,
    ): List<Contact> {
        val bb = activeBody.tightBB

        val contacts = mutableListOf<Contact>()

        testedFaces = 0

        activeBody.physicsWorld.worldMeshesManager.request(
            prevBB = prevBB,
            currentBB = bb,
            envContactGenerator = this
        )

        val mfs = meshFaces.get()
        val mes = meshEdges.get()

        for (mf in mfs) {
            collideFaces(mf.facesIn(bb), mf.axis, math, other, contacts)
        }

        setupBodyAxiss()

        for (me in mes) {
            collideEdges(
                tree = me,
                math = math,
                other = other,
                out = contacts
            )
        }

        bb.writeTo(prevBB)

        return contacts
    }

    private val _bb2d = AABB2D(doubleArrayOf(0.0, 0.0, 0.0, 0.0))
    private fun collideFaces(
        faces: List<MeshFace>,
        axis: LocalMesher.AxisD,
        math: LocalMathUtil,
        other: Body,
        out: MutableList<Contact>,
    ) {
//        println("COLLIDING $axis FACES!")
        val bb = activeBody.tightBB
        //faces are guaranteed to be inside BB
        when (axis) {
            LocalMesher.AxisD.X -> {
                _bb2d.data[0] = bb.minY
                _bb2d.data[1] = bb.minZ
                _bb2d.data[2] = bb.maxY
                _bb2d.data[3] = bb.maxZ
            }

            LocalMesher.AxisD.Y -> {
                _bb2d.data[0] = bb.minX
                _bb2d.data[1] = bb.minZ
                _bb2d.data[2] = bb.maxX
                _bb2d.data[3] = bb.maxZ
            }

            LocalMesher.AxisD.Z -> {
                _bb2d.data[0] = bb.minX
                _bb2d.data[1] = bb.minY
                _bb2d.data[2] = bb.maxX
                _bb2d.data[3] = bb.maxY
            }
        }

        val vertices = activeBody.vertices

        var i = 0
        while (i < faces.size) {
            val face = faces[i]
            testedFaces++
            val collisions = math.collidePlane(
                axis = axis,
                level = face.level,
                vertices = vertices
            )

            if (collisions == null) {
                i++
                continue
            }

            val overlappingHoles = DoubleArrayList()

            face.holes.overlaps(
                minX = _bb2d.minA(),
                minY = _bb2d.minB(),
                maxX = _bb2d.maxA(),
                maxY = _bb2d.maxB(),
                out = overlappingHoles,
            )

            if (overlappingHoles.isEmpty()) {
                i++
                continue
            }

            val overlappingAntiHoles = DoubleArrayList()

            face.antiHoles.overlaps(
                minX = _bb2d.minA(),
                minY = _bb2d.minB(),
                maxX = _bb2d.maxA(),
                maxY = _bb2d.maxB(),
                out = overlappingAntiHoles,
            )

            var j = 0
            while (j < collisions.size) {
                val collision = collisions[j]

                val pa = collision.pointA.get(axis.aOffset)
                val pb = collision.pointA.get(axis.bOffset)

                var valid = true
                var k = 0
                while (k < overlappingAntiHoles.size) {
                    val minA = overlappingAntiHoles.getDouble(k)
                    val minB = overlappingAntiHoles.getDouble(k + 1)
                    val maxA = overlappingAntiHoles.getDouble(k + 2)
                    val maxB = overlappingAntiHoles.getDouble(k + 3)

                    if (contains(
                            minA = minA,
                            minB = minB,
                            maxA = maxA,
                            maxB = maxB,
                            a = pa,
                            b = pb,
                        )
                    ) {
                        valid = false
                        break
                    }

                    k++
                }

                if (!valid) {
                    j++
                    continue
                }

                valid = false

                var l = 0
                while (l < overlappingHoles.size) {
                    val minA = overlappingHoles.getDouble(k)
                    val minB = overlappingHoles.getDouble(k + 1)
                    val maxA = overlappingHoles.getDouble(k + 2)
                    val maxB = overlappingHoles.getDouble(k + 3)

                    if (contains(
                            minA = minA,
                            minB = minB,
                            maxA = maxA,
                            maxB = maxB,
                            a = pa,
                            b = pb,
                        )
                    ) {
                        valid = true
                        break
                    }

                    l++
                }

                if (!valid) {
                    j++
                    continue
                }

                out += Contact(
                    first = activeBody,
                    second = other,
                    result = collision,
                )

                j++
            }

            i++
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
        tree: FlattenedEdgeQuadtree,
        math: LocalMathUtil,
        other: Body,
        out: MutableList<Contact>,
    ) {
//        println("COLLIDING EDGES!")
        /*
        get all edges that we could possibly be colliding with; we could have a valid collision with each of these edges (not using discrete curvature graph rn)
         */

        val outA = DoubleArrayList()
        val outB = DoubleArrayList()
        val outData = IntArrayList()

        tree.overlaps(
            minX = activeBody.tightBB.minX,
            minY = activeBody.tightBB.minY,
            minZ = activeBody.tightBB.minZ,
            maxX = activeBody.tightBB.maxX,
            maxY = activeBody.tightBB.maxY,
            maxZ = activeBody.tightBB.maxZ,
            outA = outA,
            outB = outB,
            outData = outData,
        )

        if (outA.isEmpty) return

        val vertices = activeBody.vertices

        val axis = tree.axis

        if (!setupCrossAxiss(axis.vec)) return

        val aItr = outA.iterator()
        val bItr = outB.iterator()
        val dataItr = outData.iterator()

        while (aItr.hasNext()) {
            val a = aItr.nextDouble()
            val b = bItr.nextDouble()
            val data = dataItr.nextInt()

            _edgeStart.setComponent(axis.aOffset, a)
            _edgeStart.setComponent(axis.bOffset, b)

            _edgeEnd.setComponent(axis.aOffset, a)
            _edgeEnd.setComponent(axis.bOffset, b)

            val pts = tree.points[data]
            val mounts = tree.pointMounts[data]

            val itr = pts.iterator()
            var i = 0
            while (itr.hasNext()) {
                val d1 = itr.nextDouble()
                val d2 = itr.nextDouble()

                val mount = mounts.getInt(i)

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

                check(true) {
                    """
                    Edge Contact:
                    | norm: ${r.norm}
                    | point: ${r.pointA}
                    | allowedNormal[0]: ${_allowedNormals[0]}
                    | allowedNormal[1]: ${_allowedNormals[1]}
                    | mount: $mount
                """.trimIndent()
                }

                out += Contact(
                    first = activeBody,
                    second = other,
                    result = r,
                )

                i++
            }
        }

//        for (edge in edges) {
//            /*
//            edge is represnted by a sorted list of doubles, so we jump 2 at a time through them and those are our edge starts and ends
//                might eventually be worth it to convert this into a sorted set to do O(log n) traversal instead of O(n)
//             */
//            _edgeStart.setComponent(axis.aOffset, edge.a)
//            _edgeStart.setComponent(axis.bOffset, edge.b)
//
//            _edgeEnd.setComponent(axis.aOffset, edge.a)
//            _edgeEnd.setComponent(axis.bOffset, edge.b)
//
////            println("POINTS: ${edge.points.joinToString { it.toString() }}")
//            val itr = edge.points.iterator()
//            var i = 0
//            while (itr.hasNext()) {
//                val d1 = itr.nextDouble()
//                val d2 = itr.nextDouble()
//
//                val mount = edge.pointMounts.getInt(i)
//
//                _allowedNormals[0].setComponent(axis.levelOffset, 0.0)
//                _allowedNormals[0].setComponent(axis.aOffset, EdgeMountAllowedNormals.allowedNormals[mount][0].x)
//                _allowedNormals[0].setComponent(axis.bOffset, EdgeMountAllowedNormals.allowedNormals[mount][0].y)
//
//                _allowedNormals[1].setComponent(axis.levelOffset, 0.0)
//                _allowedNormals[1].setComponent(axis.aOffset, EdgeMountAllowedNormals.allowedNormals[mount][1].x)
//                _allowedNormals[1].setComponent(axis.bOffset, EdgeMountAllowedNormals.allowedNormals[mount][1].y)
//
//                _edgeStart.setComponent(axis.levelOffset, d1)
//                _edgeEnd.setComponent(axis.levelOffset, d2)
//
//                val r = math.collideCuboidEdge(
//                    edgeStart = _edgeStart,
//                    edgeEnd = _edgeEnd,
//                    bodyAxiss = _bodyAxiss,
//                    crossAxiss = _crossAxiss,
//                    vertices = vertices,
//                    allowedNormals = _allowedNormals,
//                )
//
//                if (r == null) {
//                    i++
//                    continue
//                }
//
////                println("COLLIDED EDGE")
////                println("  - norm: ${r.norm}")
////                println("  - point: ${r.pointA}")
////                println("  - allowedNormals[0]: ${_allowedNormals[0]}")
////                println("  - allowedNormals[1]: ${_allowedNormals[1]}")
////                println("  - mount: $mount")
//
//                check(true) {
//                    """
//                    Edge Contact:
//                    | norm: ${r.norm}
//                    | point: ${r.pointA}
//                    | allowedNormal[0]: ${_allowedNormals[0]}
//                    | allowedNormal[1]: ${_allowedNormals[1]}
//                    | mount: $mount
//                """.trimIndent()
//                }
//
//                out += Contact(
//                    first = activeBody,
//                    second = other,
//                    result = r,
//                )
//
//                i++
//            }
//        }
    }
}

fun contains(
    minA: Double,
    minB: Double,
    maxA: Double,
    maxB: Double,

    a: Double,
    b: Double,
): Boolean {
    return minA <= a && maxA >= a && minB <= b && maxB >= b
}