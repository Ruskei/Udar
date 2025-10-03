package com.ixume.udar.collisiondetection.local

import com.ixume.udar.body.Body
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D
import com.ixume.udar.collisiondetection.mesh.mesh2.EdgeMountAllowedNormals
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaceSortedList
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.dynamicaabb.array.FlattenedAABBTree
import com.ixume.udar.physics.contact.a2s.A2SContactDataBuffer
import com.ixume.udar.physics.contact.a2s.EnvManifoldBuffer
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldCollection
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList
import org.joml.Vector3d

class LocalEnvContactUtil(val math: LocalMathUtil) {
    private val _contacts2 = A2SContactDataBuffer(8)
    private val _possibleManifolds = EnvManifoldBuffer(8)
    private val _validContacts2 = A2SContactDataBuffer(8)

    fun collides(
        contactGen: EnvironmentContactGenerator2,
        activeBody: ActiveBody,
        other: Body,
        out: A2SManifoldCollection,
    ): Boolean {
        setupBodyAxiss(activeBody)

        val meshes = contactGen.meshes.getMeshes()

//        println("ENV COLLISION CHECK")

        val minX = activeBody.tightBB.minX - 1.0
        val minY = activeBody.tightBB.minY - 1.0
        val minZ = activeBody.tightBB.minZ - 1.0

        val maxX = activeBody.tightBB.maxX + 1.0
        val maxY = activeBody.tightBB.maxY + 1.0
        val maxZ = activeBody.tightBB.maxZ + 1.0

        var m = 0
        while (m < meshes.size) {
//            println("- MESH")
            val mesh = meshes[m]
            if (!mesh.relevant(minX, minY, minZ, maxX, maxY, maxZ)) {
                m++
                continue
            }

            val faces = mesh.faces
            val bbs = mesh.flatTree
            if (bbs == null) {
                m++
                continue
            }

            _possibleManifolds.setup(mesh)
            _possibleManifolds.clear()

            if (faces != null) {
//                println(" - TESTING MESH FACES")
                collideFaces(activeBody, faces.xFaces, bbs, LocalMesher.AxisD.X, math, mesh)
                collideFaces(activeBody, faces.yFaces, bbs, LocalMesher.AxisD.Y, math, mesh)
                collideFaces(activeBody, faces.zFaces, bbs, LocalMesher.AxisD.Z, math, mesh)
            }

            mesh.xEdges2?.let {
                collideEdges(
                    activeBody = activeBody,
                    tree = it,
                    math = math,
                    mesh = mesh,
                )
            }

            mesh.yEdges2?.let {
                collideEdges(
                    activeBody = activeBody,
                    tree = it,
                    math = math,
                    mesh = mesh,
                )
            }

            mesh.zEdges2?.let {
                collideEdges(
                    activeBody = activeBody,
                    tree = it,
                    math = math,
                    mesh = mesh,
                )
            }

            _possibleManifolds.post(out)

            m++
        }


        return false
    }

    private val _bb2d = AABB2D(doubleArrayOf(0.0, 0.0, 0.0, 0.0))
    private val _overlappingHoles = DoubleArrayList()
    private val _overlappingAntiHoles = DoubleArrayList()

    private fun collideFaces(
        activeBody: ActiveBody,
        faces: MeshFaceSortedList,
        bbs: FlattenedAABBTree,
        axis: LocalMesher.AxisD,
        math: LocalMathUtil,
        mesh: LocalMesher.Mesh2,
    ) {
//        println("COLLIDING $axis FACES")
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

        var count = 0

        faces.forEachOverlapping(bb) { face ->
            _contacts2.clear()
            _validContacts2.clear()

            val manifoldID = constructA2SFaceManifoldID(
                body = activeBody,
                faceAxis = axis,
                faceLevel = face.level,
                mesh = mesh,
            )

            val rawManifoldIdx = activeBody.physicsWorld.prevEnvContactMap.get(manifoldID)

            count++

            val any = math.collidePlane(
                axis = axis,
                face = face,
                faces = faces,
                vertices = vertices,
                out = _contacts2,
                rawManifoldIdx = rawManifoldIdx,
                bb = bb,
            )

            if (!any) {
                return@forEachOverlapping
            }

            _overlappingHoles.clear()

            face.holes.overlaps(
                minX = _bb2d.minA(),
                minY = _bb2d.minB(),
                maxX = _bb2d.maxA(),
                maxY = _bb2d.maxB(),
                out = _overlappingHoles,
                math = math,
            )

            if (_overlappingHoles.isEmpty()) {
                return@forEachOverlapping
            }

            _overlappingAntiHoles.clear()

            face.antiHoles.overlaps(
                minX = _bb2d.minA(),
                minY = _bb2d.minB(),
                maxX = _bb2d.maxA(),
                maxY = _bb2d.maxB(),
                out = _overlappingAntiHoles,
                math = math,
            )

//            println("VALIDATING")

            var j = 0
            while (j < _contacts2.size()) {
                val pa = _contacts2.pointAComponent(j, axis.aOffset).toDouble()
                val pb = _contacts2.pointAComponent(j, axis.bOffset).toDouble()
                val plevel = _contacts2.pointAComponent(j, axis.levelOffset).toDouble()

                var x = -Double.MAX_VALUE
                var y = -Double.MAX_VALUE
                var z = -Double.MAX_VALUE

                when (axis.aOffset) {
                    0 -> x = pa
                    1 -> y = pa
                    2 -> z = pa
                }

                when (axis.bOffset) {
                    0 -> x = pb
                    1 -> y = pb
                    2 -> z = pb
                }

                when (axis.levelOffset) {
                    0 -> x = plevel
                    1 -> y = plevel
                    2 -> z = plevel
                }

                check(x != -Double.MAX_VALUE)
                check(y != -Double.MAX_VALUE)
                check(z != -Double.MAX_VALUE)

                var valid = true
                var k = 0
                check(_overlappingAntiHoles.size % 4 == 0)
                while (k < _overlappingAntiHoles.size / 4) {
                    val minA = _overlappingAntiHoles.getDouble(k * 4)
                    val minB = _overlappingAntiHoles.getDouble(k * 4 + 1)
                    val maxA = _overlappingAntiHoles.getDouble(k * 4 + 2)
                    val maxB = _overlappingAntiHoles.getDouble(k * 4 + 3)

                    if (contains(
                            minA = minA,
                            maxA = maxA,
                            a = pa,
                        ) && contains(
                            minA = minB,
                            maxA = maxB,
                            a = pb,
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
                check(_overlappingHoles.size % 4 == 0)
                while (l < _overlappingHoles.size / 4) {
                    val minA = _overlappingHoles.getDouble(l * 4)
                    val minB = _overlappingHoles.getDouble(l * 4 + 1)
                    val maxA = _overlappingHoles.getDouble(l * 4 + 2)
                    val maxB = _overlappingHoles.getDouble(l * 4 + 3)

                    if (contains(
                            minA = minA,
                            maxA = maxA,
                            a = pa,
                        ) && contains(
                            minA = minB,
                            maxA = maxB,
                            a = pb,
                        )
                    ) {
//                        println("  * CONTAINED IN HOLE:")
//                        println("  | min: ($minA $minB)")
//                        println("  | max: ($maxA $maxB)")
                        valid = true
                        break
                    }

                    l++
                }

                if (!valid) {
//                    Println("  - REJECTED: NO HOLE")
                    j++
                    continue
                }

                if (!bbs.contains(x, y, z)) {
//                    println("  - REJECTED: NOT CONTAINED")
                    j++
                    continue
                }

                _contacts2.loadInto(j, _validContacts2)

                j++
            }

            if (_validContacts2.size() > 0) {
                _possibleManifolds.addFaceManifold(
                    activeBody = activeBody,
                    contactID = manifoldID,
                    buf = _validContacts2,
                    face = face,
                )
            }
        }

//        println("  - Tested $count $axis faces!")
    }

    private fun constructA2SFaceManifoldID(
        body: ActiveBody,
        faceAxis: LocalMesher.AxisD,
        faceLevel: Double,
        mesh: LocalMesher.Mesh2,
    ): Long {
        var result = 31L

        val prime = 31L

        val mostSigBits = body.uuid.mostSignificantBits
        val leastSigBits = body.uuid.leastSignificantBits
        result = result * prime + mostSigBits
        result = result * prime + leastSigBits

        result = result * prime + faceAxis.ordinal

        val doubleBits = faceLevel.toRawBits()
        result = result * prime + doubleBits

        result = result * prime + mesh.hashCode()

        return result
    }

    private val _bodyAxiss = Array(3) { Vector3d() }
    private val _crossAxiss = Array(3) { Vector3d() }

    private fun setupBodyAxiss(activeBody: ActiveBody) {
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

    private val _outA = DoubleArrayList()
    private val _outB = DoubleArrayList()
    private val _outEdges = IntArrayList()
    private val _outData = IntArrayList()

    private fun collideEdges(
        activeBody: ActiveBody,
        tree: FlattenedEdgeQuadtree,
        math: LocalMathUtil,
        mesh: LocalMesher.Mesh2,
    ): Boolean {
        /*
        get all edges that we could possibly be colliding with; we could have a valid collision with each of these edges (not using discrete curvature graph rn)
         */

        _outA.clear()
        _outB.clear()
        _outEdges.clear()
        _outData.clear()

        tree.overlaps(
            minX = activeBody.tightBB.minX,
            minY = activeBody.tightBB.minY,
            minZ = activeBody.tightBB.minZ,

            maxX = activeBody.tightBB.maxX,
            maxY = activeBody.tightBB.maxY,
            maxZ = activeBody.tightBB.maxZ,

            outA = _outA,
            outB = _outB,
            outEdges = _outEdges,
            outData = _outData,
            math = math,
        )

        if (_outA.isEmpty) return false

        val vertices = activeBody.vertices

        val axis = tree.axis

        if (!setupCrossAxiss(axis.vec)) return false

        val s = _outA.size

        var idx = 0

        var collided = false

        while (idx < s) {
            val a = _outA.getDouble(idx)
            val b = _outB.getDouble(idx)
            val data = _outData.getInt(idx)

            _edgeStart.setComponent(axis.aOffset, a)
            _edgeStart.setComponent(axis.bOffset, b)

            _edgeEnd.setComponent(axis.aOffset, a)
            _edgeEnd.setComponent(axis.bOffset, b)

            val pts = tree.points[data]
            val mounts = tree.pointMounts[data]

            val itr = pts.doubleIterator()
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

                collided = true

                math.collideEdge(
                    activeBody = activeBody,
                    edgeStart = _edgeStart,
                    edgeEnd = _edgeEnd,
                    bodyAxiss = _bodyAxiss,
                    crossAxiss = _crossAxiss,
                    vertices = vertices,
                    allowedNormals = _allowedNormals,
                    edges = activeBody.edges,
                    out = _possibleManifolds,
                    mesh = mesh,
                )

                i++
            }

            idx++
        }

        return collided
    }
}

fun contains(
    minA: Double,
    maxA: Double,

    a: Double,
): Boolean {
    return minA <= a && maxA >= a
}
