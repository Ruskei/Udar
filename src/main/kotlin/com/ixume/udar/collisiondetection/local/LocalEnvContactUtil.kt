package com.ixume.udar.collisiondetection.local

import com.ixume.udar.body.Body
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D
import com.ixume.udar.collisiondetection.mesh.mesh2.EdgeMountAllowedNormals
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFace
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.physics.contact.A2SContactArray
import com.ixume.udar.physics.contact.A2SContactBuffer
import com.ixume.udar.physics.contact.A2SContactCollection
import com.ixume.udar.physics.contact.A2SManifoldArray
import com.ixume.udar.physics.contact.A2SManifoldCollection
import com.ixume.udar.physics.contact.ContactDataBuffer
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList
import org.joml.Vector3d

class LocalEnvContactUtil(val math: LocalMathUtil) {
    private val _mf = mutableListOf<MeshFace>()

    private val _contacts2 = ContactDataBuffer(8)
    private val _validContacts2 = ContactDataBuffer(8)

    // TODO: manifold for edge contacts
    fun collides(
        contactGen: EnvironmentContactGenerator2,
        activeBody: ActiveBody,
        other: Body,
        out: A2SManifoldCollection,
    ): Boolean {
        val bb = activeBody.tightBB

        activeBody.physicsWorld.worldMeshesManager.request(
            prevBB = contactGen.prevBB,
            currentBB = bb,
            envContactGenerator = contactGen,
        )

        val mfs = contactGen.meshFaces.get()
        val mes = contactGen.meshEdges.get()

        var j = 0
        while (j < mfs.size) {
            val mf = mfs[j]
            _mf.clear()
            mf.facesIn(bb, _mf)
            collideFaces(activeBody, _mf, mf.axis, math, other, activeBody.physicsWorld.envManifoldBuffer)
            j++
        }

        setupBodyAxiss(activeBody)

        var i = 0
        while (i < mes.size) {
            val me = mes[i]
            
            collideEdges(
                activeBody = activeBody,
                tree = me,
                math = math,
                other = other,
                out = out,
            )
            i++
        }

        return false
    }

    private val _bb2d = AABB2D(doubleArrayOf(0.0, 0.0, 0.0, 0.0))
    private val _overlappingHoles = DoubleArrayList()
    private val _overlappingAntiHoles = DoubleArrayList()

    private fun collideFaces(
        activeBody: ActiveBody,
        faces: List<MeshFace>,
        axis: LocalMesher.AxisD,
        math: LocalMathUtil,
        other: Body,
        out: A2SManifoldCollection,
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
        
        
        var count = 0

        var i = 0
        while (i < faces.size) {
            _contacts2.clear()
            _validContacts2.clear()
            val face = faces[i]
            val any = math.collidePlane(
                first = activeBody,
                axis = axis,
                level = face.level,
                vertices = vertices,
                out = _contacts2,
            )

            if (!any) {
                i++
                continue
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
                i++
                continue
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

            var j = 0
            while (j < _contacts2.size()) {
                val pa = _contacts2.pointAComponent(j, axis.aOffset).toDouble()

                var valid = true
                var k = 0
                check(_overlappingAntiHoles.size % 4 == 0)
                while (k < _overlappingAntiHoles.size / 4) {
                    val minA = _overlappingAntiHoles.getDouble(k * 4)
                    val maxA = _overlappingAntiHoles.getDouble(k * 4 + 2)

                    if (contains(
                            minA = minA,
                            maxA = maxA,
                            a = pa,
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
                    val maxA = _overlappingHoles.getDouble(l * 4 + 2)

                    if (contains(
                            minA = minA,
                            maxA = maxA,
                            a = pa,
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

                _contacts2.loadInto(count, _validContacts2)
//                _contacts.transferToBuffer(
//                    contactIdx = j,
//                    buffer = out,
//                    activeBody = activeBody,
//                )
                
                count++
                j++
            }

            i++
        }
        
        out.addManifold(
            activeBody = activeBody,
            contactID = 0L,
            buf = _validContacts2,
        )
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
    private val _outData = IntArrayList()

    private fun collideEdges(
        activeBody: ActiveBody,
        tree: FlattenedEdgeQuadtree,
        math: LocalMathUtil,
        other: Body,
        out: A2SManifoldCollection,
    ): Boolean {
        /*
        get all edges that we could possibly be colliding with; we could have a valid collision with each of these edges (not using discrete curvature graph rn)
         */

        _outA.clear()
        _outB.clear()
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
            outData = _outData,
            math = math,
        )

        if (_outA.isEmpty) return false

        val vertices = activeBody.vertices

        val axis = tree.axis

        if (!setupCrossAxiss(axis.vec)) return false

        val aItr = _outA.iterator()
        val bItr = _outB.iterator()
        val dataItr = _outData.iterator()

        var collided = false

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

                collided = true

                math.collideCuboidEdge(
                    activeBody = activeBody,
                    edgeStart = _edgeStart,
                    edgeEnd = _edgeEnd,
                    bodyAxiss = _bodyAxiss,
                    crossAxiss = _crossAxiss,
                    vertices = vertices,
                    allowedNormals = _allowedNormals,
                    edges = activeBody.edges,
                    out = out,
                )

                i++
            }
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
