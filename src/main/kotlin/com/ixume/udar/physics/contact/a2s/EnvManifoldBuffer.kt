package com.ixume.udar.physics.contact.a2s

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFace
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldArray
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldCollection
import it.unimi.dsi.fastutil.ints.IntArrayList
import it.unimi.dsi.fastutil.longs.*

class EnvManifoldBuffer(numContacts: Int) {
    /*
    construct a graph of edge and face manifolds
    after collision detection:
        for every face contact:
            only keep if it's the smallest of its neighbors ( no more transitivity )
     */

    val buffer = A2SManifoldArray(numContacts)

    private lateinit var xEdges: FlattenedEdgeQuadtree
    private lateinit var yEdges: FlattenedEdgeQuadtree
    private lateinit var zEdges: FlattenedEdgeQuadtree
    val running = Long2ObjectOpenHashMap<ManifoldNode>()
    val idxMap = Long2IntOpenHashMap()
    val edgeManifolds = IntArrayList()

    fun setup(mesh: LocalMesher.Mesh2) {
        this.xEdges = mesh.xEdges2!!
        this.yEdges = mesh.yEdges2!!
        this.zEdges = mesh.zEdges2!!
    }

    init {
        idxMap.defaultReturnValue(-1)
        running.defaultReturnValue(null)
    }

    private val _connections = LongArrayList()

    fun addFaceManifold(
        activeBody: ActiveBody,
        contactID: Long,
        buf: A2SContactDataBuffer,
        face: MeshFace,
    ) {
//        println("ADDING FACE MANIFOLD (${face.id})")
        var existing = running.get(face.id)
        if (existing == null) {
            existing = ManifoldNode(LongOpenHashSet(), false)
            running.put(face.id, existing)
        } else {
            existing.isEdge = false
        }

        val minX = buf.minX()
        val minY = buf.minY()
        val minZ = buf.minZ()

        val maxX = buf.maxX()
        val maxY = buf.maxY()
        val maxZ = buf.maxZ()

        _connections.clear()

        // find all overlapping edges
        var i = 0
        while (i < face.edgeConnections.size) {
            val conn = face.edgeConnections[i]
//            println("  - testing edge connection with ${conn.otherFaceID}, axis: ${conn.tree.axis}")
            when (conn.tree.axis) {
                LocalMesher.AxisD.X -> {
                    if (maxX >= conn.min && minX <= conn.max) {
//                        println("  * overlapping with ${conn.myFaceID}<->${conn.otherFaceID}, bounds: ${conn.min}<->${conn.max}!")
                        _connections.add(conn.otherFaceID)
                    }
                }

                LocalMesher.AxisD.Y -> {
                    if (maxY >= conn.min && minY <= conn.max) {
//                        println("  * overlapping with ${conn.myFaceID}<->${conn.otherFaceID}, bounds: ${conn.min}<->${conn.max}!")
                        _connections.add(conn.otherFaceID)
                    }
                }

                LocalMesher.AxisD.Z -> {
                    if (maxZ >= conn.min && minZ <= conn.max) {
//                        println("  * overlapping with ${conn.myFaceID}<->${conn.otherFaceID}, bounds: ${conn.min}<->${conn.max}!")
                        _connections.add(conn.otherFaceID)
                    }
                }
            }

            i++
        }

        // for every one in connected, if it exists in running, add it to mine, and add myself to theirs
        val connectedItr = _connections.longIterator()
        while (connectedItr.hasNext()) {
            val l = connectedItr.nextLong()
            val present = running.get(l)
            if (present != null) {
//                println("  - connected face with existing $l")
                existing.set.add(l)
                present.set.add(face.id)
            }
        }

//        println("  - adding face(${face.id}) as ${buffer.cursor}")
        idxMap.put(face.id, buffer.cursor)
        buffer.addManifold(activeBody, contactID, buf)
    }

    fun addEdgeManifold(
        activeBody: ActiveBody,
        contactID: Long,

        pointAX: Float,
        pointAY: Float,
        pointAZ: Float,

        normX: Float,
        normY: Float,
        normZ: Float,

        depth: Float,
        math: LocalMathUtil,

        normalLambda: Float,
        t1Lambda: Float,
        t2Lambda: Float,
    ) {
        edgeManifolds.add(buffer.cursor)
        buffer.addSingleManifold(
            activeBody = activeBody,
            contactID = contactID,
            pointAX = pointAX,
            pointAY = pointAY,
            pointAZ = pointAZ,

            normX = normX,
            normY = normY,
            normZ = normZ,

            depth = depth,
            math = math,

            normalLambda = normalLambda,
            t1Lambda = t1Lambda,
            t2Lambda = t2Lambda,
        )
    }

    fun clear() {
        buffer.clear()
        idxMap.clear()
        edgeManifolds.clear()
        val itr = Long2ObjectMaps.fastIterator(running)
        while (itr.hasNext()) {
            val e = itr.next()
            e.value.set.clear()
        }
    }

    fun post(out: A2SManifoldCollection) {
//        println("POST")
        // traverse idx map, find each node; if it's a face node and it's not the smallest of its neighbors, don't add it; otherwise, add it
        val idxItr = Long2IntMaps.fastIterator(idxMap)
        while (idxItr.hasNext()) {
            val entry = idxItr.next()
            val node = running.get(entry.longKey)!!
            val idx = entry.intValue
            if (node.isEdge) {
//                println("  - loaded edge $idx")
                out.load(buffer, idx)
            } else {
                var valid = true
                val myDepth = buffer.maxDepth(idx)
                val adjacentItr = node.set.longIterator()
                while (adjacentItr.hasNext()) {
                    val l = adjacentItr.nextLong()
                    val adj = idxMap.get(l)
                    if (adj == -1) continue

                    val adjDepth = buffer.maxDepth(adj)
                    if (adjDepth < myDepth) {
//                        println("  - failed to load face $idx, usurped by $adj")
                        valid = false
                        break
                    }
                }

                if (valid) {
//                    println("  - loaded face $idx")
                    out.load(buffer, idx)
                }
            }
        }

        var i = 0
        while (i < edgeManifolds.size) {
//            println("  - loaded edge $i")
            out.load(buffer, edgeManifolds.getInt(i))

            i++
        }
    }
}

class ManifoldNode(
    val set: LongSet,
    var isEdge: Boolean,
)