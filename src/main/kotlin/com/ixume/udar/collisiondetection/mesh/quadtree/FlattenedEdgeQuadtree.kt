package com.ixume.udar.collisiondetection.mesh.quadtree

import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaces
import com.ixume.udar.collisiondetection.mesh.mesh2.axiss
import com.ixume.udar.collisiondetection.mesh.mesh2.faceID
import com.ixume.udar.dynamicaabb.array.FlattenedAABBTree
import com.ixume.udar.dynamicaabb.array.IntStack
import com.ixume.udar.dynamicaabb.array.withHigher
import com.ixume.udar.dynamicaabb.array.withLower
import com.ixume.udar.testing.debugConnect
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d
import kotlin.math.max
import kotlin.math.min

/**
 * Since this structure's leaves contain a variable amount of data that cannot be reasonably put into a fixed size, we store indices to the data instead
 *
 * Center can be calculated quickly, don't waste space on it
 *
 * Have 3 ArrayLists of data:
 *  - _points: DoubleAVLTreeSet
 *  - xoredPoints: DoubleAVLTreeSet
 *  - pointMounts: IntArrayList
 *
 * All of these are referenced by the same index, so we can just use one value for that
 *
 * If it's a leaf, minA, and minB are interpreted as `a` and `b`
 *
 * Store the nodes and edges in separate arrays, since they don't really share much data and have a large difference in struct size
 *
 * node {
 *     isLeaf: Boolean,                        0
 *     numPoints: Int,                         0
 *
 *     minA: Double,                           1
 *     minB: Double,                           2
 *     maxA: Double,                           3
 *     maxB, Double,                           4
 *
 *     child1: Int,                            5
 *     child2: Int,                            5
 *     child3: Int,                            6
 *     child4: Int,                            6
 *
 *     edge1: Long,_                          7
 *     edge2: Long, \  MAX_POINTS_PER_NODE
 *     ...           /
 *     edgeN: Long,/
 * }
 *
 * edge {
 *     f1: Int,    0
 *     f2: Int,    0
 *
 *     a: Double,  1
 *     b: Double,  2
 *
 *     idx: Long,  3
 * }
 */
class FlattenedEdgeQuadtree(
    val axis: LocalMesher.AxisD,

    minA: Double,
    minB: Double,

    maxA: Double,
    maxB: Double,

    val levelMin: Double,
    val levelMax: Double,
) {
    val edgeData = mutableListOf<EdgeData>()

    private var nodeFreeIdx = Node(-1) // head of node free linked list
    private var edgeFreeIdx = Edge(-1) // head of edge free linked list

    private var rootIdx = Node(-1)

    private var nodeArr = DoubleArray(0)
    private var edgeArr = DoubleArray(0)

    init {
        rootIdx = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = minA,
            minB = minB,
            maxA = maxA,
            maxB = maxB,

            c1 = Node(-1),
            c2 = Node(-1),
            c3 = Node(-1),
            c4 = Node(-1),
        )
    }

    fun insertEdge(x: Double, y: Double, start: Double, end: Double, meshFaces: MeshFaces) {
        _insertEdge(x, y, start, end - ASYMMETRY_EPSILON, meshFaces)
    }

    private val fixupQueue = IntStack()

    private val _vec3Mounts = DoubleArray(12)

    fun fixUp(tree: FlattenedAABBTree, faces: MeshFaces) {
        if (rootIdx.idx == -1) return

        fixupQueue.enqueue(rootIdx.idx)

        while (fixupQueue.hasNext()) {
            val node = Node(fixupQueue.dequeue())

            if (node.isLeaf()) {
                var i = 0
                val n = node.numPoints()

                while (i < n) {
                    val pts = DoubleArrayList()
                    val ptMounts = IntArrayList()

                    val edge = node.edge(i)
                    val dataIdx = edge.dataIdx()
                    val ea = edge.data(dataIdx)
                    val _pts = ea._points
                    _pts.sort()
                    val edgePts = ea.points

                    val s = _pts.size
                    check(s % 2 == 0)

                    val a = edge.a()
                    val b = edge.b()

                    _vec3Mounts[axis.aOffset] = a + MOUNT_EPSILON
                    _vec3Mounts[axis.bOffset] = b + MOUNT_EPSILON

                    _vec3Mounts[3 + axis.aOffset] = a + MOUNT_EPSILON
                    _vec3Mounts[3 + axis.bOffset] = b - MOUNT_EPSILON

                    _vec3Mounts[6 + axis.aOffset] = a - MOUNT_EPSILON
                    _vec3Mounts[6 + axis.bOffset] = b - MOUNT_EPSILON

                    _vec3Mounts[9 + axis.aOffset] = a - MOUNT_EPSILON
                    _vec3Mounts[9 + axis.bOffset] = b + MOUNT_EPSILON

                    val lvlOffset = axis.levelOffset

                    val itr = _pts.iterator()

                    while (itr.hasNext()) {
                        var mounted = false
                        var mount = -1

                        val d0 = min(max(itr.nextDouble(), levelMin), levelMax)
                        val d1 = min(max(itr.nextDouble(), levelMin), levelMax)

                        if (d1 <= d0) continue

                        val center = d0 * 0.5 + d1 * 0.5

                        var j = 0
                        while (j < 4) {
                            _vec3Mounts[j * 3 + lvlOffset] = center
                            if (tree.contains(_vec3Mounts[j * 3], _vec3Mounts[j * 3 + 1], _vec3Mounts[j * 3 + 2])) {
                                if (mounted) {
                                    // concave...
                                    mounted = false
                                    break
                                } else {
                                    mounted = true
                                    mount = j
                                }
                            }

                            j++
                        }

                        if (mounted) {
                            pts.add(d0)
                            pts.add(d1)

                            ptMounts.add(mount)
                        }
                    }

                    if (pts.isEmpty) {
                        i++
                        ea.finalizePoints()
                        continue
                    }

                    check(pts.size % 2 == 0)
                    check(ptMounts.isNotEmpty())
                    check(ptMounts.size * 2 == pts.size)

                    val edgeMounts = ea.pointMounts

                    var currentStart = pts.getDouble(0)
                    var latestEnd: Double = pts.getDouble(1)
                    var currentEdgeMount = ptMounts.getInt(0)

                    var j = 1
                    while (j < ptMounts.size) {
                        val s = pts.getDouble(j * 2)
                        check(s > latestEnd)
                        val m = ptMounts.getInt(j)

                        if (m == currentEdgeMount && s - latestEnd < ABOVE_ASYMMETRY_EPSILON) {
                            latestEnd = pts.getDouble(j * 2 + 1)
                        } else {
                            edgePts.add(currentStart)
                            edgePts.add(latestEnd)
                            edgeMounts.add(currentEdgeMount)

                            val e = pts.getDouble(j * 2 + 1)

                            currentStart = s
                            latestEnd = e
                            currentEdgeMount = m
                        }

                        j++
                    }

                    edgePts.add(currentStart)
                    edgePts.add(latestEnd)
                    ea.finalizePoints()
                    edgeMounts.add(currentEdgeMount)

                    check(edgePts.size % 2 == 0)
                    // create the graph
                    val completedItr = edgePts.doubleIterator()
                    while (completedItr.hasNext()) {
                        val s = completedItr.nextDouble()
                        val e = completedItr.nextDouble()
                        check(e > s)

                        val aFaces = when (axis) {
                            LocalMesher.AxisD.X -> faces.yFaces
                            LocalMesher.AxisD.Y -> faces.xFaces
                            LocalMesher.AxisD.Z -> faces.xFaces
                        }

                        val f1 = aFaces.getFaceIdxAt(a)

                        check(f1 != -1)

                        val bFaces = when (axis) {
                            LocalMesher.AxisD.X -> faces.zFaces
                            LocalMesher.AxisD.Y -> faces.zFaces
                            LocalMesher.AxisD.Z -> faces.yFaces
                        }

                        val f2 = bFaces.getFaceIdxAt(b)

                        check(f2 != -1)

                        val aFID = faceID(aFaces.axis, f1)
                        val bFID = faceID(bFaces.axis, f2)

                        aFaces.ls[f1].edgeConnections += EdgeConnection(
                            tree = this,
                            otherFaceID = bFID,
                            min = s,
                            max = e,
                        )

                        bFaces.ls[f2].edgeConnections += EdgeConnection(
                            tree = this,
                            otherFaceID = aFID,
                            min = s,
                            max = e,
                        )
                    }

                    i++
                }
            } else {
                fixupQueue.enqueue(node.child1().idx)
                fixupQueue.enqueue(node.child2().idx)
                fixupQueue.enqueue(node.child3().idx)
                fixupQueue.enqueue(node.child4().idx)
            }
        }
    }

    fun overlaps(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,

        outA: DoubleArrayList,
        outB: DoubleArrayList,
        outEdges: IntArrayList,
        outData: IntArrayList,

        math: LocalMathUtil,
    ) {
        if (rootIdx.idx == -1) return

        rootIdx._overlaps(minX, minY, minZ, maxX, maxY, maxZ, outA, outB, outEdges, outData, math)
    }

    private fun Node._overlaps(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,

        outA: DoubleArrayList,
        outB: DoubleArrayList,
        outEdges: IntArrayList,
        outData: IntArrayList,

        math: LocalMathUtil,
    ) {
        when (axis) {
            LocalMesher.AxisD.X -> {
                if (!overlaps(minY, minZ, maxY, maxZ)) return

                if (isLeaf()) {
                    val np = numPoints()
                    var i = 0
                    while (i < np) {
                        val e = edge(i)
                        val a = e.a()
                        val b = e.b()

                        if (a >= minY && a < maxY &&
                            b >= minZ && b < maxZ
                        ) {
                            outA.add(a)
                            outB.add(b)
                            outEdges.add(e.idx)
                            outData.add(e.dataIdx())
                        }

                        i++
                    }

                    return
                }
            }

            LocalMesher.AxisD.Y -> {
                if (!overlaps(minX, minZ, maxX, maxZ)) return

                if (isLeaf()) {
                    val np = numPoints()
                    var i = 0
                    while (i < np) {
                        val e = edge(i)
                        val a = e.a()
                        val b = e.b()

                        if (a >= minX && a < maxX &&
                            b >= minZ && b < maxZ
                        ) {
                            outA.add(a)
                            outB.add(b)
                            outEdges.add(e.idx)
                            outData.add(e.dataIdx())
                        }

                        i++
                    }

                    return
                }
            }

            LocalMesher.AxisD.Z -> {
                if (!overlaps(minX, minY, maxX, maxY)) return

                if (isLeaf()) {
                    val np = numPoints()
                    var i = 0
                    while (i < np) {
                        val e = edge(i)
                        val a = e.a()
                        val b = e.b()

                        if (a >= minX && a < maxX &&
                            b >= minY && b < maxY
                        ) {
                            outA.add(a)
                            outB.add(b)
                            outEdges.add(e.idx)
                            outData.add(e.dataIdx())
                        }

                        i++
                    }

                    return
                }
            }
        }

        child1()._overlaps(minX, minY, minZ, maxX, maxY, maxZ, outA, outB, outEdges, outData, math)
        child2()._overlaps(minX, minY, minZ, maxX, maxY, maxZ, outA, outB, outEdges, outData, math)
        child3()._overlaps(minX, minY, minZ, maxX, maxY, maxZ, outA, outB, outEdges, outData, math)
        child4()._overlaps(minX, minY, minZ, maxX, maxY, maxZ, outA, outB, outEdges, outData, math)
    }

    private val edgeInsertionQueue = IntStack()

    private fun _insertEdge(
        a: Double,
        b: Double,
        start: Double,
        end: Double,
        meshFaces: MeshFaces,
    ): Boolean {
        edgeInsertionQueue.enqueue(rootIdx.idx)

        while (edgeInsertionQueue.hasNext()) {
            val node = Node(edgeInsertionQueue.dequeue())

            if (!node.contains(a, b)) continue

            if (node.isLeaf()) {
                var i = 0
                val np = node.numPoints()
                while (i < np) {
                    val edge = node.edge(i)
                    val na = edge.a()
                    val nb = edge.b()

                    if (na == a && nb == b) { // found matching edge !
                        edge.xor(start, end, edge.dataIdx())

                        return true
                    }

                    i++
                }

                // didn't find a matching edge, so we have to create a new point or subdivide

                val numPts = node.numPoints()
                if (numPts < MAX_POINTS_PER_NODE) {
                    val aFaces = when (axis) {
                        LocalMesher.AxisD.X -> meshFaces.yFaces
                        LocalMesher.AxisD.Y -> meshFaces.xFaces
                        LocalMesher.AxisD.Z -> meshFaces.xFaces
                    }

                    val f1 = aFaces.getFaceIdxAt(a)

                    check(f1 != -1)

                    val bFaces = when (axis) {
                        LocalMesher.AxisD.X -> meshFaces.zFaces
                        LocalMesher.AxisD.Y -> meshFaces.zFaces
                        LocalMesher.AxisD.Z -> meshFaces.yFaces
                    }

                    val f2 = bFaces.getFaceIdxAt(b)

                    check(f2 != -1)

                    val idx = newDataIdx()

                    val ne = newEdge(
                        f1 = f1,
                        f2 = f2,

                        a = a,
                        b = b,

                        axis = axis,
                        idx = idx,
                    )

                    /*
                    face1<->face2
                    face1<->edge
                    face2<->edge
                    
                    connections need to be spatial... so an edge can only connect a certain range of 2 faces
                    
                     */

                    ne.xor(start, end, ne.dataIdx())

                    node.edge(numPts, ne)
                    node.numPoints(numPts + 1)

                    return true
                }

                node.subdivide()
            }

            edgeInsertionQueue.enqueue(node.child1().idx)
            edgeInsertionQueue.enqueue(node.child2().idx)
            edgeInsertionQueue.enqueue(node.child3().idx)
            edgeInsertionQueue.enqueue(node.child4().idx)
        }

        return false
    }

    private fun Node.subdivide() {
        val n1 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = minA(),
            minB = centerB(),
            maxA = centerA(),
            maxB = maxB(),

            c1 = Node(-1),
            c2 = Node(-1),
            c3 = Node(-1),
            c4 = Node(-1),
        )

        val n2 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = minA(),
            minB = minB(),
            maxA = centerA(),
            maxB = centerB(),

            c1 = Node(-1),
            c2 = Node(-1),
            c3 = Node(-1),
            c4 = Node(-1),
        )

        val n3 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = centerA(),
            minB = minB(),
            maxA = maxA(),
            maxB = centerB(),

            c1 = Node(-1),
            c2 = Node(-1),
            c3 = Node(-1),
            c4 = Node(-1),
        )

        val n4 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = centerA(),
            minB = centerB(),
            maxA = maxA(),
            maxB = maxB(),

            c1 = Node(-1),
            c2 = Node(-1),
            c3 = Node(-1),
            c4 = Node(-1),
        )

        val n = numPoints()

        var i = 0
        while (i < n) {
            val ei = edge(i)
            n1.insertFormedEdge(ei) ||
            n2.insertFormedEdge(ei) ||
            n3.insertFormedEdge(ei) ||
            n4.insertFormedEdge(ei)

            i++
        }

        setLeaf(false)
        numPoints(0)

        child1(n1)
        child2(n2)
        child3(n3)
        child4(n4)
    }

    private fun Node.insertFormedEdge(edge: Edge): Boolean {
        val ea = edge.a()
        val eb = edge.b()

        if (!contains(ea, eb)) return false
        check(isLeaf())

        val n = numPoints()

        if (n < MAX_POINTS_PER_NODE) {
            edge(n, edge)
            numPoints(n + 1)

            return true
        }

        subdivide()

        return child1().insertFormedEdge(edge) ||
               child2().insertFormedEdge(edge) ||
               child3().insertFormedEdge(edge) ||
               child4().insertFormedEdge(edge)
    }

    /**
     * Does not perform bounds checks
     * @param start Start in node index, inclusive
     * @param end End in node index, exclusive
     * @return Head of linked list
     */
    private fun setNodeArrFree(start: Int, end: Int): Node {
        if (end <= start) return Node(-1)

        var i = start
        while (i < end - 1) {
            Node(i).nextFreeNode(Node(i + 1))
            Node(i).freeNode()

            i++
        }

        if (end - 1 >= 0) {
            Node(end - 1).nextFreeNode(Node(-1))
            Node(end - 1).freeNode()
        }

        return Node(start)
    }

    private fun setEdgeArrFree(start: Int, end: Int): Edge {
        if (end <= start) return Edge(-1)

        var i = start
        while (i < end - 1) {
            Edge(i).nextFreeEdge(Edge(i + 1))
            Edge(i).freeEdge()

            i++
        }

        if (end - 1 >= 0) {
            Edge(end - 1).nextFreeEdge(Edge(-1))
            Edge(end - 1).freeEdge()
        }

        return Edge(start)
    }

    /**
     * Only meant to be used on free nodes
     * @return The index of the next free node; -1 if full
     */
    private fun Node.nextFreeNode(): Node {
        return Node(nodeArr[this.idx * NODE_DATA_SIZE + NEXT_FREE_IDX_OFFSET].toRawBits().toInt())
    }

    private fun Node.nextFreeNode(next: Node) {
        nodeArr[this.idx * NODE_DATA_SIZE + NEXT_FREE_IDX_OFFSET] = Double.fromBits(next.idx.toLong())
    }

    private fun Node.freeNode() {
        nodeArr[this.idx * NODE_DATA_SIZE + STATUS_OFFSET] =
            nodeArr[this.idx * NODE_DATA_SIZE + STATUS_OFFSET].withHigher(NODE_UNUSED_STATUS)
    }

    private fun Edge.nextFreeEdge(): Edge {
        return Edge(edgeArr[this.idx * EDGE_DATA_SIZE + NEXT_FREE_IDX_OFFSET].toRawBits().toInt())
    }

    private fun Edge.nextFreeEdge(next: Edge) {
        edgeArr[this.idx * EDGE_DATA_SIZE + NEXT_FREE_IDX_OFFSET] = Double.fromBits(next.idx.toLong())
    }

    private fun Edge.freeEdge() {
        edgeArr[this.idx * EDGE_DATA_SIZE + STATUS_OFFSET] =
            edgeArr[this.idx * EDGE_DATA_SIZE + STATUS_OFFSET].withHigher(NODE_UNUSED_STATUS)
    }

    private fun Node.isLeaf(): Boolean {
        return nodeArr[this.idx * NODE_DATA_SIZE + IS_LEAF_OFFSET].toRawBits().toInt() == 1
    }

    private fun Node.numPoints(n: Int) {
        nodeArr[this.idx * NODE_DATA_SIZE + NUM_POINTS_OFFSET] =
            nodeArr[this.idx * NODE_DATA_SIZE + NUM_POINTS_OFFSET].withHigher(n)
    }

    private fun Node.numPoints(): Int {
        return (nodeArr[this.idx * NODE_DATA_SIZE + NUM_POINTS_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Node.setLeaf(l: Boolean) {
        if (l) {
            nodeArr[this.idx * NODE_DATA_SIZE + IS_LEAF_OFFSET] =
                nodeArr[this.idx * NODE_DATA_SIZE + IS_LEAF_OFFSET].withLower(1)
        } else {
            nodeArr[this.idx * NODE_DATA_SIZE + IS_LEAF_OFFSET] =
                nodeArr[this.idx * NODE_DATA_SIZE + IS_LEAF_OFFSET].withLower(0)
        }
    }

    private fun Node.child1(): Node {
        return Node(nodeArr[this.idx * NODE_DATA_SIZE + C1_OFFSET].toRawBits().toInt())
    }

    private fun Node.child1(c: Node) {
        nodeArr[this.idx * NODE_DATA_SIZE + C1_OFFSET] = nodeArr[this.idx * NODE_DATA_SIZE + C1_OFFSET].withLower(c.idx)
    }

    private fun Node.child2(): Node {
        return Node((nodeArr[this.idx * NODE_DATA_SIZE + C2_OFFSET].toRawBits() ushr 32).toInt())
    }

    private fun Node.child2(c: Node) {
        nodeArr[this.idx * NODE_DATA_SIZE + C2_OFFSET] =
            nodeArr[this.idx * NODE_DATA_SIZE + C2_OFFSET].withHigher(c.idx)
    }

    private fun Node.child3(): Node {
        return Node(nodeArr[this.idx * NODE_DATA_SIZE + C3_OFFSET].toRawBits().toInt())
    }

    private fun Node.child3(c: Node) {
        nodeArr[this.idx * NODE_DATA_SIZE + C3_OFFSET] = nodeArr[this.idx * NODE_DATA_SIZE + C3_OFFSET].withLower(c.idx)
    }

    private fun Node.child4(): Node {
        return Node((nodeArr[this.idx * NODE_DATA_SIZE + C4_OFFSET].toRawBits() ushr 32).toInt())
    }

    private fun Node.child4(c: Node) {
        nodeArr[this.idx * NODE_DATA_SIZE + C4_OFFSET] =
            nodeArr[this.idx * NODE_DATA_SIZE + C4_OFFSET].withHigher(c.idx)
    }

    /**
     * Called on a node index
     * @return index of edge in edge array
     */
    private fun Node.edge(idx: Int): Edge {
        return Edge(nodeArr[this.idx * NODE_DATA_SIZE + EDGES_OFFSET + idx].toRawBits().toInt())
    }

    private fun Node.edge(idx: Int, v: Edge) {
        nodeArr[this.idx * NODE_DATA_SIZE + EDGES_OFFSET + idx] = Double.fromBits(v.idx.toLong())
    }

    private fun Edge.f1(): Int {
        return edgeArr[this.idx * EDGE_DATA_SIZE + F1_OFFSET].toRawBits().toInt()
    }

    private fun Edge.f1(i: Int) {
        edgeArr[this.idx * EDGE_DATA_SIZE + F1_OFFSET].withLower(i)
    }

    private fun Edge.f2(): Int {
        return (edgeArr[this.idx * EDGE_DATA_SIZE + F2_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Edge.f2(i: Int) {
        edgeArr[this.idx * EDGE_DATA_SIZE + F2_OFFSET].withHigher(i)
    }

    private fun Edge.a(): Double {
        return edgeArr[this.idx * EDGE_DATA_SIZE + A_OFFSET]
    }

    private fun Edge.a(d: Double) {
        edgeArr[this.idx * EDGE_DATA_SIZE + A_OFFSET] = d
    }

    private fun Edge.b(): Double {
        return edgeArr[this.idx * EDGE_DATA_SIZE + B_OFFSET]
    }

    private fun Edge.b(d: Double) {
        edgeArr[this.idx * EDGE_DATA_SIZE + B_OFFSET] = d
    }

    private fun Edge.dataIdx(): Int {
        return edgeArr[this.idx * EDGE_DATA_SIZE + DATA_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Edge.dataIdx(i: Int) {
        edgeArr[this.idx * EDGE_DATA_SIZE + DATA_IDX_OFFSET] = Double.fromBits(i.toLong())
    }

    private fun Edge.dataAxis(): LocalMesher.AxisD {
        return axiss[(edgeArr[this.idx * EDGE_DATA_SIZE + AXIS_OFFSET].toRawBits() ushr 32).toInt()]
    }

    private fun Edge.dataAxis(axis: LocalMesher.AxisD) {
        edgeArr[this.idx * EDGE_DATA_SIZE + AXIS_OFFSET] =
            edgeArr[this.idx * EDGE_DATA_SIZE + AXIS_OFFSET].withHigher(axis.levelOffset)
    }

    /**
     * `this` is index of edge in edgeArray
     */
    private fun Edge.xor(start: Double, end: Double, dataIdx: Int) {
        val _pts = data(dataIdx)._points

        _pts.add(start)
        _pts.add(end)
    }

    private fun Edge.data(dataIdx: Int): EdgeData {
        return edgeData[dataIdx]
    }

    /**
     * @return The index of the new node
     */
    private fun newNode(
        isLeaf: Boolean,
        numPoints: Int,

        minA: Double,
        minB: Double,
        maxA: Double,
        maxB: Double,

        c1: Node,
        c2: Node,
        c3: Node,
        c4: Node,
    ): Node {
        val currFreeIdx = nodeFreeIdx
        if (currFreeIdx.idx == -1) {
            val f = growNodeArrTo((nodeArr.size / NODE_DATA_SIZE) + 1)
            check(f.idx != -1)
            nodeFreeIdx = f.nextFreeNode()

            f.resetNode(
                isLeaf = isLeaf,
                numPoints = numPoints,

                minA = minA,
                minB = minB,
                maxA = maxA,
                maxB = maxB,

                c1 = c1,
                c2 = c2,
                c3 = c3,
                c4 = c4,
            )

            return f
        }

        nodeFreeIdx = nodeFreeIdx.nextFreeNode()

        if (nodeFreeIdx.idx == -1) {
            nodeFreeIdx = growNodeArrTo((nodeArr.size / NODE_DATA_SIZE) + 1)
        }

        currFreeIdx.resetNode(
            isLeaf = isLeaf,
            numPoints = numPoints,

            minA = minA,
            minB = minB,
            maxA = maxA,
            maxB = maxB,

            c1 = c1,
            c2 = c2,
            c3 = c3,
            c4 = c4,
        )

        return currFreeIdx
    }

    private fun newEdge(
        f1: Int,
        f2: Int,

        a: Double,
        b: Double,

        axis: LocalMesher.AxisD,
        idx: Int,
    ): Edge {
        val currFreeIdx = edgeFreeIdx
        if (currFreeIdx.idx == -1) {
            val f = growEdgeArrTo((edgeArr.size / EDGE_DATA_SIZE) + 1)
            check(f.idx != -1)
            edgeFreeIdx = f.nextFreeEdge()

            f.resetEdge(
                f1 = f1,
                f2 = f2,

                a = a,
                b = b,

                axis = axis,
                idx = idx,
            )

            return f
        }

        edgeFreeIdx = edgeFreeIdx.nextFreeEdge()

        if (edgeFreeIdx.idx == -1) {
            edgeFreeIdx = growEdgeArrTo((edgeArr.size / EDGE_DATA_SIZE) + 1)
        }

        currFreeIdx.resetEdge(
            f1 = f1,
            f2 = f2,

            a = a,
            b = b,

            axis = axis,
            idx = idx,
        )

        return currFreeIdx
    }

    private fun Node.resetNode(
        isLeaf: Boolean,
        numPoints: Int,

        minA: Double,
        minB: Double,
        maxA: Double,
        maxB: Double,

        c1: Node,
        c2: Node,
        c3: Node,
        c4: Node,
    ) {
        setLeaf(isLeaf)
        numPoints(numPoints)

        minA(minA)
        minB(minB)
        maxA(maxA)
        maxB(maxB)

        child1(c1)
        child2(c2)
        child3(c3)
        child4(c4)
    }

    private fun Edge.resetEdge(
        f1: Int,
        f2: Int,

        a: Double,
        b: Double,

        axis: LocalMesher.AxisD,
        idx: Int,
    ) {
        f1(f1)
        f2(f2)

        a(a)
        b(b)

        dataAxis(axis)
        dataIdx(idx)
    }

    /**
     * Grows array to fit the extra element and fills it with linked list of unused nodes
     * @param size Size in nodes
     * @return Head of newly created list, or current free index of nothing new was needed
     */
    private fun growNodeArrTo(size: Int): Node {
        if (nodeArr.size >= size * NODE_DATA_SIZE) return nodeFreeIdx

        val prevSize = nodeArr.size
        val newSize = max(size * NODE_DATA_SIZE, (nodeArr.size * 3 / 2) / NODE_DATA_SIZE * NODE_DATA_SIZE)

        nodeArr = nodeArr.copyOf(newSize)

        setNodeArrFree(prevSize / NODE_DATA_SIZE, newSize / NODE_DATA_SIZE)
        return Node(prevSize / NODE_DATA_SIZE)
    }

    private fun growEdgeArrTo(size: Int): Edge {
        if (edgeArr.size >= size * EDGE_DATA_SIZE) return edgeFreeIdx

        val prevSize = edgeArr.size
        val newSize = max(size * EDGE_DATA_SIZE, (edgeArr.size * 3 / 2) / EDGE_DATA_SIZE * EDGE_DATA_SIZE)

        edgeArr = edgeArr.copyOf(newSize)

        setEdgeArrFree(prevSize / EDGE_DATA_SIZE, newSize / EDGE_DATA_SIZE)
        return Edge(prevSize / EDGE_DATA_SIZE)
    }

    private fun Node.contains(a: Double, b: Double): Boolean {
        return a >= minA() && a < maxA() &&
               b >= minB() && b < maxB()
    }

    private fun Node.overlaps(
        minA: Double,
        minB: Double,
        maxA: Double,
        maxB: Double,
    ): Boolean {
        return maxA() >= minA && minA() < maxA &&
               maxB() >= minB && minB() < maxB
    }

    private fun Node.minA(): Double {
        return nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MIN_A_OFFSET]
    }

    private fun Node.minA(d: Double) {
        nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MIN_A_OFFSET] = d
    }

    private fun Node.minB(): Double {
        return nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MIN_B_OFFSET]
    }

    private fun Node.minB(d: Double) {
        nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MIN_B_OFFSET] = d
    }

    private fun Node.maxA(): Double {
        return nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MAX_A_OFFSET]
    }

    private fun Node.maxA(d: Double) {
        nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MAX_A_OFFSET] = d
    }

    private fun Node.maxB(): Double {
        return nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MAX_B_OFFSET]
    }

    private fun Node.maxB(d: Double) {
        nodeArr[this.idx * NODE_DATA_SIZE + BOUNDS_MAX_B_OFFSET] = d
    }

    private fun Node.centerA(): Double {
        return minA() * 0.5 + maxA() * 0.5
    }

    private fun Node.centerB(): Double {
        return minB() * 0.5 + maxB() * 0.5
    }

    /**
     * Ensures that data arrays can hold a new element and returns the index for this new element
     */
    private fun newDataIdx(): Int {
        val idx = edgeData.size

        edgeData += EdgeData()

        return idx
    }

    private val _varr1 = DoubleArray(3)
    private val _varr2 = DoubleArray(3)

    fun clear() {
        nodeFreeIdx = setNodeArrFree(0, nodeArr.size / NODE_DATA_SIZE)
        edgeFreeIdx = setEdgeArrFree(0, edgeArr.size / EDGE_DATA_SIZE)

        rootIdx = Node(-1)

        edgeData.clear()
    }

    fun visualize(world: World) {
        val q = IntStack()

        if (rootIdx.idx == -1) return

        q.enqueue(rootIdx.idx)

        while (q.hasNext()) {
            val i = Node(q.dequeue())

            if (i.isLeaf()) {
                var j = 0
                val n = i.numPoints()
                while (j < n) {
                    val e = i.edge(j)
                    val ea = e.data(e.dataIdx())

                    val a = e.a()
                    val b = e.b()

                    val pts = ea.points

                    _varr1[axis.aOffset] = a
                    _varr1[axis.bOffset] = b

                    _varr2[axis.aOffset] = a
                    _varr2[axis.bOffset] = b

                    val itr = pts.iterator()
                    while (itr.hasNext()) {
                        val d1 = itr.nextDouble()
                        val d2 = itr.nextDouble()

                        _varr1[axis.levelOffset] = d1
                        _varr2[axis.levelOffset] = d2

                        val s = Vector3d(_varr1)
                        val e = Vector3d(_varr2)

                        world.debugConnect(
                            start = s,
                            end = e,
                            options = Particle.DustOptions(Color.LIME, 0.4f),
                        )
                    }

                    j++
                }
            } else {
                q.enqueue(i.child1().idx)
                q.enqueue(i.child2().idx)
                q.enqueue(i.child3().idx)
                q.enqueue(i.child4().idx)
            }
        }
    }
}

const val MAX_POINTS_PER_NODE = 4
const val MOUNT_EPSILON = 1e-6

private const val NODE_DATA_SIZE = 7 + MAX_POINTS_PER_NODE

private const val IS_LEAF_OFFSET = 0 // lower 32 bits
private const val NUM_POINTS_OFFSET = 0 // higher 32 bits

private const val BOUNDS_MIN_A_OFFSET = 1
private const val BOUNDS_MIN_B_OFFSET = 2
private const val BOUNDS_MAX_A_OFFSET = 3
private const val BOUNDS_MAX_B_OFFSET = 4

private const val C1_OFFSET = 5 // lower half
private const val C2_OFFSET = 5 // upper half
private const val C3_OFFSET = 6 // lower half
private const val C4_OFFSET = 6 // upper half

private const val EDGES_OFFSET = 7

private const val NEXT_FREE_IDX_OFFSET = 0 // low 32 bits
private const val STATUS_OFFSET =
    0 // high 32 bits; normally represents natural number, however is -1 if it's unused

private const val NODE_UNUSED_STATUS = -1

private const val EDGE_DATA_SIZE = 4

private const val F1_OFFSET = 0 // lower half
private const val F2_OFFSET = 0 // higher half

private const val A_OFFSET = 1
private const val B_OFFSET = 2

private const val DATA_IDX_OFFSET = 3 // lower half
private const val AXIS_OFFSET = 3 // upper half

const val ASYMMETRY_EPSILON = 1e-8
const val ABOVE_ASYMMETRY_EPSILON = 2f * ASYMMETRY_EPSILON

private const val X_PRIME = 65282653
private const val Y_PRIME = 49021097
private const val Z_PRIME = 20262443

@JvmInline
private value class Node(val idx: Int)

@JvmInline
private value class Edge(val idx: Int)
