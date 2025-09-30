package com.ixume.udar.collisiondetection.mesh.quadtree

import com.google.common.math.IntMath
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaces
import com.ixume.udar.collisiondetection.mesh.mesh2.axiss
import com.ixume.udar.collisiondetection.mesh.mesh2.faceID
import com.ixume.udar.dynamicaabb.array.FlattenedAABBTree
import com.ixume.udar.dynamicaabb.array.IntQueue
import com.ixume.udar.dynamicaabb.array.withHigher
import com.ixume.udar.dynamicaabb.array.withLower
import com.ixume.udar.physics.contact.LongGraph
import com.ixume.udar.testing.debugConnect
import it.unimi.dsi.fastutil.doubles.DoubleAVLTreeSet
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntArrayList
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
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
    val _points = mutableListOf<DoubleArrayList>()

    val points = mutableListOf<DoubleAVLTreeSet>()
    val pointMounts = mutableListOf<IntArrayList>()

    private var nodeFreeIdx = -1 // head of node free linked list
    private var edgeFreeIdx = -1 // head of edge free linked list

    private var rootIdx = -1

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

            c1 = -1,
            c2 = -1,
            c3 = -1,
            c4 = -1,
        )
    }

    fun insertEdge(x: Double, y: Double, start: Double, end: Double, meshFaces: MeshFaces, graph: LongGraph) {
        _insertEdge(x, y, start, end - ASYMMETRY_EPSILON, meshFaces, graph)
    }

    private val fixupQueue = IntQueue()

    private val _vec3Mounts = DoubleArray(12)

    fun fixUp(tree: FlattenedAABBTree, faces: MeshFaces) {
        if (rootIdx == -1) return

        fixupQueue.enqueue(rootIdx)

        while (fixupQueue.hasNext()) {
            val node = fixupQueue.dequeue()

            if (node.isLeaf()) {
                var i = 0
                val n = node.numPoints()

                while (i < n) {
                    val pts = DoubleArrayList()
                    val ptMounts = IntArrayList()

                    val edge = node.edge(i)
                    val dataIdx = edge.dataIdx()
                    val _pts = edge._points(dataIdx)
                    _pts.sort()
                    val edgePts = edge.points(dataIdx)

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
                        continue
                    }

                    check(pts.size % 2 == 0)
                    check(ptMounts.isNotEmpty())
                    check(ptMounts.size * 2 == pts.size)

                    val edgeMounts = edge.mounts(dataIdx)

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
                fixupQueue.enqueue(node.child1())
                fixupQueue.enqueue(node.child2())
                fixupQueue.enqueue(node.child3())
                fixupQueue.enqueue(node.child4())
            }
        }
    }

    fun edgeID(edgeIdx: Int, s: Double, e: Double): Long {
        val prime = when (axis) {
            LocalMesher.AxisD.X -> X_PRIME
            LocalMesher.AxisD.Y -> Y_PRIME
            LocalMesher.AxisD.Z -> Z_PRIME
        }

        return (IntMath.pow(prime, edgeIdx).toLong() shl 32) xor s.toRawBits() xor e.toRawBits()
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
        if (rootIdx == -1) return

        val q = math.envEdgeOverlapQueue

        q.enqueue(rootIdx)

        while (q.hasNext()) {
            val node = q.dequeue()

            when (axis) {
                LocalMesher.AxisD.X -> {
                    if (!node.overlaps(minY, minZ, maxY, maxZ)) continue

                    if (node.isLeaf()) {
                        val np = node.numPoints()
                        var i = 0
                        while (i < np) {
                            val e = node.edge(i)
                            val a = e.a()
                            val b = e.b()

                            if (a >= minY && a <= maxY &&
                                b >= minZ && b <= maxZ
                            ) {
                                outA.add(a)
                                outB.add(b)
                                outEdges.add(e)
                                outData.add(e.dataIdx())
                            }

                            i++
                        }

                        continue
                    }
                }

                LocalMesher.AxisD.Y -> {
                    if (!node.overlaps(minX, minZ, maxX, maxZ)) continue

                    if (node.isLeaf()) {
                        val np = node.numPoints()
                        var i = 0
                        while (i < np) {
                            val e = node.edge(i)
                            val a = e.a()
                            val b = e.b()

                            if (a >= minX && a <= maxX &&
                                b >= minZ && b <= maxZ
                            ) {
                                outA.add(a)
                                outB.add(b)
                                outEdges.add(e)
                                outData.add(e.dataIdx())
                            }

                            i++
                        }

                        continue
                    }
                }

                LocalMesher.AxisD.Z -> {
                    if (!node.overlaps(minX, minY, maxX, maxY)) continue

                    if (node.isLeaf()) {
                        val np = node.numPoints()
                        var i = 0
                        while (i < np) {
                            val e = node.edge(i)
                            val a = e.a()
                            val b = e.b()

                            if (a >= minX && a <= maxX &&
                                b >= minY && b <= maxY
                            ) {
                                outA.add(a)
                                outB.add(b)
                                outEdges.add(e)
                                outData.add(e.dataIdx())
                            }

                            i++
                        }

                        continue
                    }
                }
            }

            q.enqueue(node.child1())
            q.enqueue(node.child2())
            q.enqueue(node.child3())
            q.enqueue(node.child4())
        }
    }

    private val edgeInsertionQueue = IntQueue()

    private fun _insertEdge(
        a: Double,
        b: Double,
        start: Double,
        end: Double,
        meshFaces: MeshFaces,
        graph: LongGraph,
    ): Boolean {
        edgeInsertionQueue.enqueue(rootIdx)

        while (edgeInsertionQueue.hasNext()) {
            val node = edgeInsertionQueue.dequeue()

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

            edgeInsertionQueue.enqueue(node.child1())
            edgeInsertionQueue.enqueue(node.child2())
            edgeInsertionQueue.enqueue(node.child3())
            edgeInsertionQueue.enqueue(node.child4())
        }

        return false
    }

    private fun Int.subdivide() {
        val n1 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = minA(),
            minB = centerB(),
            maxA = centerA(),
            maxB = maxB(),

            c1 = -1,
            c2 = -1,
            c3 = -1,
            c4 = -1,
        )

        val n2 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = minA(),
            minB = minB(),
            maxA = centerA(),
            maxB = centerB(),

            c1 = -1,
            c2 = -1,
            c3 = -1,
            c4 = -1,
        )

        val n3 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = centerA(),
            minB = minB(),
            maxA = maxA(),
            maxB = centerB(),

            c1 = -1,
            c2 = -1,
            c3 = -1,
            c4 = -1,
        )

        val n4 = newNode(
            isLeaf = true,
            numPoints = 0,

            minA = centerA(),
            minB = centerB(),
            maxA = maxA(),
            maxB = maxB(),

            c1 = -1,
            c2 = -1,
            c3 = -1,
            c4 = -1,
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

    private fun Int.insertFormedEdge(edge: Int): Boolean {
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
    private fun setNodeArrFree(start: Int, end: Int): Int {
        if (end <= start) return -1

        var i = start
        while (i < end - 1) {
            i.nextFreeNode(i + 1)
            i.freeNode()

            i++
        }

        if (end - 1 >= 0) {
            (end - 1).nextFreeNode(-1)
            (end - 1).freeNode()
        }

        return start
    }

    private fun setEdgeArrFree(start: Int, end: Int): Int {
        if (end <= start) return -1

        var i = start
        while (i < end - 1) {
            i.nextFreeEdge(i + 1)
            i.freeEdge()

            i++
        }

        if (end - 1 >= 0) {
            (end - 1).nextFreeEdge(-1)
            (end - 1).freeEdge()
        }

        return start
    }

    /**
     * Only meant to be used on free nodes
     * @return The index of the next free node; -1 if full
     */
    private fun Int.nextFreeNode(): Int {
        return nodeArr[this * NODE_DATA_SIZE + NEXT_FREE_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Int.nextFreeNode(next: Int) {
        nodeArr[this * NODE_DATA_SIZE + NEXT_FREE_IDX_OFFSET] = Double.fromBits(next.toLong())
    }

    private fun Int.freeNode() {
        nodeArr[this * NODE_DATA_SIZE + STATUS_OFFSET] =
            nodeArr[this * NODE_DATA_SIZE + STATUS_OFFSET].withHigher(NODE_UNUSED_STATUS)
    }

    private fun Int.nextFreeEdge(): Int {
        return edgeArr[this * EDGE_DATA_SIZE + NEXT_FREE_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Int.nextFreeEdge(next: Int) {
        edgeArr[this * EDGE_DATA_SIZE + NEXT_FREE_IDX_OFFSET] = Double.fromBits(next.toLong())
    }

    private fun Int.freeEdge() {
        edgeArr[this * EDGE_DATA_SIZE + STATUS_OFFSET] =
            edgeArr[this * EDGE_DATA_SIZE + STATUS_OFFSET].withHigher(NODE_UNUSED_STATUS)
    }

    private fun Int.isLeaf(): Boolean {
        return nodeArr[this * NODE_DATA_SIZE + IS_LEAF_OFFSET].toRawBits().toInt() == 1
    }

    private fun Int.numPoints(n: Int) {
        nodeArr[this * NODE_DATA_SIZE + NUM_POINTS_OFFSET] =
            nodeArr[this * NODE_DATA_SIZE + NUM_POINTS_OFFSET].withHigher(n)
    }

    private fun Int.numPoints(): Int {
        return (nodeArr[this * NODE_DATA_SIZE + NUM_POINTS_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Int.setLeaf(l: Boolean) {
        if (l) {
            nodeArr[this * NODE_DATA_SIZE + IS_LEAF_OFFSET] =
                nodeArr[this * NODE_DATA_SIZE + IS_LEAF_OFFSET].withLower(1)
        } else {
            nodeArr[this * NODE_DATA_SIZE + IS_LEAF_OFFSET] =
                nodeArr[this * NODE_DATA_SIZE + IS_LEAF_OFFSET].withLower(0)
        }
    }

    private fun Int.child1(): Int {
        return nodeArr[this * NODE_DATA_SIZE + C1_OFFSET].toRawBits().toInt()
    }

    private fun Int.child1(c: Int) {
        nodeArr[this * NODE_DATA_SIZE + C1_OFFSET] = nodeArr[this * NODE_DATA_SIZE + C1_OFFSET].withLower(c)
    }

    private fun Int.child2(): Int {
        return (nodeArr[this * NODE_DATA_SIZE + C2_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Int.child2(c: Int) {
        nodeArr[this * NODE_DATA_SIZE + C2_OFFSET] = nodeArr[this * NODE_DATA_SIZE + C2_OFFSET].withHigher(c)
    }

    private fun Int.child3(): Int {
        return nodeArr[this * NODE_DATA_SIZE + C3_OFFSET].toRawBits().toInt()
    }

    private fun Int.child3(c: Int) {
        nodeArr[this * NODE_DATA_SIZE + C3_OFFSET] = nodeArr[this * NODE_DATA_SIZE + C3_OFFSET].withLower(c)
    }

    private fun Int.child4(): Int {
        return (nodeArr[this * NODE_DATA_SIZE + C4_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Int.child4(c: Int) {
        nodeArr[this * NODE_DATA_SIZE + C4_OFFSET] = nodeArr[this * NODE_DATA_SIZE + C4_OFFSET].withHigher(c)
    }

    /**
     * Called on a node index
     * @return index of edge in edge array
     */
    private fun Int.edge(idx: Int): Int {
        return nodeArr[this * NODE_DATA_SIZE + EDGES_OFFSET + idx].toRawBits().toInt()
    }

    private fun Int.edge(idx: Int, v: Int) {
        nodeArr[this * NODE_DATA_SIZE + EDGES_OFFSET + idx] = Double.fromBits(v.toLong())
    }

    private fun Int.f1(): Int {
        return edgeArr[this * EDGE_DATA_SIZE + F1_OFFSET].toRawBits().toInt()
    }

    private fun Int.f1(i: Int) {
        edgeArr[this * EDGE_DATA_SIZE + F1_OFFSET].withLower(i)
    }

    private fun Int.f2(): Int {
        return (edgeArr[this * EDGE_DATA_SIZE + F2_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Int.f2(i: Int) {
        edgeArr[this * EDGE_DATA_SIZE + F2_OFFSET].withHigher(i)
    }

    private fun Int.a(): Double {
        return edgeArr[this * EDGE_DATA_SIZE + A_OFFSET]
    }

    private fun Int.a(d: Double) {
        edgeArr[this * EDGE_DATA_SIZE + A_OFFSET] = d
    }

    private fun Int.b(): Double {
        return edgeArr[this * EDGE_DATA_SIZE + B_OFFSET]
    }

    private fun Int.b(d: Double) {
        edgeArr[this * EDGE_DATA_SIZE + B_OFFSET] = d
    }

    private fun Int.dataIdx(): Int {
        return edgeArr[this * EDGE_DATA_SIZE + DATA_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Int.dataIdx(i: Int) {
        edgeArr[this * EDGE_DATA_SIZE + DATA_IDX_OFFSET] = Double.fromBits(i.toLong())
    }

    private fun Int.dataAxis(): LocalMesher.AxisD {
        return axiss[(edgeArr[this * EDGE_DATA_SIZE + AXIS_OFFSET].toRawBits() ushr 32).toInt()]
    }

    private fun Int.dataAxis(axis: LocalMesher.AxisD) {
        edgeArr[this * EDGE_DATA_SIZE + AXIS_OFFSET] =
            edgeArr[this * EDGE_DATA_SIZE + AXIS_OFFSET].withHigher(axis.levelOffset)
    }

    /**
     * `this` is index of edge in edgeArray
     */
    private fun Int.xor(start: Double, end: Double, dataIdx: Int) {
        val _pts = _points(dataIdx)

        _pts.add(start)
        _pts.add(end)
    }

    private fun Int._points(dataIdx: Int): DoubleArrayList {
        return _points[dataIdx]
    }

    private fun Int.points(dataIdx: Int): DoubleAVLTreeSet {
        return points[dataIdx]
    }

    private fun Int.mounts(dataIdx: Int): IntArrayList {
        return pointMounts[dataIdx]
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

        c1: Int,
        c2: Int,
        c3: Int,
        c4: Int,
    ): Int {
        val currFreeIdx = nodeFreeIdx
        if (currFreeIdx == -1) {
            val f = growNodeArrTo((nodeArr.size / NODE_DATA_SIZE) + 1)
            check(f != -1)
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

        if (nodeFreeIdx == -1) {
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
    ): Int {
        val currFreeIdx = edgeFreeIdx
        if (currFreeIdx == -1) {
            val f = growEdgeArrTo((edgeArr.size / EDGE_DATA_SIZE) + 1)
            check(f != -1)
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

        if (edgeFreeIdx == -1) {
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

    private fun Int.resetNode(
        isLeaf: Boolean,
        numPoints: Int,

        minA: Double,
        minB: Double,
        maxA: Double,
        maxB: Double,

        c1: Int,
        c2: Int,
        c3: Int,
        c4: Int,
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

    private fun Int.resetEdge(
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
    private fun growNodeArrTo(size: Int): Int {
        if (nodeArr.size >= size * NODE_DATA_SIZE) return nodeFreeIdx

        val prevSize = nodeArr.size
        val newSize = max(size * NODE_DATA_SIZE, (nodeArr.size * 3 / 2) / NODE_DATA_SIZE * NODE_DATA_SIZE)

        nodeArr = nodeArr.copyOf(newSize)

        setNodeArrFree(prevSize / NODE_DATA_SIZE, newSize / NODE_DATA_SIZE)
        return prevSize / NODE_DATA_SIZE
    }

    private fun growEdgeArrTo(size: Int): Int {
        if (edgeArr.size >= size * EDGE_DATA_SIZE) return edgeFreeIdx

        val prevSize = edgeArr.size
        val newSize = max(size * EDGE_DATA_SIZE, (edgeArr.size * 3 / 2) / EDGE_DATA_SIZE * EDGE_DATA_SIZE)

        edgeArr = edgeArr.copyOf(newSize)

        setEdgeArrFree(prevSize / EDGE_DATA_SIZE, newSize / EDGE_DATA_SIZE)
        return prevSize / EDGE_DATA_SIZE
    }

    private fun Int.contains(a: Double, b: Double): Boolean {
        return a >= minA() && a < maxA() &&
               b >= minB() && b < maxB()
    }

    private fun Int.overlaps(
        minA: Double,
        minB: Double,
        maxA: Double,
        maxB: Double,
    ): Boolean {
        return maxA() >= minA && minA() <= maxA &&
               maxB() >= minB && minB() <= maxB
    }

    private fun Int.minA(): Double {
        return nodeArr[this * NODE_DATA_SIZE + BOUNDS_MIN_A_OFFSET]
    }

    private fun Int.minA(d: Double) {
        nodeArr[this * NODE_DATA_SIZE + BOUNDS_MIN_A_OFFSET] = d
    }

    private fun Int.minB(): Double {
        return nodeArr[this * NODE_DATA_SIZE + BOUNDS_MIN_B_OFFSET]
    }

    private fun Int.minB(d: Double) {
        nodeArr[this * NODE_DATA_SIZE + BOUNDS_MIN_B_OFFSET] = d
    }

    private fun Int.maxA(): Double {
        return nodeArr[this * NODE_DATA_SIZE + BOUNDS_MAX_A_OFFSET]
    }

    private fun Int.maxA(d: Double) {
        nodeArr[this * NODE_DATA_SIZE + BOUNDS_MAX_A_OFFSET] = d
    }

    private fun Int.maxB(): Double {
        return nodeArr[this * NODE_DATA_SIZE + BOUNDS_MAX_B_OFFSET]
    }

    private fun Int.maxB(d: Double) {
        nodeArr[this * NODE_DATA_SIZE + BOUNDS_MAX_B_OFFSET] = d
    }

    private fun Int.centerA(): Double {
        return minA() * 0.5 + maxA() * 0.5
    }

    private fun Int.centerB(): Double {
        return minB() * 0.5 + maxB() * 0.5
    }

    /**
     * Ensures that data arrays can hold a new element and returns the index for this new element
     */
    private fun newDataIdx(): Int {
        check(_points.size == points.size)
        check(_points.size == pointMounts.size)

        val idx = _points.size

        _points += DoubleArrayList()
        points += DoubleAVLTreeSet()
        pointMounts += IntArrayList()

        return idx
    }

    private val _varr1 = DoubleArray(3)
    private val _varr2 = DoubleArray(3)

    fun clear() {
        nodeFreeIdx = setNodeArrFree(0, nodeArr.size / NODE_DATA_SIZE)
        edgeFreeIdx = setEdgeArrFree(0, edgeArr.size / EDGE_DATA_SIZE)

        rootIdx = -1

        _points.clear()

        points.clear()
        pointMounts.clear()
    }

    fun visualize(world: World) {
        val q = IntQueue()

        if (rootIdx == -1) return

        q.enqueue(rootIdx)

        while (q.hasNext()) {
            val i = q.dequeue()

            if (i.isLeaf()) {
                var j = 0
                val n = i.numPoints()
                while (j < n) {
                    val e = i.edge(j)

                    val a = e.a()
                    val b = e.b()

                    val pts = e.points(e.dataIdx())

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
                q.enqueue(i.child1())
                q.enqueue(i.child2())
                q.enqueue(i.child3())
                q.enqueue(i.child4())
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
