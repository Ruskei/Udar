package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D.Companion.MAX_A
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D.Companion.MAX_B
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D.Companion.MIN_A
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D.Companion.MIN_B
import com.ixume.udar.collisiondetection.mesh.quadtree.EdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.dynamicaabb.AABBTree
import org.bukkit.World
import org.joml.Vector2d
import org.joml.Vector3d
import org.joml.Vector3i
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

class LocalMesher {
    private val _bbTree = AABBTree()
    private val _bbs = BBList()

    private lateinit var _xAxiss: EdgeQuadtree
    private lateinit var _yAxiss: EdgeQuadtree
    private lateinit var _zAxiss: EdgeQuadtree

    fun mesh(
        world: World,
        boundingBox: AABB,
    ): Mesh2 {
        val meshStart = Vector3i(
            floor(boundingBox.minX).toInt(),
            floor(boundingBox.minY).toInt(),
            floor(boundingBox.minZ).toInt(),
        )
        val meshEnd = Vector3i(
            floor(boundingBox.maxX).toInt(),
            floor(boundingBox.maxY).toInt(),
            floor(boundingBox.maxZ).toInt(),
        )

        _bbTree.clear()
        _bbs.clear()

        for (x in (meshStart.x)..(meshEnd.x)) {
            for (y in (meshStart.y)..(meshEnd.y)) {
                for (z in (meshStart.z)..(meshEnd.z)) {
                    val block = world.getBlockAt(x, y, z)
                    if (block.isPassable) continue
                    for (boundingBox in block.collisionShape.boundingBoxes) {
                        val bb = boundingBox.clone()
                        bb.shift(block.location)
                        val aabb = AABB(bb.minX, bb.minY, bb.minZ, bb.maxX, bb.maxY, bb.maxZ)
                        _bbTree.insert(aabb)
                        _bbs.add(aabb)
                    }
                }
            }
        }

        if (_bbs.size == 0) {
            return Mesh2(
                start = meshStart,
                end = meshEnd,
            )
        }

//        println("BBs: ${_bbs.size}")

        val meshFaces = MeshFaces(
            createMeshFaces(AxisD.X, meshStart, meshEnd),
            createMeshFaces(AxisD.Y, meshStart, meshEnd),
            createMeshFaces(AxisD.Z, meshStart, meshEnd),
        )

        createMeshEdges(meshStart, meshEnd, meshFaces)

        return Mesh2(
            start = meshStart,
            end = meshEnd,
            faces = meshFaces,
            xEdges = _xAxiss,
            yEdges = _yAxiss,
            zEdges = _zAxiss,
        )
    }

    private fun createMeshFaces(
        axis: AxisD,
        meshStart: Vector3i,
        meshEnd: Vector3i,
    ): MeshFaceSortedList {
        /*
        iterate through all bb's, and add holes and anti-holes
        terminology: a face F points in axis A (A in [X, Y, Z]). the A'th component of F is its 'level'
        for a bb, we can quickly find matching face with binary search, or quickly confirm that there exists none

        for bb in bbs
            relevantFace = if bb.min in faces
                it
            else
                new face

            add hole

        add anti-holes
         */

        val faces = MeshFaceSortedList(axis)

        val _bb = DoubleArray(6)

        var i = 0
        while (i < _bbs.size) {
            _bbs.get(i, _bb)

            val minFace = faces.placeFaceAt(_bb.minLevel(axis))
            minFace.stamp(_bb, false)

            val maxFace = faces.placeFaceAt(_bb.maxLevel(axis))
            maxFace.stamp(_bb, true)

            i++
        }

        faces.constructAntiHoles()

        return faces
    }

    private fun MeshFace.stamp(bb: DoubleArray, inDir: Boolean) {
        val axis = axis
        val arr = DoubleArray(4)

        arr[MIN_A] = bb.minA(axis)
        arr[MIN_B] = bb.minB(axis)
        arr[MAX_A] = bb.maxA(axis)
        arr[MAX_B] = bb.maxB(axis)

        val bb2d = AABB2D(data = arr)
        bb2d.inDir = inDir

        holes.insert(bb2d)
    }

    private inline fun DoubleArray.minLevel(axis: AxisD): Double {
        return this[axis.levelOffset]
    }

    private inline fun DoubleArray.minA(axis: AxisD): Double {
        return this[axis.aOffset]
    }

    private inline fun DoubleArray.minB(axis: AxisD): Double {
        return this[axis.bOffset]
    }

    private inline fun DoubleArray.maxLevel(axis: AxisD): Double {
        return this[3 + axis.levelOffset]
    }

    private inline fun DoubleArray.maxA(axis: AxisD): Double {
        return this[3 + axis.aOffset]
    }

    private inline fun DoubleArray.maxB(axis: AxisD): Double {
        return this[3 + axis.bOffset]
    }

    private fun createMeshEdges(
        meshStart: Vector3i,
        meshEnd: Vector3i,
        meshFaces: MeshFaces,
    ) {
        _xAxiss = EdgeQuadtree(
            AxisD.X,
            Vector2d(meshStart.y.toDouble() - 2.0, meshStart.z.toDouble() - 2.0),
            Vector2d(meshEnd.y.toDouble() + 2.0, meshEnd.z.toDouble() + 2.0),
        )
        _yAxiss = EdgeQuadtree(
            AxisD.Y,
            Vector2d(meshStart.x.toDouble() - 2.0, meshStart.z.toDouble() - 2.0),
            Vector2d(meshEnd.x.toDouble() + 2.0, meshEnd.z.toDouble() + 2.0),
        )
        _zAxiss = EdgeQuadtree(
            AxisD.Z,
            Vector2d(meshStart.x.toDouble() - 2.0, meshStart.y.toDouble() - 2.0),
            Vector2d(meshEnd.x.toDouble() + 2.0, meshEnd.y.toDouble() + 2.0),
        )

        val _bb = DoubleArray(6) // [ minX, minY, minZ, maxX, maxY, maxZ ]

        var i = 0
        while (i < _bbs.size) {
            _bbs.get(i, _bb)

            //x axis
            val xStart = _bb[BB_MIN_X]
            val xEnd = _bb[BB_MAX_X]

            val x0aStart = _bb[BB_MIN_Y]
            val x0bStart = _bb[BB_MIN_Z]
            _xAxiss.insertEdge(x0aStart, x0bStart, xStart, xEnd, meshFaces)

            val x1aStart = _bb[BB_MAX_Y]
            val x1bStart = _bb[BB_MIN_Z]
            _xAxiss.insertEdge(x1aStart, x1bStart, xStart, xEnd, meshFaces)

            val x2aStart = _bb[BB_MAX_Y]
            val x2bStart = _bb[BB_MAX_Z]
            _xAxiss.insertEdge(x2aStart, x2bStart, xStart, xEnd, meshFaces)

            val x3aStart = _bb[BB_MIN_Y]
            val x3bStart = _bb[BB_MAX_Z]
            _xAxiss.insertEdge(x3aStart, x3bStart, xStart, xEnd, meshFaces)

            //y axis
            val yStart = _bb[BB_MIN_Y]
            val yEnd = _bb[BB_MAX_Y]

            val y0aStart = _bb[BB_MIN_X]
            val y0bStart = _bb[BB_MIN_Z]
            _yAxiss.insertEdge(y0aStart, y0bStart, yStart, yEnd, meshFaces)

            val y1aStart = _bb[BB_MAX_X]
            val y1bStart = _bb[BB_MIN_Z]
            _yAxiss.insertEdge(y1aStart, y1bStart, yStart, yEnd, meshFaces)

            val y2aStart = _bb[BB_MAX_X]
            val y2bStart = _bb[BB_MAX_Z]
            _yAxiss.insertEdge(y2aStart, y2bStart, yStart, yEnd, meshFaces)

            val y3aStart = _bb[BB_MIN_X]
            val y3bStart = _bb[BB_MAX_Z]
            _yAxiss.insertEdge(y3aStart, y3bStart, yStart, yEnd, meshFaces)

            //z axis
            val zStart = _bb[BB_MIN_Z]
            val zEnd = _bb[BB_MAX_Z]

            val z0aStart = _bb[BB_MIN_X]
            val z0bStart = _bb[BB_MIN_Y]
            _zAxiss.insertEdge(z0aStart, z0bStart, zStart, zEnd, meshFaces)

            val z1aStart = _bb[BB_MAX_X]
            val z1bStart = _bb[BB_MIN_Y]
            _zAxiss.insertEdge(z1aStart, z1bStart, zStart, zEnd, meshFaces)

            val z2aStart = _bb[BB_MAX_X]
            val z2bStart = _bb[BB_MAX_Y]
            _zAxiss.insertEdge(z2aStart, z2bStart, zStart, zEnd, meshFaces)

            val z3aStart = _bb[BB_MIN_X]
            val z3bStart = _bb[BB_MAX_Y]
            _zAxiss.insertEdge(z3aStart, z3bStart, zStart, zEnd, meshFaces)

            i++
        }

        _xAxiss.fixUp(_bbTree)
        _yAxiss.fixUp(_bbTree)
        _zAxiss.fixUp(_bbTree)
    }

    class Mesh2(
        val start: Vector3i,
        val end: Vector3i,
        val faces: MeshFaces? = null,
        val xEdges: EdgeQuadtree? = null,
        val yEdges: EdgeQuadtree? = null,
        val zEdges: EdgeQuadtree? = null,
    ) {
        fun visualize(world: World) {
//            faces?.xFaces?.ls?.forEach { it.visualize(world) }
//            faces?.yFaces?.ls?.forEach { it.visualize(world) }
//            faces?.zFaces?.ls?.forEach { it.visualize(world) }

            xEdges?.visualize(world)
            yEdges?.visualize(world)
            zEdges?.visualize(world)
        }
    }

    enum class AxisD(
        val vec: Vector3d,
        val levelOffset: Int,
        val aOffset: Int,
        val bOffset: Int,
    ) {
        X(Vector3d(1.0, 0.0, 0.0), 0, 1, 2), Y(Vector3d(0.0, 1.0, 0.0), 1, 0, 2), Z(Vector3d(0.0, 0.0, 1.0), 2, 0, 1);

        fun project(vertices: Array<Vector3d>): Pair<Double, Double> {
            var min = Double.MAX_VALUE
            var max = -Double.MAX_VALUE

            when (this) {
                X -> {
                    for (vertex in vertices) {
                        min = min(min, vertex.x)
                        max = max(max, vertex.x)
                    }
                }
                Y -> {
                    for (vertex in vertices) {
                        min = min(min, vertex.y)
                        max = max(max, vertex.y)
                    }
                }
                Z -> {
                    for (vertex in vertices) {
                        min = min(min, vertex.z)
                        max = max(max, vertex.z)
                    }
                }
            }

            return min to max
        }

        fun project(vertex: Vector3d): Double {
            return when (this) {
                X -> vertex.x
                Y -> vertex.y
                Z -> vertex.z
            }
        }
    }
}

private const val BB_MIN_X = 0
private const val BB_MIN_Y = 1
private const val BB_MIN_Z = 2
private const val BB_MAX_X = 3
private const val BB_MAX_Y = 4
private const val BB_MAX_Z = 5