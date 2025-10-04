package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.MESH_SIZE
import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.rollingVec3Checksum
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.dynamicaabb.array.FlattenedAABBTree
import org.bukkit.World
import org.joml.Vector3d
import org.joml.Vector3i
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

class LocalMesher {
    //    private val _flatTree = FlattenedAABBTree(0)
    private val _bbs = BBList()

    private lateinit var _xAxiss2: FlattenedEdgeQuadtree
    private lateinit var _yAxiss2: FlattenedEdgeQuadtree
    private lateinit var _zAxiss2: FlattenedEdgeQuadtree

    /*
    since meshes are tiled, we deal with boundary issues by having each mesh only worry about its minimum bounds
     */

    fun mesh(
        world: World,
        boundingBox: AABB,
//        experimental: Boolean = false,
    ): Mesh2 {
//        println("GENERATING!")
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

//        println("| meshStart: $meshStart")
//        println("| meshEnd: $meshEnd")
//
        val flatTree = FlattenedAABBTree(0)
        _bbs.clear()

        val width = meshEnd.x - meshStart.x + 1
        val height = meshEnd.y - meshStart.y + 1
        val length = meshEnd.z - meshStart.z + 1

        val state = LongArray(width * height * length)

        for (x in (meshStart.x - 1)..(meshEnd.x + 1)) {
            for (y in (meshStart.y - 1)..(meshEnd.y + 1)) {
                for (z in (meshStart.z - 1)..(meshEnd.z + 1)) {
                    val block = world.getBlockAt(x, y, z)
                    val mine =
                        x in meshStart.x..meshEnd.x &&
                        y in meshStart.y..meshEnd.y &&
                        z in meshStart.z..meshEnd.z


                    if (block.isPassable) continue

                    var sum = 1L

                    for (boundingBox in block.collisionShape.boundingBoxes) {
                        val bb = boundingBox.clone()
                        bb.shift(block.location)
                        val aabb = AABB(bb.minX, bb.minY, bb.minZ, bb.maxX, bb.maxY, bb.maxZ)

                        flatTree.insert(
                            minX = bb.minX,
                            minY = bb.minY,
                            minZ = bb.minZ,
                            maxX = bb.maxX,
                            maxY = bb.maxY,
                            maxZ = bb.maxZ,
                        )

                        _bbs.add(aabb)

                        sum = rollingVec3Checksum(
                            sum,
                            x + boundingBox.minX,
                            y + boundingBox.minY,
                            z + boundingBox.minZ,
                        )
                        sum = rollingVec3Checksum(
                            sum,
                            x + boundingBox.maxX,
                            y + boundingBox.maxY,
                            z + boundingBox.maxZ,
                        )
                    }

                    if (mine) {
                        state[(z - meshStart.z) + (y - meshStart.y) * length + (x - meshStart.x) * length * height] =
                            sum
                    }
                }
            }
        }

//        println("${_bbs.size} BBs!")

        if (_bbs.size == 0) {
            return Mesh2(
                start = meshStart,
                end = meshEnd,
                flatTree = null,
                state = state,
            )
        }

        val meshFaces = MeshFaces(
            createMeshFaces(AxisD.X, meshStart, meshEnd, flatTree),
            createMeshFaces(AxisD.Y, meshStart, meshEnd, flatTree),
            createMeshFaces(AxisD.Z, meshStart, meshEnd, flatTree),
        )

        createMeshEdges2(meshStart, meshEnd, meshFaces, flatTree)

        return Mesh2(
            start = meshStart,
            end = meshEnd,
            faces = meshFaces,

            xEdges2 = _xAxiss2,
            yEdges2 = _yAxiss2,
            zEdges2 = _zAxiss2,
            flatTree = flatTree,
            state = state,
        )
    }

    private fun createMeshFaces(
        axis: AxisD,
        meshStart: Vector3i,
        meshEnd: Vector3i,
        tree: FlattenedAABBTree,
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

        val faces = MeshFaceSortedList(axis, meshStart, meshEnd)

        val _bb = DoubleArray(6)

        val itr = tree.iterator()

        while (itr.hasNext()) {
            itr.next(_bb)

            val minFace = faces.placeFaceAt(_bb.minLevel(axis))
            minFace?.stamp(_bb, false)


            val maxFace = faces.placeFaceAt(_bb.maxLevel(axis))
            maxFace?.stamp(_bb, true)
        }

        faces.finish()

        return faces
    }

    private fun MeshFace.stamp(bb: DoubleArray, inDir: Boolean) {
        val axis = axis

        holes.insert(
            minX = bb.minA(axis),
            minY = bb.minB(axis),
            maxX = bb.maxA(axis),
            maxY = bb.maxB(axis),
        )
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

    private fun createMeshEdges2(
        meshStart: Vector3i,
        meshEnd: Vector3i,
        meshFaces: MeshFaces,
        tree: FlattenedAABBTree,
    ) {
        val epsilon = 0
        //TODO: Dont put out of bounds points on edges
        _xAxiss2 = FlattenedEdgeQuadtree(
            AxisD.X,
            meshStart.y.toDouble() - epsilon, meshStart.z.toDouble() - epsilon,
            meshEnd.y.toDouble() + 2.0 + epsilon, meshEnd.z.toDouble() + 2.0 + epsilon,
            levelMin = meshStart.x.toDouble() - epsilon,
            levelMax = meshEnd.x.toDouble() + 1.0 + epsilon,
        )
        _yAxiss2 = FlattenedEdgeQuadtree(
            AxisD.Y,
            meshStart.x.toDouble() - epsilon, meshStart.z.toDouble() - epsilon,
            meshEnd.x.toDouble() + 2.0 + epsilon, meshEnd.z.toDouble() + 2.0 + epsilon,
            levelMin = meshStart.y.toDouble() - epsilon,
            levelMax = meshEnd.y.toDouble() + 1.0 - epsilon,
        )
        _zAxiss2 = FlattenedEdgeQuadtree(
            AxisD.Z,
            meshStart.x.toDouble() - epsilon, meshStart.y.toDouble() - epsilon,
            meshEnd.x.toDouble() + 2.0 + epsilon, meshEnd.y.toDouble() + 2.0 + epsilon,
            levelMin = meshStart.z.toDouble() - epsilon,
            levelMax = meshEnd.z.toDouble() + 1.0 + epsilon,
        )

        val _bb = DoubleArray(6) // [ minX, minY, minZ, maxX, maxY, maxZ ]
        val itr = tree.iterator()

        while (itr.hasNext()) {
            itr.next(_bb)

            //x axis
            val xStart = _bb[BB_MIN_X]
            val xEnd = _bb[BB_MAX_X]

            //y axis
            val yStart = _bb[BB_MIN_Y]
            val yEnd = _bb[BB_MAX_Y]

            //z axis
            val zStart = _bb[BB_MIN_Z]
            val zEnd = _bb[BB_MAX_Z]

            val x0aStart = _bb[BB_MIN_Y]
            val x0bStart = _bb[BB_MIN_Z]
            _xAxiss2.insertEdge(x0aStart, x0bStart, xStart, xEnd, meshFaces)

            val x1aStart = _bb[BB_MAX_Y]
            val x1bStart = _bb[BB_MIN_Z]
            _xAxiss2.insertEdge(x1aStart, x1bStart, xStart, xEnd, meshFaces)

            val x2aStart = _bb[BB_MAX_Y]
            val x2bStart = _bb[BB_MAX_Z]
            _xAxiss2.insertEdge(x2aStart, x2bStart, xStart, xEnd, meshFaces)

            val x3aStart = _bb[BB_MIN_Y]
            val x3bStart = _bb[BB_MAX_Z]
            _xAxiss2.insertEdge(x3aStart, x3bStart, xStart, xEnd, meshFaces)

            val y0aStart = _bb[BB_MIN_X]
            val y0bStart = _bb[BB_MIN_Z]
            _yAxiss2.insertEdge(y0aStart, y0bStart, yStart, yEnd, meshFaces)

            val y1aStart = _bb[BB_MAX_X]
            val y1bStart = _bb[BB_MIN_Z]
            _yAxiss2.insertEdge(y1aStart, y1bStart, yStart, yEnd, meshFaces)

            val y2aStart = _bb[BB_MAX_X]
            val y2bStart = _bb[BB_MAX_Z]
            _yAxiss2.insertEdge(y2aStart, y2bStart, yStart, yEnd, meshFaces)

            val y3aStart = _bb[BB_MIN_X]
            val y3bStart = _bb[BB_MAX_Z]
            _yAxiss2.insertEdge(y3aStart, y3bStart, yStart, yEnd, meshFaces)


            val z0aStart = _bb[BB_MIN_X]
            val z0bStart = _bb[BB_MIN_Y]
            _zAxiss2.insertEdge(z0aStart, z0bStart, zStart, zEnd, meshFaces)

            val z1aStart = _bb[BB_MAX_X]
            val z1bStart = _bb[BB_MIN_Y]
            _zAxiss2.insertEdge(z1aStart, z1bStart, zStart, zEnd, meshFaces)

            val z2aStart = _bb[BB_MAX_X]
            val z2bStart = _bb[BB_MAX_Y]
            _zAxiss2.insertEdge(z2aStart, z2bStart, zStart, zEnd, meshFaces)

            val z3aStart = _bb[BB_MIN_X]
            val z3bStart = _bb[BB_MAX_Y]
            _zAxiss2.insertEdge(z3aStart, z3bStart, zStart, zEnd, meshFaces)
        }

        _xAxiss2.fixUp(tree, meshFaces)
        _yAxiss2.fixUp(tree, meshFaces)
        _zAxiss2.fixUp(tree, meshFaces)
    }

    class Mesh2(
        val start: Vector3i,
        val end: Vector3i,
        val faces: MeshFaces? = null,

        val xEdges2: FlattenedEdgeQuadtree? = null,
        val yEdges2: FlattenedEdgeQuadtree? = null,
        val zEdges2: FlattenedEdgeQuadtree? = null,
        val flatTree: FlattenedAABBTree?,

        val state: LongArray,
    ) {
        val mp = Vector3i(start).div(MESH_SIZE)
        val height = end.y - start.y + 1
        val length = end.z - start.z + 1

        fun visualize(world: World) {
//            println("visualizing mesh")
            xEdges2?.visualize(world)
            yEdges2?.visualize(world)
            zEdges2?.visualize(world)
//            flatTree?.visualize(world)

//            faces?.xFaces?.ls?.forEach { it.visualize(world) }
//            faces?.yFaces?.ls?.forEach { it.visualize(world) }
//            faces?.zFaces?.ls?.forEach { it.visualize(world) }
////
//            xEdges?.visualize(world)
//            yEdges?.visualize(world)
//            zEdges?.visualize(world)
        }

        fun stateAt(x: Int, y: Int, z: Int): Long {
            return state[(z - start.z) + (y - start.y) * length + (x - start.x) * height * length]
        }

        fun relevant(
            minX: Double,
            minY: Double,
            minZ: Double,

            maxX: Double,
            maxY: Double,
            maxZ: Double,
        ): Boolean {
            return maxX >= start.x && minX <= end.x + 1 &&
                   maxY >= start.y && minY <= end.y + 1 &&
                   maxZ >= start.z && minZ <= end.z + 1
        }

        fun minLevel(axis: AxisD): Int {
            return when (axis) {
                AxisD.X -> start.x
                AxisD.Y -> start.y
                AxisD.Z -> start.z
            }
        }

        fun minA(axis: AxisD): Int {
            return when (axis) {
                AxisD.X -> start.y
                AxisD.Y -> start.x
                AxisD.Z -> start.x
            }
        }

        fun minB(axis: AxisD): Int {
            return when (axis) {
                AxisD.X -> start.z
                AxisD.Y -> start.z
                AxisD.Z -> start.y
            }
        }

        override fun equals(other: Any?): Boolean {
            return other != null && other is Mesh2 && other.start == start
        }

        override fun hashCode(): Int {
            val p1 = 60727
            val p2 = 63907
            val p3 = 90803

            return (mp.x and 0b11)
                .or((mp.y and 0b11) shl 2)
                .or((mp.z and 0b11) shl 4)
                .or(((mp.x * p1) xor (mp.y * p2) xor (mp.z * p3)) and (0b111111.inv()))
        }
    }

    enum class AxisD(
        val vec: Vector3d,
        val levelOffset: Int,
        val aOffset: Int,
        val bOffset: Int,
    ) {
        X(Vector3d(1.0, 0.0, 0.0), 0, 1, 2), Y(Vector3d(0.0, 1.0, 0.0), 1, 0, 2), Z(Vector3d(0.0, 0.0, 1.0), 2, 0, 1);

        fun project(
            vertices: Array<Vector3d>,
            out: DoubleArray,
        ) {
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

            out[0] = min
            out[1] = max
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

val axiss = arrayOf(
    LocalMesher.AxisD.X,
    LocalMesher.AxisD.Y,
    LocalMesher.AxisD.Z,
)


private const val BB_MIN_X = 0
private const val BB_MIN_Y = 1
private const val BB_MIN_Z = 2
private const val BB_MAX_X = 3
private const val BB_MAX_Y = 4
private const val BB_MAX_Z = 5