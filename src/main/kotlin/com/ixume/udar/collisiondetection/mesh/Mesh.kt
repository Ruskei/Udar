package com.ixume.udar.collisiondetection.mesh

import com.ixume.udar.testing.debugConnect
import org.bukkit.Color.*
import org.bukkit.Particle.DustOptions
import org.bukkit.World
import org.bukkit.util.BoundingBox
import org.bukkit.util.Vector
import org.joml.Vector2d
import org.joml.Vector3d
import org.joml.Vector3i
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

class Mesh(
    val start: Vector3i,
    val end: Vector3i,
    val faces: List<MeshFace>,
    val edges: List<Edge>,
) {
    fun visualize(world: World, visualizeFaces: Boolean, visualizeEdges: Boolean) {
        if (visualizeFaces) {
            val validInterval = 0.24
            val interval = 0.05
            for (face in faces) {
                when (face.axis) {
                    Axis.X -> {
                        continue
                        for ((start, end) in face.invalid) {
                            val min = Vector3d(face.start.x, start.x, start.y)
                            val max = Vector3d(face.start.x, end.x, end.y)

                            world.square(face.axis, min, max, DustOptions(RED, 0.5f), interval)
                        }

                        for ((start, end) in face.valid) {
                            val min = Vector3d(face.start.x, start.x, start.y)
                            val max = Vector3d(face.start.x, end.x, end.y)

                            world.square(face.axis, min, max, DustOptions(WHITE, 0.3f), validInterval)
                        }
                    }

                    Axis.Y -> {
                        continue
                        for ((start, end) in face.invalid) {
                            val min = Vector3d(face.start.y, start.x, start.y)
                            val max = Vector3d(face.start.y, end.x, end.y)

                            world.square(face.axis, min, max, DustOptions(RED, 0.5f), interval)
                        }

                        for ((start, end) in face.valid) {
                            val min = Vector3d(face.start.y, start.x, start.y)
                            val max = Vector3d(face.start.y, end.x, end.y)

                            world.square(face.axis, min, max, DustOptions(WHITE, 0.3f), validInterval)
                        }
                    }

                    Axis.Z -> {
                        for ((start, end) in face.invalid) {
                            val min = Vector3d(face.start.z, start.x, start.y)
                            val max = Vector3d(face.start.z, end.x, end.y)

                            world.square(face.axis, min, max, DustOptions(RED, 0.5f), interval)
                        }

                        for ((start, end) in face.valid) {
                            val min = Vector3d(face.start.z, start.x, start.y)
                            val max = Vector3d(face.start.z, end.x, end.y)

                            world.square(face.axis, min, max, DustOptions(WHITE, 0.3f), validInterval)
                        }
                    }
                }
            }
        }

        if (visualizeEdges) {
            for (edge in edges) {
                world.debugConnect(edge.start, edge.end, DustOptions(FUCHSIA, 0.25f), 0.1)
            }
        }
    }

    companion object {
        private const val OUTER = 1
        const val INNER = 1

        fun mesh(
            world: World,
            boundingBox: BoundingBox,
        ): Mesh {
            val meshStart = Vector3i(
                floor(boundingBox.minX).toInt() - OUTER,
                floor(boundingBox.minY).toInt() - OUTER,
                floor(boundingBox.minZ).toInt() - OUTER,
            )
            val meshEnd = Vector3i(
                floor(boundingBox.maxX).toInt() + OUTER,
                floor(boundingBox.maxY).toInt() + OUTER,
                floor(boundingBox.maxZ).toInt() + OUTER,
            )

            val boundingBoxes = mutableListOf<BoundingBox>()

            for (x in (meshStart.x)..(meshEnd.x)) {
                for (y in (meshStart.y)..(meshEnd.y)) {
                    for (z in (meshStart.z)..(meshEnd.z)) {
                        val block = world.getBlockAt(x, y, z)
                        if (block.isPassable) continue
                        for (boundingBox in block.collisionShape.boundingBoxes) {
                            val bb = boundingBox.clone()
                            bb.shift(block.location)
                            boundingBoxes += bb
                        }
                    }
                }
            }

            val meshFaces = mutableListOf<MeshFace>().apply {
                this += createMeshFaces(Axis.X, boundingBoxes, meshStart, meshEnd)
                this += createMeshFaces(Axis.Y, boundingBoxes, meshStart, meshEnd)
                this += createMeshFaces(Axis.Z, boundingBoxes, meshStart, meshEnd)
            }

            val edges = createMeshEdges(boundingBoxes)

            return Mesh(
                start = meshStart,
                end = meshEnd,
                faces = meshFaces,
                edges = edges,
            )
        }

        private fun createMeshFaces(
            axis: Axis,
            boundingBoxes: List<BoundingBox>,
            meshStart: Vector3i,
            meshEnd: Vector3i,
        ): MutableList<MeshFace> {
            fun Vector.a(): Double {
                return when (axis) {
                    Axis.X -> x
                    Axis.Y -> y
                    Axis.Z -> z
                }
            }

            fun Vector.b(): Double {
                return when (axis) {
                    Axis.X -> y
                    Axis.Y -> x
                    Axis.Z -> x
                }
            }

            fun Vector.c(): Double {
                return when (axis) {
                    Axis.X -> z
                    Axis.Y -> z
                    Axis.Z -> y
                }
            }

            fun Vector3i.a(): Int {
                return when (axis) {
                    Axis.X -> x
                    Axis.Y -> y
                    Axis.Z -> z
                }
            }

            fun Vector3i.b(): Int {
                return when (axis) {
                    Axis.X -> y
                    Axis.Y -> x
                    Axis.Z -> x
                }
            }

            fun Vector3i.c(): Int {
                return when (axis) {
                    Axis.X -> z
                    Axis.Y -> z
                    Axis.Z -> y
                }
            }

            fun Vector3d.a(): Double {
                return when (axis) {
                    Axis.X -> x
                    Axis.Y -> y
                    Axis.Z -> z
                }
            }

            fun Vector3d.b(): Double {
                return when (axis) {
                    Axis.X -> y
                    Axis.Y -> x
                    Axis.Z -> x
                }
            }

            fun Vector3d.c(): Double {
                return when (axis) {
                    Axis.X -> z
                    Axis.Y -> z
                    Axis.Z -> y
                }
            }

            fun Vector3d.deshuffle(): Vector3d {
                return when (axis) {
                    Axis.X -> Vector3d(x, y, z) //from x, y, z
                    Axis.Y -> Vector3d(y, x, z) //from y, x, z
                    Axis.Z -> Vector3d(y, z, x) //form z, x, y
                }
            }

            val epsilon = 1e-10
            val faces = mutableListOf<MeshFace>()

            //first encounter adds all passes
            //on every encounter, holes are added
            for (i in boundingBoxes.indices) {
                val bb1 = boundingBoxes[i]

                var doMin = false
                val minFace = faces.firstOrNull { abs(it.start.a() - bb1.min.a()) < epsilon } ?: let {
                    val f = MeshFace(
                        axis = axis,
                        start = Vector3d(bb1.min.a(), meshStart.b().toDouble(), meshStart.c().toDouble()).deshuffle(),
                        end = Vector3d(bb1.min.a(), meshEnd.b().toDouble(), meshEnd.c().toDouble()).deshuffle(),
                        valid = mutableListOf(
                            MeshFacePass(
                                Vector2d(bb1.min.b(), bb1.min.c()),
                                Vector2d(bb1.max.b(), bb1.max.c()),
                                false,

                                )
                        ),
                        invalid = mutableListOf(),
                        level = bb1.min.a(),
                    )

                    doMin = true
                    faces += f
                    f
                }

                var doMax = false
                val maxFace = faces.firstOrNull { abs(it.start.a() - bb1.max.a()) < epsilon } ?: let {
                    val f = MeshFace(
                        axis = axis,
                        start = Vector3d(bb1.max.a(), meshStart.b().toDouble(), meshStart.c().toDouble()).deshuffle(),
                        end = Vector3d(bb1.max.a(), meshEnd.b().toDouble(), meshEnd.c().toDouble()).deshuffle(),
                        valid = mutableListOf(
                            MeshFacePass(
                                Vector2d(bb1.min.b(), bb1.min.c()),
                                Vector2d(bb1.max.b(), bb1.max.c()),
                                true
                            )
                        ),
                        invalid = mutableListOf(),
                        level = bb1.max.a(),
                    )

                    doMax = true
                    faces += f
                    f
                }

                for (j in (i + 1)..<boundingBoxes.size) {
                    val bb2 = boundingBoxes[j]

                    val minMaxClose = abs(bb1.min.a() - bb2.max.a()) < epsilon
                    val isMinClose = abs(bb1.min.a() - bb2.min.a()) < epsilon
                    if (doMin && (minMaxClose || isMinClose)) {
                        minFace.valid += MeshFacePass(
                            Vector2d(bb2.min.b(), bb2.min.c()),
                            Vector2d(bb2.max.b(), bb2.max.c()),
                            !isMinClose
                        )
                    }

                    if (minMaxClose) {
                        overlap(
                            Vector2d(bb1.min.b(), bb1.min.c()),
                            Vector2d(bb1.max.b(), bb1.max.c()),
                            Vector2d(bb2.min.b(), bb2.min.c()),
                            Vector2d(bb2.max.b(), bb2.max.c()),
                        )?.let { minFace.invalid += it }
                    }

                    val maxMinClose = abs(bb1.max.a() - bb2.min.a()) < epsilon
                    val isMaxClose = abs(bb1.max.a() - bb2.max.a()) < epsilon
                    if (doMax && (maxMinClose || isMaxClose)) {
                        maxFace.valid += MeshFacePass(
                            Vector2d(bb2.min.b(), bb2.min.c()),
                            Vector2d(bb2.max.b(), bb2.max.c()),
                            isMaxClose
                        )
                    }

                    if (maxMinClose) {
                        overlap(
                            Vector2d(bb1.min.b(), bb1.min.c()),
                            Vector2d(bb1.max.b(), bb1.max.c()),
                            Vector2d(bb2.min.b(), bb2.min.c()),
                            Vector2d(bb2.max.b(), bb2.max.c()),
                        )?.let { maxFace.invalid += it }
                    }
                }
            }

            return faces
        }

        private fun createMeshEdges(
            boundingBoxes: List<BoundingBox>,
        ): List<Edge> {
            //iterate through all bounding boxes, xor the edges, done
            //store edges depending on axis
            //  - diff x,y,z structures
            //  - array for axiss

            val epsilon = 1e-8

            val xAxiss = mutableListOf<Pair<Vector2d, MutableList<Double>>>()
            val yAxiss = mutableListOf<Pair<Vector2d, MutableList<Double>>>()
            val zAxiss = mutableListOf<Pair<Vector2d, MutableList<Double>>>()

            for (bb in boundingBoxes) {
                val vertices = listOf(
                    Vector3d(bb.min.x, bb.min.y, bb.min.z),
                    Vector3d(bb.max.x, bb.min.y, bb.min.z),
                    Vector3d(bb.max.x, bb.min.y, bb.max.z),
                    Vector3d(bb.min.x, bb.min.y, bb.max.z),

                    Vector3d(bb.min.x, bb.max.y, bb.min.z),
                    Vector3d(bb.max.x, bb.max.y, bb.min.z),
                    Vector3d(bb.max.x, bb.max.y, bb.max.z),
                    Vector3d(bb.min.x, bb.max.y, bb.max.z),
                )

                val xEdges = listOf(
                    Edge(vertices[0], vertices[1]),
                    Edge(vertices[3], vertices[2]),
                    Edge(vertices[4], vertices[5]),
                    Edge(vertices[7], vertices[6]),
                )

                for (xEdge in xEdges) {
                    val matchingAxis =
                        xAxiss.firstOrNull { it.first.distance(xEdge.start.y, xEdge.start.z) < epsilon } ?: let {
                            val axis = Vector2d(xEdge.start.y, xEdge.start.z) to mutableListOf<Double>()
                            xAxiss += axis
                            axis
                        }
                    matchingAxis.second.xor(xEdge.start.x)
                    matchingAxis.second.xor(xEdge.end.x)
                }

                val yEdges = listOf(
                    Edge(vertices[0], vertices[4]),
                    Edge(vertices[1], vertices[5]),
                    Edge(vertices[2], vertices[6]),
                    Edge(vertices[3], vertices[7]),
                )

                for (yEdge in yEdges) {
                    val matchingAxis =
                        yAxiss.firstOrNull { it.first.distance(yEdge.start.x, yEdge.start.z) < epsilon } ?: let {
                            val axis = Vector2d(yEdge.start.x, yEdge.start.z) to mutableListOf<Double>()
                            yAxiss += axis
                            axis
                        }
                    matchingAxis.second.xor(yEdge.start.y)
                    matchingAxis.second.xor(yEdge.end.y)
                }

                val zEdges = listOf(
                    Edge(vertices[0], vertices[3]),
                    Edge(vertices[1], vertices[2]),
                    Edge(vertices[4], vertices[7]),
                    Edge(vertices[5], vertices[6]),
                )

                for (zEdge in zEdges) {
                    val matchingAxis =
                        zAxiss.firstOrNull { it.first.distance(zEdge.start.x, zEdge.start.y) < epsilon } ?: let {
                            val axis = Vector2d(zEdge.start.x, zEdge.start.y) to mutableListOf<Double>()
                            zAxiss += axis
                            axis
                        }
                    matchingAxis.second.xor(zEdge.start.z)
                    matchingAxis.second.xor(zEdge.end.z)
                }
            }

            val edges = mutableListOf<Edge>()

            for (axis in xAxiss) {
                require(axis.second.size % 2 == 0)
                axis.second.sort()
                for (i in axis.second.indices step 2) {
                    //don't add concave edges
                    val s = axis.second[i]
                    val e = axis.second[i + 1]

                    val x = s * 0.5 + e * 0.5
                    //categorize bounding boxes into 3d array if this gets too slow; though shouldn't be a problem for small objects
                    var mount: EdgeMount? = null
                    var sum = 0
                    for (bb in boundingBoxes) {
                        for (e in EdgeMount.entries) {
                            if (bb.contains(
                                    x,
                                    axis.first.x + 2.0 * epsilon * e.a,
                                    axis.first.y + 2.0 * epsilon * e.b,
                                )
                            ) {
                                sum++
                                mount = e
                            }
                        }

                        if (mount != null) break
                    }

                    if (sum == 1) {
                        edges += Edge(
                            start = Vector3d(s, axis.first.x, axis.first.y),
                            end = Vector3d(e, axis.first.x, axis.first.y),
                            mount = mount,
                            axis = Axis.X,
                        )
                    }
                }
            }

            for (axis in yAxiss) {
                require(axis.second.size % 2 == 0)
                axis.second.sort()
                for (i in axis.second.indices step 2) {
                    //don't add concave edges
                    val s = axis.second[i]
                    val e = axis.second[i + 1]

                    val y = s * 0.5 + e * 0.5
                    //categorize bounding boxes into 3d array if this gets too slow; though shouldn't be a problem for small objects
                    var mount: EdgeMount? = null
                    var sum = 0
                    for (bb in boundingBoxes) {
                        for (e in EdgeMount.entries) {
                            if (bb.contains(
                                    axis.first.x + 2.0 * epsilon * e.a,
                                    y,
                                    axis.first.y + 2.0 * epsilon * e.b,
                                )
                            ) {
                                sum++
                                mount = e
                            }
                        }

                        if (mount != null) break
                    }

                    if (sum == 1) {
                        edges += Edge(
                            start = Vector3d(axis.first.x, s, axis.first.y),
                            end = Vector3d(axis.first.x, e, axis.first.y),
                            mount = mount,
                            axis = Axis.Y,
                        )
                    }
                }
            }

            for (axis in zAxiss) {
                require(axis.second.size % 2 == 0)
                axis.second.sort()
                for (i in axis.second.indices step 2) {
                    //don't add concave edges
                    val s = axis.second[i]
                    val e = axis.second[i + 1]

                    val z = s * 0.5 + e * 0.5
                    //categorize bounding boxes into 3d array if this gets too slow; though shouldn't be a problem for small objects
                    var mount: EdgeMount? = null
                    var sum = 0
                    for (bb in boundingBoxes) {
                        for (e in EdgeMount.entries) {
                            if (bb.contains(
                                    axis.first.x + 2.0 * epsilon * e.a,
                                    axis.first.y + 2.0 * epsilon * e.b,
                                    z,
                                )
                            ) {
                                sum++
                                mount = e
                            }
                        }

                        if (mount != null) break
                    }

                    if (sum == 1) {
                        edges += Edge(
                            start = Vector3d(axis.first.x, axis.first.y, s),
                            end = Vector3d(axis.first.x, axis.first.y, e),
                            mount = mount,
                            axis = Axis.Z,
                        )
                    }
                }
            }

            return edges
        }

        private fun overlap(
            min1: Vector2d,
            max1: Vector2d,
            min2: Vector2d,
            max2: Vector2d,
        ): Pair<Vector2d, Vector2d>? {
            if ((min2.x < max1.x && min2.y < max1.y) && (max2.x > min1.x && max2.y > min1.y)) {
                return Vector2d(
                    max(min1.x, min2.x),
                    max(min1.y, min2.y),
                ) to Vector2d(
                    min(max1.x, max2.x),
                    min(max1.y, max2.y),
                )
            }

            return null
        }

        private fun MutableList<Double>.xor(elem: Double) {
            indexOfFirst { abs(it - elem) < 1e-10 }.takeUnless { it == -1 }?.let { removeAt(it) }
                ?: run { this += elem }
        }
    }
}

fun World.square(
    axis: Axis,
    min: Vector3d,
    max: Vector3d,
    options: DustOptions, interval: Double
) {
    fun Vector3d.deshuffle(): Vector3d {
        return when (axis) {
            Axis.X -> Vector3d(x, y, z)
            Axis.Y -> Vector3d(y, x, z)
            Axis.Z -> Vector3d(y, z, x)
        }
    }

    debugConnect(
        Vector3d(min.x, min.y, min.z).deshuffle(),
        Vector3d(min.x, max.y, min.z).deshuffle(),
        options, interval
    )
    debugConnect(
        Vector3d(min.x, max.y, min.z).deshuffle(),
        Vector3d(min.x, max.y, max.z).deshuffle(),
        options, interval
    )
    debugConnect(
        Vector3d(min.x, max.y, max.z).deshuffle(),
        Vector3d(min.x, min.y, max.z).deshuffle(),
        options, interval
    )
    debugConnect(
        Vector3d(min.x, min.y, max.z).deshuffle(),
        Vector3d(min.x, min.y, min.z).deshuffle(),
        options, interval
    )
}