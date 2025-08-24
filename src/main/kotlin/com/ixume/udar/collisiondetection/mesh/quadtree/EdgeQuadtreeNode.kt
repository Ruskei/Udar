package com.ixume.udar.collisiondetection.mesh.quadtree

import com.ixume.udar.collisiondetection.mesh.LocalMesher
import com.ixume.udar.dynamicaabb.AABBTree
import com.ixume.udar.testing.debugConnect
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector2d
import org.joml.Vector3d

/**
 *    |
 *  0 | 3
 *    |
 * -------
 *    |
 *  1 | 2
 *    |
 */

class EdgeQuadtreeNode(
    val min: Vector2d,
    val max: Vector2d,
) {
    val center = Vector2d(min.x / 2.0 + max.x / 2.0, min.y / 2.0 + max.y / 2.0)
    var children: Array<EdgeQuadtreeNode>? = null

    val points = DoubleArray(MAX_POINTS_PER_NODE * 2)
    val pointData = Array<QuadtreeEdge?>(MAX_POINTS_PER_NODE) { null }
    var numPoints = 0

    var isLeaf = true

    fun contains(a: Double, b: Double): Boolean {
        return a >= min.x && a < max.x && b >= min.y && b < max.y
    }

    fun insertEdge(
        a: Double,
        b: Double,
        start: Double,
        end: Double,
        axis: LocalMesher.AxisD,
        meshFaces: LocalMesher.MeshFaces
    ): Boolean {
        if (!contains(a, b)) return false

        if (isLeaf) {
            var i = 0
            while (i < numPoints) {
                val nx = points[i * 2]
                val ny = points[i * 2 + 1]

                if (nx == a && ny == b) { // found matching edge!
                    pointData[i]!!.xor(start, end)

                    return true
                }

                i++
            }

            // didn't find a matching edge, so we need to create new point or subdivide

            if (numPoints < MAX_POINTS_PER_NODE) {
                points[numPoints * 2] = a
                points[numPoints * 2 + 1] = b

                val f1 = when (axis) {
                    LocalMesher.AxisD.X -> meshFaces.yFaces.getFaceAt(a)
                    LocalMesher.AxisD.Y -> meshFaces.xFaces.getFaceAt(a)
                    LocalMesher.AxisD.Z -> meshFaces.xFaces.getFaceAt(a)
                }

                checkNotNull(f1)

                val f2 = when (axis) {
                    LocalMesher.AxisD.X -> meshFaces.zFaces.getFaceAt(b)
                    LocalMesher.AxisD.Y -> meshFaces.zFaces.getFaceAt(b)
                    LocalMesher.AxisD.Z -> meshFaces.yFaces.getFaceAt(b)
                }

                checkNotNull(f2)

                val edge = QuadtreeEdge(f1, f2)
                edge.xor(start, end)
                pointData[numPoints] = edge

                numPoints++
                return true
            }

            subdivide()
        }

        return (children!![0].insertEdge(a, b, start, end, axis, meshFaces)
                || children!![1].insertEdge(a, b, start, end, axis, meshFaces)
                || children!![2].insertEdge(a, b, start, end, axis, meshFaces)
                || children!![3].insertEdge(a, b, start, end, axis, meshFaces))
    }

    /**
     * Used during subdivision to insert an already formed edge
     */
    fun insertFormedEdge(x: Double, y: Double, edge: QuadtreeEdge): Boolean {
        if (!contains(x, y)) return false
        check(isLeaf)

        // no matching edges by construction
        if (numPoints < MAX_POINTS_PER_NODE) {
            points[numPoints * 2] = x
            points[numPoints * 2 + 1] = y

            pointData[numPoints] = edge

            numPoints++
            return true
        }

        subdivide()

        //this could happen if all the edges from parent still only fit in this one quadrant, in which case we need to repeat process
        return (children!![0].insertFormedEdge(x, y, edge)
                || children!![1].insertFormedEdge(x, y, edge)
                || children!![2].insertFormedEdge(x, y, edge)
                || children!![3].insertFormedEdge(x, y, edge))
    }

    private fun subdivide() {
        val c = Array<EdgeQuadtreeNode?>(4) {
            null
        }

        c[0] = EdgeQuadtreeNode(Vector2d(min.x, center.y), Vector2d(center.x, max.y))
        c[1] = EdgeQuadtreeNode(min, center)
        c[2] = EdgeQuadtreeNode(Vector2d(center.x, min.y), Vector2d(max.x, center.y))
        c[3] = EdgeQuadtreeNode(center, max)

        var i = 0
        while (i < numPoints) {
            c[0]!!.insertFormedEdge(points[i * 2], points[i * 2 + 1], pointData[i]!!)
            c[1]!!.insertFormedEdge(points[i * 2], points[i * 2 + 1], pointData[i]!!)
            c[2]!!.insertFormedEdge(points[i * 2], points[i * 2 + 1], pointData[i]!!)
            c[3]!!.insertFormedEdge(points[i * 2], points[i * 2 + 1], pointData[i]!!)

            i++
        }

        numPoints = 0

        children = c as Array<EdgeQuadtreeNode>

        isLeaf = false
    }

    fun getNeighbors(x: Double, y: Double, r: Double, out: MutableList<Vector2d>) {
        if (isLeaf) {
            var i = 0
            val _v = Vector2d()
            while (i < numPoints) {
                _v.set(points[i * 2], points[i * 2 + 1])
                if (_v.distance(x, y) < r) {
                    out += Vector2d(_v)
                }

                i++
            }
        } else {
            //figure out which quadrants to look at
            if (x < center.x + r) {
                if (y > center.y - r) {
                    children!![0].getNeighbors(x, y, r, out)
                }

                if (y < center.y + r) {
                    children!![1].getNeighbors(x, y, r, out)
                }
            }

            if (x > center.y - r) {
                if (y < center.y + r) {
                    children!![2].getNeighbors(x, y, r, out)
                }

                if (y > center.y - r) {
                    children!![3].getNeighbors(x, y, r, out)
                }
            }
        }
    }

    private val _vec3Mounts = DoubleArray(12)

    fun clearConcave(tree: AABBTree, axis: LocalMesher.AxisD) {
        if (isLeaf) {
            // go through every edge in every axis and check if its center is mounted to more than 1 block; if so, it's concave and should be removed
            var i = 0
            while (i < numPoints) {
                val data = pointData[i]!!
                val s = data.points.size
                check(s % 2 == 0) {
                    """
                        | Axis: $axis
                        | Edge points: ${data.points.joinToString { it.toString() }}
                        | Axis point: (${points[i * 2]}, ${points[i * 2 + 1]})
                    """.trimMargin()
                }

                val removedPoints = DoubleArrayList()

                _vec3Mounts[axis.aOffset] = points[i * 2] + MOUNT_EPSILON
                _vec3Mounts[axis.bOffset] = points[i * 2 + 1] + MOUNT_EPSILON

                _vec3Mounts[3 + axis.aOffset] = points[i * 2] + MOUNT_EPSILON
                _vec3Mounts[3 + axis.bOffset] = points[i * 2 + 1] - MOUNT_EPSILON

                _vec3Mounts[6 + axis.aOffset] = points[i * 2] - MOUNT_EPSILON
                _vec3Mounts[6 + axis.bOffset] = points[i * 2 + 1] - MOUNT_EPSILON

                _vec3Mounts[9 + axis.aOffset] = points[i * 2] - MOUNT_EPSILON
                _vec3Mounts[9 + axis.bOffset] = points[i * 2 + 1] + MOUNT_EPSILON

                val lvlOffset = axis.levelOffset

                val itr = data.points.iterator()

                while (itr.hasNext()) {
                    var mounted = false

                    val d0 = itr.nextDouble()
                    val d1 = itr.nextDouble()

                    val center = d0 / 2.0 + d1 / 2.0

                    var j = 0
                    while (j < 4) {
                        _vec3Mounts[j * 3 + lvlOffset] = center
                        if (tree.contains(_vec3Mounts[j * 3], _vec3Mounts[j * 3 + 1], _vec3Mounts[j * 3 + 2])) {
                            if (mounted) {
                                // concave...
                                removedPoints.add(d0)
                                removedPoints.add(d1)
                                break
                            } else {
                                mounted = true
                            }
                        }

                        j++
                    }
                }

                data.points.removeAll(removedPoints)

                i++
            }
        } else {
            var i = 0
            while (i < 4) {
                children!![i].clearConcave(tree, axis)
                i++
            }
        }
    }

    fun visualize(world: World, axis: LocalMesher.AxisD) {
        if (isLeaf) {
            var i = 0
            while (i < numPoints) {
                val a = points[i * 2]
                val b = points[i * 2 + 1]
                val data = pointData[i]!!
                check(data.points.size % 2 == 0)

                val arr1 = DoubleArray(3)
                arr1[axis.aOffset] = a
                arr1[axis.bOffset] = b
                val arr2 = DoubleArray(3)
                arr2[axis.aOffset] = a
                arr2[axis.bOffset] = b

                val itr = data.points.iterator()
                while (itr.hasNext()) {
                    val d1 = itr.nextDouble()
                    val d2 = itr.nextDouble()

                    arr1[axis.levelOffset] = d1
                    arr2[axis.levelOffset] = d2

                    world.debugConnect(
                        start = Vector3d(arr1),
                        end = Vector3d(arr2),
                        options = Particle.DustOptions(Color.YELLOW, 0.3f),
                    )
                }

                i++
            }
        } else {
            children!!.forEach { it.visualize(world, axis) }
        }
    }

    companion object {
        const val MAX_POINTS_PER_NODE = 4
        private const val MOUNT_EPSILON = 1e-6
    }
}