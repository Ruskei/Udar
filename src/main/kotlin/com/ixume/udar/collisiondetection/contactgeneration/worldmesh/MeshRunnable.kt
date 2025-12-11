package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.AABB
import kotlin.system.measureNanoTime

class MeshRunnable : Runnable {
    lateinit var mps: List<MeshPosition>

    var start = 0
    var end = 0

    val out = mutableListOf<LocalMesher.Mesh2>()
    val mesher = LocalMesher()

    override fun run() {
        if (start >= end) {
            return
        }

        var i = start
        while (i < end) {
            val mp = mps[i]
            out += mp.genMesh()

            i++
        }
    }

    fun MeshPosition.genMesh(): LocalMesher.Mesh2 {
        val meshBB = AABB(
            minX = x * MESH_SIZE.toDouble(),
            minY = y * MESH_SIZE.toDouble(),
            minZ = z * MESH_SIZE.toDouble(),
            maxX = x * MESH_SIZE.toDouble() + MESH_SIZE,
            maxY = y * MESH_SIZE.toDouble() + MESH_SIZE,
            maxZ = z * MESH_SIZE.toDouble() + MESH_SIZE,
        )

        val mesh: LocalMesher.Mesh2

        val t = measureNanoTime {
            mesh = mesher.mesh(
                world = world,
                boundingBox = meshBB
            )
        }

        return mesh
    }
}