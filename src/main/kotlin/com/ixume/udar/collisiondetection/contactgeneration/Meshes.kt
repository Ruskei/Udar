package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.AtomicList
import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.MeshPosition
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher

class Meshes {
    private val meshes = AtomicList<LocalMesher.Mesh2>()
    val map: MutableMap<MeshPosition, LocalMesher.Mesh2> = mutableMapOf()

    fun getMeshes(): List<LocalMesher.Mesh2> {
        return meshes.get()
    }

    fun addMesh(pos: MeshPosition, mesh: LocalMesher.Mesh2) {
        val prev = map.put(pos, mesh)
        if (prev != null) {
            meshes.remove(prev)
        }

        meshes += mesh
    }

    fun removeMesh(pos: MeshPosition) {
        val mesh = map[pos]
        if (mesh != null) {
            meshes -= mesh
            map.remove(pos)
        }
    }
}