package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.collisiondetection.contactgeneration.worldmesh.MeshPosition
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher

class Meshes(
    val map: MutableMap<MeshPosition, LocalMesher.Mesh2> = mutableMapOf(),
    val meshes: MutableList<LocalMesher.Mesh2> = mutableListOf(),
) {

    fun addMesh(pos: MeshPosition, mesh: LocalMesher.Mesh2) {
        val prev = map.put(pos, mesh)
        if (prev != null) {
            meshes.remove(prev)
        }

        meshes += mesh
    }
    
    fun clone(): Meshes {
        return Meshes(map.toMutableMap(), meshes.toMutableList())
    }
}