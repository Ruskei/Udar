package com.ixume.udar.collisiondetection.mesh.mesh2

import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D

class MeshFaces(
    val xFaces: MeshFaceSortedList,
    val yFaces: MeshFaceSortedList,
    val zFaces: MeshFaceSortedList,
)

class Directed2DBBs(
    val axis: LocalMesher.AxisD,
    val bbs: List<AABB2D>,
)