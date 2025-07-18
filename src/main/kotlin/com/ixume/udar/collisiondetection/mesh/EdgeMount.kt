package com.ixume.udar.collisiondetection.mesh

enum class EdgeMount(
    val a: Double,
    val b: Double,
) {
    NEGNEG(-1.0, -1.0),
    NEGPOS(-1.0, 1.0),
    POSNEG(1.0, -1.0),
    POSPOS(1.0, 1.0),
}
