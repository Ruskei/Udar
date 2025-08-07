package com.ixume.udar.collisiondetection

data class MutableBB(
    var minX: Double,
    var minY: Double,
    var minZ: Double,
    var maxX: Double,
    var maxY: Double,
    var maxZ: Double,
) {
    fun overlaps(other: MutableBB): Boolean {
        return overlaps(other.minX, other.minY, other.minZ, other.maxX, other.maxY, other.maxZ)
    }
    
    fun overlaps(minX: Double, minY: Double, minZ: Double, maxX: Double, maxY: Double, maxZ: Double): Boolean {
        return this.minX < maxX && this.maxX > minX
                && this.minY < maxY && this.maxY > minY
                && this.minZ < maxZ && this.maxZ > minZ
    }
}