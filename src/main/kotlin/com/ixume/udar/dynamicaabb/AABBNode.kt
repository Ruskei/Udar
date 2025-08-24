package com.ixume.udar.dynamicaabb

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.broadphase.BoundAABB
import org.bukkit.World
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class AABBNode(
    var parent: AABBNode?,
    //null if leaf
    var child1: AABBNode?,
    var child2: AABBNode?,
    var bb: AABB,
) : Comparable<AABBNode> {
    init {
        bb.node = this
    }

    val cost: Double
        get() {
            return if (isLeaf) bb.volume else AABB.unifiedCost(child1!!.bb, child2!!.bb)
        }

    val isLeaf: Boolean
        get() {
            return child1 == null
        }

    fun refittingCost(b: AABB): Double {
        return AABB.unifiedCost(bb, b) - cost
    }

    /**
     * Calculates refitting cost for this noe and all its parents
     */
    fun recursiveRefittingCost(b: AABB): Double {
        var c = 0.0
        var p: AABBNode? = this
        while (p != null) {
            c += p.refittingCost(b)
            p = p.parent
        }

        return c
    }

    fun refit(b: AABB) {
        bb.minX = min(bb.minX, b.minX)
        bb.minY = min(bb.minY, b.minY)
        bb.minZ = min(bb.minZ, b.minZ)
        bb.maxX = max(bb.maxX, b.maxX)
        bb.maxY = max(bb.maxY, b.maxY)
        bb.maxZ = max(bb.maxZ, b.maxZ)
    }

    private var _inherited = 0.0
    fun insertionCost(b: AABB): Double {
        _inherited = if (parent == null) 0.0 else parent!!.recursiveRefittingCost(b)
        return AABB.unifiedCost(bb, b) + _inherited
    }

    fun minChildrenCost(b: AABB): Double {
        return b.volume + refittingCost(b) + _inherited
    }

    fun visualize(world: World, depth: Int) {
        bb.visualize(world, depth)

        child1?.visualize(world, depth + 1)
        child2?.visualize(world, depth + 1)
    }

    var exploredCost: Double = -1.0

    override fun compareTo(other: AABBNode): Int {
        require(exploredCost != -1.0)
        require(other.exploredCost != -1.0)
        return sign(other.exploredCost - exploredCost).toInt()
    }

    fun contains(x: Double, y: Double, z: Double): Boolean {
        if (!bb.contains(x, y, z)) return false
        if (isLeaf) return false
        return child1!!.contains(x, y, z) || child2!!.contains(x, y, z)
    }

    companion object {
        fun pairs(a: AABBNode, b: AABBNode, ls: MutableMap<ActiveBody, MutableList<ActiveBody>>) {
            if (a.isLeaf) {
                if (b.isLeaf) {
                    val abb = a.bb as BoundAABB
                    val bbb = b.bb as BoundAABB

                    if (a.bb.overlaps(b.bb) && abb.body!!.tightBB.overlaps(bbb.body!!.tightBB)) {
                        ls.getOrPut(abb.body!!) { mutableListOf() } += bbb.body!!
                    }

                    return
                } else {
                    if (a.parent === b.parent) {
                        pairs(b.child1!!, b.child2!!, ls)
                    }

                    if (a.bb.overlaps(b.bb)) {
                        pairs(a, b.child1!!, ls)
                        pairs(a, b.child2!!, ls)
                    }
                }
            } else {
                if (b.isLeaf) {
                    if (a.parent === b.parent) {
                        pairs(a.child1!!, a.child2!!, ls)
                    }

                    if (a.bb.overlaps(b.bb)) {
                        pairs(b, a.child1!!, ls)
                        pairs(b, a.child2!!, ls)
                    }
                } else {
                    if (a.parent === b.parent) {
                        pairs(a.child1!!, a.child2!!, ls)
                        pairs(b.child1!!, b.child2!!, ls)
                    }

                    if (a.bb.overlaps(b.bb)) {
                        pairs(a.child1!!, b.child1!!, ls)
                        pairs(a.child1!!, b.child2!!, ls)
                        pairs(a.child2!!, b.child1!!, ls)
                        pairs(a.child2!!, b.child2!!, ls)
                    }
                }
            }
        }
    }
}