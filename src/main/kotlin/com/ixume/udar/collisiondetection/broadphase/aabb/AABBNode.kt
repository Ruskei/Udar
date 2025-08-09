package com.ixume.udar.collisiondetection.broadphase.aabb

import com.ixume.udar.body.active.ActiveBody
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
            check((child1 == null) == (child2 == null))
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

    fun insertionCost(b: AABB): Double {
        val inherited = if (parent == null) 0.0 else parent!!.recursiveRefittingCost(b)
        return AABB.unifiedCost(bb, b) + inherited
    }

    fun minChildrenCost(b: AABB): Double {
        return b.volume + recursiveRefittingCost(b)
    }

    fun remove(tree: AABBTree) {
        if (parent == null) {
            tree.root = null
            return
        }

        val cp = parent!!
        val cpc1 = cp.child1
        val cpc2 = cp.child2
        if (cpc1 == null || cpc2 == null) {
            throw IllegalStateException("YK! 1: $cpc1 2: $cpc2")
        }

        val other = if (cp.child1 == this) cp.child2!! else cp.child1!!

        val cpp = cp.parent
        if (cpp == null) {
            tree.root = other
            other.parent = null
            return
        }

        if (cpp.child1 == cp) {
            cpp.child1 = other
        } else {
            cpp.child2 = other
        }

        cp.child1 = null
        cp.child2 = null
        other.parent = cpp
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

    companion object {
        fun pairs(a: AABBNode, b: AABBNode, ls: MutableMap<ActiveBody, MutableList<ActiveBody>>) {
            if (a.isLeaf) {
                if (b.isLeaf) {
                    if (a.bb.overlaps(b.bb) && a.bb.body!!.tightBB.overlaps(b.bb.body!!.tightBB)) {
                        ls.getOrPut(a.bb.body!!) { mutableListOf() } += b.bb.body!!
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