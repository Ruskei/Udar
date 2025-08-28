package com.ixume.udar.collisiondetection.mesh.aabbtree2d

import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import org.bukkit.World
import kotlin.math.sign

class AABBNode2D(
    var parent: AABBNode2D?,
    //null if leaf
    var child1: AABBNode2D?,
    var child2: AABBNode2D?,
    var bb: AABB2D,
) : Comparable<AABBNode2D> {
    init {
        bb.node = this
    }

    val cost: Double
        get() {
            return if (isLeaf) bb.volume else AABB2D.unifiedCost(child1!!.bb, child2!!.bb)
        }

    val isLeaf: Boolean
        get() {
            return child1 == null
        }

    fun refittingCost(b: AABB2D): Double {
        return AABB2D.unifiedCost(bb, b) - cost
    }

    /**
     * Calculates refitting cost for this noe and all its parents
     */
    fun recursiveRefittingCost(b: AABB2D): Double {
        var c = 0.0
        var p: AABBNode2D? = this
        while (p != null) {
            c += p.refittingCost(b)
            p = p.parent
        }

        return c
    }

    fun refit(b: AABB2D) {
        bb.refit(b)
    }


    private var _inherited = 0.0
    fun insertionCost(b: AABB2D): Double {
        _inherited = if (parent == null) 0.0 else parent!!.recursiveRefittingCost(b)
        return AABB2D.unifiedCost(bb, b) + _inherited
    }

    fun minChildrenCost(b: AABB2D): Double {
        return b.volume + refittingCost(b) + _inherited
    }

    var exploredCost: Double = -1.0

    override fun compareTo(other: AABBNode2D): Int {
//        require(exploredCost != -1.0)
//        require(other.exploredCost != -1.0)
        return sign(exploredCost - other.exploredCost).toInt()
    }

    fun overlaps(bb: AABB2D, out: MutableList<AABB2D>) {
        if (!bb.overlaps(this.bb)) return
        if (isLeaf) {
            out += this.bb
            return
        }

        child1!!.overlaps(bb, out)
        child2!!.overlaps(bb, out)
    }

    fun visualize(world: World, depth: Int, level: Double, axis: LocalMesher.AxisD, isHole: Boolean) {
        if (isLeaf) {
            bb.visualize(world, depth, level, axis, isHole)
        }

        child1?.visualize(world, depth + 1, level, axis, isHole)
        child2?.visualize(world, depth + 1, level, axis, isHole)
    }

    companion object {
        fun pairs(a: AABBNode2D, b: AABBNode2D, out: AABBTree2D) {
            if (a.isLeaf) {
                if (b.isLeaf) {
                    if (a.bb.overlaps(b.bb)) {
                        //insert overlap, the anti-hole
                        out.insert(a.bb.calcOverlap(b.bb))
                    }

                    return
                } else {
                    if (a.parent === b.parent) {
                        pairs(b.child1!!, b.child2!!, out)
                    }

                    if (a.bb.overlaps(b.bb)) {
                        pairs(a, b.child1!!, out)
                        pairs(a, b.child2!!, out)
                    }
                }
            } else {
                if (b.isLeaf) {
                    if (a.parent === b.parent) {
                        pairs(a.child1!!, a.child2!!, out)
                    }

                    if (a.bb.overlaps(b.bb)) {
                        pairs(b, a.child1!!, out)
                        pairs(b, a.child2!!, out)
                    }
                } else {
                    if (a.parent === b.parent) {
                        pairs(a.child1!!, a.child2!!, out)
                        pairs(b.child1!!, b.child2!!, out)
                    }

                    if (a.bb.overlaps(b.bb)) {
                        pairs(a.child1!!, b.child1!!, out)
                        pairs(a.child1!!, b.child2!!, out)
                        pairs(a.child2!!, b.child1!!, out)
                        pairs(a.child2!!, b.child2!!, out)
                    }
                }
            }
        }
    }
}