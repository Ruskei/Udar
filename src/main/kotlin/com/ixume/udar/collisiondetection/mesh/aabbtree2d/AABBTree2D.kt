package com.ixume.udar.collisiondetection.mesh.aabbtree2d

import com.ixume.udar.collisiondetection.mesh.LocalMesher
import org.bukkit.World
import java.util.PriorityQueue

/**
 * Not thread safe.
 */
class AABBTree2D {
    var root: AABBNode2D? = null

    fun collisions(): AABBTree2D {
        val collisionTree = AABBTree2D()

        val r = root
        if (r == null) {
            return collisionTree
        }

        if (r.isLeaf) {
            return collisionTree
        }

        AABBNode2D.Companion.pairs(r.child1!!, r.child2!!, collisionTree)

        return collisionTree
    }

    fun insert(bb: AABB2D) {
        val best = bestSibling(bb)
        if (best == null) {
            // root must be null
            root = AABBNode2D(
                parent = null,
                child1 = null,
                child2 = null,
                bb = bb,
            )

            return
        }

        val leaf = AABBNode2D(
            parent = null,
            child1 = null,
            child2 = null,
            bb = bb,
        )

        val oldParent = best.parent
        val newParent = AABBNode2D(
            parent = oldParent,
            child1 = null,
            child2 = null,
            bb = AABB2D.union(bb, best.bb),
        )

        if (oldParent == null) {
            //root
            newParent.child1 = best
            newParent.child2 = leaf
            leaf.parent = newParent
            best.parent = newParent

            root = newParent
        } else {
            if (oldParent.child1 == best) {
                oldParent.child1 = newParent
            } else {
                oldParent.child2 = newParent
            }

            newParent.child1 = best
            newParent.child2 = leaf
            best.parent = newParent
            leaf.parent = newParent
        }

        var p = leaf.parent
        while (p != null) {
            p.refit(bb)
            rotate(p)

            p = p.parent
        }
    }

    private fun rotate(p: AABBNode2D) {
        val pp = p.parent ?: return

        if (p.isLeaf) {
            return
        }

        val isFirst: Boolean
        val other: AABBNode2D
        if (p == pp.child1) {
            isFirst = true
            other = pp.child2!!
        } else {
            isFirst = false
            other = pp.child1!!
        }

        val c1 = p.child1!!
        val c2 = p.child2!!

        val curr = p.cost
        val c1Swap = AABB2D.unifiedCost(other.bb, c2.bb)
        val c2Swap = AABB2D.unifiedCost(other.bb, c1.bb)

        if (c1Swap < curr) {
            p.child1 = other
            other.parent = p

            if (isFirst) {
                pp.child2 = c1
            } else {
                pp.child1 = c1
            }

            c1.parent = pp
            p.bb.setUnion(p.child1!!.bb, p.child2!!.bb)
        } else if (c2Swap < curr) {
            p.child2 = other
            other.parent = p

            if (isFirst) {
                pp.child2 = c2
            } else {
                pp.child1 = c2
            }

            c2.parent = pp
            p.bb.setUnion(p.child1!!.bb, p.child2!!.bb)
        }
    }

    val queue = PriorityQueue<AABBNode2D>()
    private fun bestSibling(bb: AABB2D): AABBNode2D? {
        val cRoot = root ?: return null
        if (cRoot.isLeaf) return cRoot

        var bestCost = AABB2D.unifiedCost(bb, cRoot.bb)
        var bestNode = cRoot

        queue.clear()
        queue += cRoot

        while (queue.isNotEmpty()) {
            val node = queue.poll()!!
            val c = node.insertionCost(bb)
            if (c < bestCost) {
                bestNode = node
                bestCost = c
            }

            if (!node.isLeaf) {
                val e = node.minChildrenCost(bb)
                if (e < bestCost) {
                    node.child1!!.exploredCost = e
                    node.child2!!.exploredCost = e

                    queue += node.child1!!
                    queue += node.child2!!
                }
            }
        }

        return bestNode
    }

    fun remove(node: AABBNode2D) {
        val cp = node.parent

        if (cp == null) {
            root = null

            return
        }

        val cpc1 = cp.child1
        val cpc2 = cp.child2
        if (cpc1 == null || cpc2 == null) {
            throw IllegalStateException("YK! 1: $cpc1 2: $cpc2")
        }

        val other = if (cp.child1 === node) cp.child2!! else cp.child1!!

        val cpp = cp.parent
        if (cpp == null) {
            root = other
            other.parent = null

            return
        }

        if (cpp.child1 == null || cpp.child2 == null) {
            throw IllegalStateException("YK 2! 1: $${cpp.child1} 2: ${cpp.child2}")
        }

        if (cpp.child1 === cp) {
            cpp.child1 = other
        } else {
            cpp.child2 = other
        }

        cp.child1 = null
        cp.child2 = null
        other.parent = cpp
    }

    fun contains(x: Double, y: Double): Boolean {
        return root?.isFilledAt(x, y) ?: false
    }

    fun visualize(world: World, level: Double, axis: LocalMesher.AxisD, isHole: Boolean) {
        root?.visualize(world, 0, level, axis, isHole)
    }

    fun clear() {
        root = null // and let GC do the rest :)
        //TODO: use cleaner api maybe to help this along...
    }
}