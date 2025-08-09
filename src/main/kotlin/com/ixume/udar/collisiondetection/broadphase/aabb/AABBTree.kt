package com.ixume.udar.collisiondetection.broadphase.aabb

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.broadphase.aabb.AABBNode.Companion.pairs
import org.bukkit.World
import java.util.*

class AABBTree {
    var root: AABBNode? = null

    fun collisions(): Map<ActiveBody, List<ActiveBody>> {
        val r = root ?: return mapOf()
        if (r.isLeaf) return mapOf()

        val m = mutableMapOf<ActiveBody, MutableList<ActiveBody>>()
        pairs(r.child1!!, r.child2!!, m)

        return m
    }

    fun insert(bb: AABB) {
        val best = bestSibling(bb)
        if (best == null) {
            // root must be null
            root = AABBNode(
                parent = null,
                child1 = null,
                child2 = null,
                bb = bb,
            )

            return
        }

        val leaf = AABBNode(
            parent = null,
            child1 = null,
            child2 = null,
            bb = bb,
        )

        val oldParent = best.parent
        val newParent = AABBNode(
            parent = oldParent,
            child1 = null,
            child2 = null,
            bb = AABB.union(bb, best.bb),
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

    private fun rotate(p: AABBNode) {
        val pp = p.parent ?: return

        if (p.isLeaf) {
            return
        }

        val isFirst: Boolean
        val other: AABBNode
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
        val c1Swap = AABB.unifiedCost(other.bb, c2.bb)
        val c2Swap = AABB.unifiedCost(other.bb, c1.bb)

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

    private fun bestSibling(bb: AABB): AABBNode? {
        val cRoot = root ?: return null
        if (cRoot.isLeaf) return cRoot

        var bestCost = AABB.unifiedCost(bb, cRoot.bb)
        var bestNode = cRoot
        val queue = PriorityQueue<AABBNode>()

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

    fun visualize(world: World) {
        root?.visualize(world, 0)
    }
}