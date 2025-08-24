package com.ixume.udar.dynamicaabb

import com.ixume.udar.body.active.ActiveBody
import org.bukkit.World
import java.util.*
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

class AABBTree {
    private val blocked = AtomicBoolean(false)
    private val status = AtomicInteger(0)
    var root: AABBNode? = null

    fun collisions(): Map<ActiveBody, List<ActiveBody>> {
        if (!blocked.compareAndSet(false, true)) throw IllegalStateException("Tried to get collisions while blocked! STATUS: ${status.get()}")
        status.set(1)

        val r = root
        if (r == null) {
            status.set(0)
            blocked.set(false)

            return emptyMap()
        }

        if (r.isLeaf) {
            status.set(0)
            blocked.set(false)

            return emptyMap()
        }

        val m = mutableMapOf<ActiveBody, MutableList<ActiveBody>>()
        AABBNode.Companion.pairs(r.child1!!, r.child2!!, m)

        status.set(0)
        blocked.set(false)

        return m
    }

    fun insert(bb: AABB) {
        if (!blocked.compareAndSet(false, true)) throw IllegalStateException("Tried to insert while blocked! STATUS: ${status.get()}")
        status.set(2)

        val best = bestSibling(bb)
        if (best == null) {
            // root must be null
            root = AABBNode(
                parent = null,
                child1 = null,
                child2 = null,
                bb = bb,
            )

            status.set(0)
            blocked.set(false)
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

        status.set(0)
        blocked.set(false)
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

    fun remove(node: AABBNode) {
        if (!blocked.compareAndSet(false, true)) throw IllegalStateException("Tried to remove while blocked! STATUS: ${status.get()}")
        status.set(3)

        val cp = node.parent

        if (cp == null) {
            root = null

            status.set(0)
            blocked.set(false)
            return
        }

        val cpc1 = cp.child1
        val cpc2 = cp.child2
        if (cpc1 == null || cpc2 == null) {
            status.set(0)
            blocked.set(false)
            throw IllegalStateException("YK! 1: $cpc1 2: $cpc2")
        }

        val other = if (cp.child1 === node) cp.child2!! else cp.child1!!

        val cpp = cp.parent
        if (cpp == null) {
            root = other
            other.parent = null

            status.set(0)
            blocked.set(false)
            return
        }

        if (cpp.child1 == null || cpp.child2 == null) {
            status.set(0)
            blocked.set(false)
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

        status.set(0)
        blocked.set(false)
    }

    fun clear() {
        root = null // and let GC do the rest :)
        //TODO: use cleaner api maybe to help this along...
    }

    fun contains(x: Double, y: Double, z: Double): Boolean {
        return root?.contains(x, y, z) ?: false
    }

    fun visualize(world: World) {
        root?.visualize(world, 0)
    }
}