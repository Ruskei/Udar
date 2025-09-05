package com.ixume.udar.collisiondetection.mesh.aabbtree2d

import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.aabbtree2d.AABB2D.Companion.withLevel
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.array.IntQueue
import com.ixume.udar.testing.debugConnect
import it.unimi.dsi.fastutil.doubles.DoubleArrayList
import it.unimi.dsi.fastutil.ints.IntComparator
import it.unimi.dsi.fastutil.ints.IntHeapPriorityQueue
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/**
 * Represent a dynamic AABB tree as a flattened AABB DoubleArray; a node k has its data at array[k * DATA_SIZE],
 *
 * data {
 *     parentIdx: Int       0
 *     isLeaf: Boolean      0
 *     child1Idx: Int       1
 *     child2Idx: Int       1
 *     minX: Double         2
 *     minY: Double         3
 *     maxX: Double         4
 *     maxY: Double         5
 *     exploredCost: Double 6
 * }
 *
 * Array is initialized as a flattened linked list of unused nodes. When a node is added, the first available free
 * node is found (with freeHead variable) and modified to hold the data of the added node. When a node is removed,
 * its updated to point to the current freeHead, and freeHead is updated.
 *
 * Implementation is filled with extension methods. These just apply to the indices, so you would use them on the index
 * of the node that you're referencing.
 */
class FlattenedAABBTree2D(
    capacity: Int,
    val level: Double,
    val axis: LocalMesher.AxisD,
) : IntComparator {
    private val blocked = AtomicBoolean(false)

    private var freeIdx = -1 // head of free linked list
    private var rootIdx = -1
    @Volatile
    var arr = DoubleArray(capacity * DATA_SIZE)

    init {
        setFree(0, capacity)
    }

    override fun compare(k1: Int, k2: Int): Int {
        return sign(k1.exploredCost() - k2.exploredCost()).toInt()
    }

    private val q = IntQueue()

    fun contains(x: Double, y: Double): Boolean {
        while (q.hasNext()) {
            val i = q.dequeue()

            if (!i.contains(x, y)) continue

            if (i.isLeaf()) {
                return true
            }

            q.enqueue(i.child1())
            q.enqueue(i.child2())
        }

        return false
    }

    private val _overlap = DoubleArray(4)

    fun constructCollisions(): FlattenedAABBTree2D {
        if (rootIdx == -1) return FlattenedAABBTree2D(0, level, axis)
        if (rootIdx.isLeaf()) return FlattenedAABBTree2D(0, level, axis)

        val out = FlattenedAABBTree2D(0, level, axis)

        val aq = IntQueue()
        val bq = IntQueue()

        aq.enqueue(rootIdx.child1())
        bq.enqueue(rootIdx.child2())

        while (aq.hasNext()) {
            val a = aq.dequeue()
            val b = bq.dequeue()

            if (a.isLeaf()) {
                if (b.isLeaf()) {
                    if (a.overlapsNode(b)) {
                        a.calcOverlap(
                            minX = b.minX(),
                            minY = b.minY(),
                            maxX = b.maxX(),
                            maxY = b.maxY(),
                            out = _overlap,
                        )

                        out.insert(
                            minX = _overlap[0],
                            minY = _overlap[1],
                            maxX = _overlap[2],
                            maxY = _overlap[3],
                        )
                    }

                    continue
                } else {
                    if (a.parent() == b.parent()) {
                        aq.enqueue(b.child1())
                        bq.enqueue(b.child2())
                    }

                    if (a.overlapsNode(b)) {
                        aq.enqueue(a)
                        bq.enqueue(b.child1())

                        aq.enqueue(a)
                        bq.enqueue(b.child2())
                    }
                }
            } else {
                if (b.isLeaf()) {
                    if (a.parent() == b.parent()) {
                        aq.enqueue(a.child1())
                        bq.enqueue(a.child2())
                    }

                    if (a.overlapsNode(b)) {
                        aq.enqueue(b)
                        bq.enqueue(a.child1())

                        aq.enqueue(b)
                        bq.enqueue(a.child2())
                    }
                } else {
                    if (a.parent() == b.parent()) {
                        aq.enqueue(a.child1())
                        bq.enqueue(a.child2())

                        aq.enqueue(b.child1())
                        bq.enqueue(b.child2())
                    }

                    if (a.overlapsNode(b)) {
                        aq.enqueue(a.child1())
                        bq.enqueue(b.child1())

                        aq.enqueue(a.child1())
                        bq.enqueue(b.child2())

                        aq.enqueue(a.child2())
                        bq.enqueue(b.child1())

                        aq.enqueue(a.child2())
                        bq.enqueue(b.child2())
                    }
                }
            }
        }

        return out
    }

    private fun Int.contains(x: Double, y: Double): Boolean {
        return x >= minX() && x <= maxX() &&
               y >= minY() && y <= maxY()
    }

    private fun Int.overlaps(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Boolean {
        return minX() < maxX && maxX() > minX &&
               minY() < maxY && maxY() > minY
    }

    private fun Int.overlapsNode(i: Int): Boolean {
        return overlaps(i.minX(), i.minY(), i.maxX(), i.maxY())
    }

    private fun Int.calcOverlap(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,

        out: DoubleArray,
    ) {
        out[0] = max(minX(), minX)
        out[1] = max(minY(), minY)
        out[2] = min(maxX(), maxX)
        out[3] = min(maxY(), maxY)
    }

    fun insert(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ) {
        if (!blocked.compareAndSet(false, true)) throw IllegalStateException("Tried to insert while blocked!")

        val best = bestSibling(
            minX,
            minY,
            maxX,
            maxY,
        )
       
        if (best == -1) {
            rootIdx = newNode(
                parent = -1,
                isLeaf = true,
                c1 = -1,
                c2 = -1,
                minX = minX,
                minY = minY,
                maxX = maxX,
                maxY = maxY,
            )

            blocked.set(false)
            return
        }

        if (best.isLeaf()) {
            val bv = best.volume()
            val uc = unifiedCost(
                aMinX = best.minX(),
                aMinY = best.minY(),
                aMaxX = best.maxX(),
                aMaxY = best.maxY(),

                bMinX = minX,
                bMinY = minY,
                bMaxX = maxX,
                bMaxY = maxY,
            )

            if (abs(bv + ((maxX - minX) * (maxY - minY)) - uc) < 1e-14) {
                best.setUnion(
                    aMinX = best.minX(),
                    aMinY = best.minY(),
                    aMaxX = best.maxX(),
                    aMaxY = best.maxY(),

                    bMinX = minX,
                    bMinY = minY,
                    bMaxX = maxX,
                    bMaxY = maxY,
                )

                best.parent().refitRecursively(
                    minX,
                    minY,
                    maxX,
                    maxY,
                )

                blocked.set(false)
                return
            }
        }

        val leaf = newNode(
            parent = -1,
            isLeaf = true,
            c1 = -1,
            c2 = -1,
            minX = minX,
            minY = minY,
            maxX = maxX,
            maxY = maxY,
        )

        val oldParent = best.parent()

        val newParent = newNode(
            parent = oldParent,
            isLeaf = false,
            c1 = -1,
            c2 = -1,
            minX = min(minX, best.minX()),
            minY = min(minY, best.minY()),
            maxX = max(maxX, best.maxX()),
            maxY = max(maxY, best.maxY()),
        )

        if (oldParent == -1) {
            newParent.child1(best)
            newParent.child2(leaf)

            rootIdx = newParent
        } else {
            if (oldParent.child1() == best) {
                oldParent.child1(newParent)
            } else {
                oldParent.child2(newParent)
            }

            newParent.child1(best)
            newParent.child2(leaf)
        }

        best.parent(newParent)
        leaf.parent(newParent)

        newParent.refitRecursively(
            minX,
            minY,
            maxX,
            maxY,
        )

        blocked.set(false)
    }

    private fun Int.refitRecursively(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ) {
        var p = this

        while (p != -1) {
            if (!p.tryMerge()) {
                p.refit(
                    minX,
                    minY,
                    maxX,
                    maxY,
                )
            }

            if (!p.isLeaf()) p.rotate()

            p = p.parent()
        }
    }

    private fun Int.tryMerge(): Boolean {
        if (isLeaf()) {
            return false
        }
        val c1 = child1()
        if (!c1.isLeaf()) {
            return false
        }
        val c2 = child2()
        if (!c2.isLeaf()) {
            return false
        }

        val c =
            unifiedCost(
                aMinX = c1.minX(),
                aMinY = c1.minY(),
                aMaxX = c1.maxX(),
                aMaxY = c1.maxY(),

                bMinX = c2.minX(),
                bMinY = c2.minY(),
                bMaxX = c2.maxX(),
                bMaxY = c2.maxY(),
            )

        val c1v = c1.volume()
        val c2v = c2.volume()

        if (abs(c1v + c2v - c) < 1e-14) {
            setUnion(
                aMinX = c1.minX(),
                aMinY = c1.minY(),
                aMaxX = c1.maxX(),
                aMaxY = c1.maxY(),

                bMinX = c2.minX(),
                bMinY = c2.minY(),
                bMaxX = c2.maxX(),
                bMaxY = c2.maxY(),
            )

            leaf(true)

            child1(-1)
            child2(-1)

            c1.removeNode()
            c2.removeNode()

            return true
        }

        return false
    }

    private fun Int.removeNode() {
        nextFree(freeIdx)
        free()

        freeIdx = this
    }

    val siblingQueue = IntHeapPriorityQueue(this)

    /**
     * Finds the best sibling of a given AABB and returns its index; -1 if no node is found
     */
    private fun bestSibling(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Int {
        if (rootIdx == -1) return -1
        if (rootIdx.isLeaf()) return rootIdx

        var bestCost = unifiedCost(
            minX,
            minY,
            maxX,
            maxY,
            rootIdx.minX(),
            rootIdx.minY(),
            rootIdx.maxX(),
            rootIdx.maxY(),
        )

        var bestNode = rootIdx

        siblingQueue.enqueue(rootIdx)

        while (!siblingQueue.isEmpty) {
            val node = siblingQueue.dequeueInt()
            val c = node.insertionCost(
                minX,
                minY,
                maxX,
                maxY,
            )

            if (c < bestCost) {
                bestNode = node
                bestCost = c
            }

            if (!node.isLeaf()) {
                val e = node.minChildrenCost(
                    minX,
                    minY,
                    maxX,
                    maxY,
                )

                if (e < bestCost) {
                    val c1 = node.child1()
                    val c2 = node.child2()
                    c1.exploredCost(e)
                    c2.exploredCost(e)

                    siblingQueue.enqueue(c1)
                    siblingQueue.enqueue(c2)
                }
            }
        }

        return bestNode
    }

    private fun Int.minChildrenCost(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Double {
        return ((maxX - minX) * (maxY - minY)) + refittingCost(
            minX,
            minY,
            maxX,
            maxY,
        ) + exploredCost()
    }

    /**
     * Modifiers exploredCost to store inherited cost
     */
    private fun Int.insertionCost(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Double {
        val parent = parent()
        val inherited = if (this == -1 || parent == -1) 0.0 else parent.recursiveRefittingCost(
            minX,
            minY,
            maxX,
            maxY,
        )

        exploredCost(inherited)

        return unifiedCost(
            aMinX = minX(),
            aMinY = minY(),
            aMaxX = maxX(),
            aMaxY = maxY(),

            bMinX = minX,
            bMinY = minY,
            bMaxX = maxX,
            bMaxY = maxY,
        ) + inherited
    }

    private fun Int.recursiveRefittingCost(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Double {
        var c = 0.0
        var p = this
        while (p != -1) {
            c += p.refittingCost(
                minX,
                minY,
                maxX,
                maxY,
            )

            p = p.parent()
        }

        return c
    }

    private fun Int.refittingCost(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Double {
        return unifiedCost(
            aMinX = minX(),
            aMinY = minY(),
            aMaxX = maxX(),
            aMaxY = maxY(),

            bMinX = minX,
            bMinY = minY,
            bMaxX = maxX,
            bMaxY = maxY,
        ) - cost()
    }

    private fun Int.cost(): Double {
        return if (isLeaf()) volume() else {
            val c1 = child1()
            val c2 = child2()
            unifiedCost(
                aMinX = c1.minX(),
                aMinY = c1.minY(),
                aMaxX = c1.maxX(),
                aMaxY = c1.maxY(),

                bMinX = c2.minX(),
                bMinY = c2.minY(),
                bMaxX = c2.maxX(),
                bMaxY = c2.maxY(),
            )
        }
    }

    private fun Int.volume(): Double {
        return (maxX() - minX()) * (maxY() - minY())
    }

    private fun Int.child1(): Int {
        return arr[this * DATA_SIZE + CHILD_1_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Int.child1(c1: Int) {
        arr[this * DATA_SIZE + CHILD_1_IDX_OFFSET] = arr[this * DATA_SIZE + CHILD_1_IDX_OFFSET].withLower(c1)
    }

    private fun Int.child2(): Int {
        return (arr[this * DATA_SIZE + CHILD_2_IDX_OFFSET].toRawBits() ushr 32).toInt()
    }

    private fun Int.child2(c2: Int) {
        arr[this * DATA_SIZE + CHILD_2_IDX_OFFSET] = arr[this * DATA_SIZE + CHILD_2_IDX_OFFSET].withHigher(c2)
    }

    private fun Int.isFree(): Boolean {
        return (arr[this * DATA_SIZE + NODE_STATUS_OFFSET].toRawBits() ushr 32).toInt() == NODE_UNUSED_STATUS
    }

    private fun Int.free() {
        arr[this * DATA_SIZE + NODE_STATUS_OFFSET] =
            arr[this * DATA_SIZE + NODE_STATUS_OFFSET].withHigher(NODE_UNUSED_STATUS)
    }

    private fun Int.isLeaf(): Boolean {
        return (arr[this * DATA_SIZE + IS_LEAF_OFFSET].toRawBits() ushr 32).toInt() == 1
    }

    private fun Int.leaf(l: Boolean) {
        if (l) {
            arr[this * DATA_SIZE + IS_LEAF_OFFSET] = arr[this * DATA_SIZE + IS_LEAF_OFFSET].withHigher(1)
        } else {
            arr[this * DATA_SIZE + IS_LEAF_OFFSET] = arr[this * DATA_SIZE + IS_LEAF_OFFSET].withHigher(0)
        }
    }

    private fun Int.minX(): Double {
        return arr[this * DATA_SIZE + MIN_X_OFFSET]
    }

    private fun Int.minX(d: Double) {
        arr[this * DATA_SIZE + MIN_X_OFFSET] = d
    }

    private fun Int.minY(): Double {
        return arr[this * DATA_SIZE + MIN_Y_OFFSET]
    }

    private fun Int.minY(d: Double) {
        arr[this * DATA_SIZE + MIN_Y_OFFSET] = d
    }

    private fun Int.maxX(): Double {
        return arr[this * DATA_SIZE + MAX_X_OFFSET]
    }

    private fun Int.maxX(d: Double) {
        arr[this * DATA_SIZE + MAX_X_OFFSET] = d
    }

    private fun Int.maxY(): Double {
        return arr[this * DATA_SIZE + MAX_Y_OFFSET]
    }

    private fun Int.maxY(d: Double) {
        arr[this * DATA_SIZE + MAX_Y_OFFSET] = d
    }

    private fun Int.exploredCost(): Double {
        return arr[this * DATA_SIZE + EXPLORED_COST_OFFSET]
    }

    private fun Int.exploredCost(d: Double) {
        arr[this * DATA_SIZE + EXPLORED_COST_OFFSET] = d
    }

    private fun Int.parent(): Int {
        return arr[this * DATA_SIZE + PARENT_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Int.parent(parent: Int) {
        arr[this * DATA_SIZE + PARENT_IDX_OFFSET] = arr[this * DATA_SIZE + PARENT_IDX_OFFSET].withLower(parent)
    }

    /**
     * Grows array to fit the extra element and fills it with linked list of unused nodes
     * @param size Size in nodes
     * @return Head of newly created list, or current free index of nothing new was needed
     */
    private fun growTo(size: Int): Int {
        if (arr.size >= size * DATA_SIZE) return freeIdx

        val prevSize = arr.size
        val newSize = max(size * DATA_SIZE, (arr.size * 3 / 2) / DATA_SIZE * DATA_SIZE)

        arr = arr.copyOf(newSize)

        setFree(prevSize / DATA_SIZE, newSize / DATA_SIZE)
        return prevSize / DATA_SIZE
    }

    /**
     * @return The index of the new node
     */
    private fun newNode(
        parent: Int,
        isLeaf: Boolean,
        c1: Int,
        c2: Int,
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ): Int {
        val currFreeIdx = freeIdx
        if (currFreeIdx == -1) {
            val f = growTo((arr.size / DATA_SIZE) + 1)
            check(f != -1)
            freeIdx = f.nextFree()

            f.parent(parent)
            f.leaf(isLeaf)
            f.child1(c1)
            f.child2(c2)
            f.minX(minX)
            f.minY(minY)
            f.maxX(maxX)
            f.maxY(maxY)

            return f
        }

        freeIdx = freeIdx.nextFree()

        if (freeIdx == -1) {
            freeIdx = growTo((arr.size / DATA_SIZE) + 1)
        }

        currFreeIdx.parent(parent)
        currFreeIdx.leaf(isLeaf)
        currFreeIdx.child1(c1)
        currFreeIdx.child2(c2)
        currFreeIdx.minX(minX)
        currFreeIdx.minY(minY)
        currFreeIdx.maxX(maxX)
        currFreeIdx.maxY(maxY)

        return currFreeIdx
    }

    /**
     * Only meant to be used on free nodes
     * @return The index of the next free node; -1 if full
     */
    private fun Int.nextFree(): Int {
        return arr[this * DATA_SIZE + NEXT_FREE_IDX_OFFSET].toRawBits().toInt()
    }

    private fun Int.nextFree(next: Int) {
        arr[this * DATA_SIZE + NEXT_FREE_IDX_OFFSET] = Double.fromBits(next.toLong())
    }

    private fun Int.refit(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,
    ) {
        minX(min(minX(), minX))
        minY(min(minY(), minY))
        maxX(max(maxX(), maxX))
        maxY(max(maxY(), maxY))
    }

    private fun Int.rotate() {
        val pp = parent()
        if (pp == -1) return

        val isFirst: Boolean
        val other: Int

        if (this == pp.child1()) {
            isFirst = true
            other = pp.child2()
        } else {
            isFirst = false
            other = pp.child1()
        }

        val c1 = child1()
        val c2 = child2()

        val curr = cost()
        val c1Swap = unifiedCost(
            aMinX = other.minX(),
            aMinY = other.minY(),
            aMaxX = other.maxX(),
            aMaxY = other.maxY(),

            bMinX = c2.minX(),
            bMinY = c2.minY(),
            bMaxX = c2.maxX(),
            bMaxY = c2.maxY(),
        )
        val c2Swap = unifiedCost(
            aMinX = other.minX(),
            aMinY = other.minY(),
            aMaxX = other.maxX(),
            aMaxY = other.maxY(),

            bMinX = c1.minX(),
            bMinY = c1.minY(),
            bMaxX = c1.maxX(),
            bMaxY = c1.maxY(),
        )

        if (c1Swap < curr) {
            child1(other)
            other.parent(this)

            if (isFirst) {
                pp.child2(c1)
            } else {
                pp.child1(c1)
            }

            c1.parent(pp)

            val nc1 = child1()
            val nc2 = child2()

            setUnion(
                aMinX = nc1.minX(),
                aMinY = nc1.minY(),
                aMaxX = nc1.maxX(),
                aMaxY = nc1.maxY(),

                bMinX = nc2.minX(),
                bMinY = nc2.minY(),
                bMaxX = nc2.maxX(),
                bMaxY = nc2.maxY(),
            )
        } else if (c2Swap < curr) {
            child2(other)
            other.parent(this)

            if (isFirst) {
                pp.child2(c2)
            } else {
                pp.child1(c2)
            }

            c2.parent(pp)

            val nc1 = child1()
            val nc2 = child2()

            setUnion(
                aMinX = nc1.minX(),
                aMinY = nc1.minY(),
                aMaxX = nc1.maxX(),
                aMaxY = nc1.maxY(),

                bMinX = nc2.minX(),
                bMinY = nc2.minY(),
                bMaxX = nc2.maxX(),
                bMaxY = nc2.maxY(),
            )
        }
    }

    private fun Int.setUnion(
        aMinX: Double,
        aMinY: Double,
        aMaxX: Double,
        aMaxY: Double,

        bMinX: Double,
        bMinY: Double,
        bMaxX: Double,
        bMaxY: Double,
    ) {
        minX(min(bMinX, aMinX))
        minY(min(bMinY, aMinY))
        maxX(max(bMaxX, aMaxX))
        maxY(max(bMaxY, aMaxY))
    }

    fun clear() {
        freeIdx = setFree(0, arr.size / DATA_SIZE)
        rootIdx = -1
    }

    /**
     * Only modifiers internal array size, causing a potentially invalid state. Call `clear` to reset to valid state
     */
    fun trim(size: Int) {
        arr = arr.copyOf(size)
    }

    /**
     * Does not perform bounds checks
     * @param start Start in node index, inclusive
     * @param end End in node index, exclusive
     * @return Head of linked list
     */
    private fun setFree(start: Int, end: Int): Int {
        if (end <= start) return -1

        var i = start
        while (i < end - 1) {
            i.nextFree(i + 1)
            i.free()

            i++
        }

        if (end - 1 >= 0) {
            (end - 1).nextFree(-1)
            (end - 1).free()
        }

        return start
    }

    /**
     * @return a flattened list of 2d aabb structs
     */
    fun overlaps(
        minX: Double,
        minY: Double,
        maxX: Double,
        maxY: Double,

        out: DoubleArrayList,
        math: LocalMathUtil,
    ) {
        if (rootIdx == -1) return

        val q = math.envOverlapQueue

        q.enqueue(rootIdx)

        while (q.hasNext()) {
            val node = q.dequeue()

            if (!node.overlaps(minX, minY, maxX, maxY)) continue

            if (node.isLeaf()) {
                out.add(node.minX())
                out.add(node.minY())
                out.add(node.maxX())
                out.add(node.maxY())

                continue
            }

            q.enqueue(node.child1())
            q.enqueue(node.child2())
        }
    }
    
    private val visualizationQueue = IntQueue()

    fun visualize(world: World, isHole: Boolean) {
        if (rootIdx == -1) return
        
        visualizationQueue.enqueue(rootIdx)
        
        while (visualizationQueue.hasNext()) {
            val node = visualizationQueue.dequeue()
            
            if (node.isLeaf()) {
                val options = if (isHole) Particle.DustOptions(Color.BLUE, 0.25f) else Particle.DustOptions(Color.RED, 0.25f)

                world.debugConnect(
                    start = withLevel(axis, node.minX(), node.minY(), level),
                    end = withLevel(axis, node.maxX(), node.minY(), level),
                    options = options,
                )
                world.debugConnect(
                    start = withLevel(axis, node.maxX(), node.minY(), level),
                    end = withLevel(axis, node.maxX(), node.maxY(), level),
                    options = options,
                )
                world.debugConnect(
                    start = withLevel(axis, node.maxX(), node.maxY(), level),
                    end = withLevel(axis, node.minX(), node.maxY(), level),
                    options = options,
                )
                world.debugConnect(
                    start = withLevel(axis, node.minX(), node.maxY(), level),
                    end = withLevel(axis, node.minX(), node.minY(), level),
                    options = options,
                )
            } else {
                visualizationQueue.enqueue(node.child1())
                visualizationQueue.enqueue(node.child2())
            }
        }
    }
}

private val DUST_OPTIONS = Particle.DustOptions(Color.FUCHSIA, 0.25f)

private const val DATA_SIZE = 7
private const val PARENT_IDX_OFFSET = 0 // low 32 bits; -1 if no parent
private const val IS_LEAF_OFFSET = 0 // high 32 bits
private const val CHILD_1_IDX_OFFSET = 1 // low 32 bits
private const val CHILD_2_IDX_OFFSET = 1 // high 32 bits
private const val MIN_X_OFFSET = 2
private const val MIN_Y_OFFSET = 3
private const val MAX_X_OFFSET = 4
private const val MAX_Y_OFFSET = 5
private const val EXPLORED_COST_OFFSET = 6

private const val NEXT_FREE_IDX_OFFSET = 0 // low 32 bits
private const val NODE_STATUS_OFFSET =
    0 // high 32 bits; normally represents boolean, however is -1 if it's unused

private const val NODE_UNUSED_STATUS = -1

private fun unifiedCost(
    aMinX: Double,
    aMinY: Double,
    aMaxX: Double,
    aMaxY: Double,

    bMinX: Double,
    bMinY: Double,
    bMaxX: Double,
    bMaxY: Double,
): Double {
    val minX = min(aMinX, bMinX)
    val minY = min(aMinY, bMinY)
    val maxX = max(aMaxX, bMaxX)
    val maxY = max(aMaxY, bMaxY)

    return (maxX - minX) * (maxY - minY)
}

const val LOWER_MASK = 0xFFFFFFFFL
const val UPPER_MASK = LOWER_MASK.inv()

fun Double.withLower(i: Int): Double {
    return Double.fromBits((this.toRawBits() and UPPER_MASK) or i.toLong())
}

fun Double.withHigher(i: Int): Double {
    return Double.fromBits((this.toRawBits() and LOWER_MASK) or (i.toLong() shl 32))
}