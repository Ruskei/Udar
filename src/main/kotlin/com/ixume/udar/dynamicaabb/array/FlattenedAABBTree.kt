package com.ixume.udar.dynamicaabb.array

import com.ixume.udar.testing.debugConnect
import it.unimi.dsi.fastutil.ints.IntComparator
import it.unimi.dsi.fastutil.ints.IntHeapPriorityQueue
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.joml.Vector3d
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
 *     minZ: Double         4
 *     maxX: Double         5
 *     maxY: Double         6
 *     maxZ: Double         7
 *     exploredCost: Double 8
 * }
 *
 * Array is initialized as a flattened linked list of unused nodes. When a node is added, the first available free
 * node is found (with freeHead variable) and modified to hold the data of the added node. When a node is removed,
 * its updated to point to the current freeHead, and freeHead is updated.
 *
 * Implementation is filled with extension methods. These just apply to the indices, so you would use them on the index
 * of the node that you're referencing.
 */
class FlattenedAABBTree(
    capacity: Int,
) : IntComparator {
    private val blocked = AtomicBoolean(false)

    private var freeIdx = -1 // head of free linked list
    private var rootIdx = -1
    var arr = DoubleArray(capacity * DATA_SIZE)

    init {
        setFree(0, capacity)
    }

    override fun compare(k1: Int, k2: Int): Int {
        return sign(k1.exploredCost() - k2.exploredCost()).toInt()
    }

    fun iterator(): FlattenedAABBTreeIterator {
        return FlattenedAABBTreeIterator(this)
    }

    val containmentQueue = IntQueue()

    fun contains(x: Double, y: Double, z: Double): Boolean {
        if (rootIdx == -1) return false

        containmentQueue.enqueue(rootIdx)

        while (containmentQueue.hasNext()) {
            val i = containmentQueue.dequeue()

            if (!i.contains(x, y, z)) continue

            if (i.isLeaf()) {
                return true
            }

            containmentQueue.enqueue(i.child1())
            containmentQueue.enqueue(i.child2())
        }

        return false
    }

    private fun Int.contains(x: Double, y: Double, z: Double): Boolean {
        return x >= minX() && x <= maxX() &&
               y >= minY() && y <= maxY() &&
               z >= minZ() && z <= maxZ()
    }

    fun insert(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ) {
        if (!blocked.compareAndSet(false, true)) throw IllegalStateException("Tried to insert while blocked!")

        val best = bestSibling(
            minX,
            minY,
            minZ,
            maxX,
            maxY,
            maxZ,
        )

        if (best == -1) {
            rootIdx = newNode(
                parent = -1,
                isLeaf = true,
                c1 = -1,
                c2 = -1,
                minX = minX,
                minY = minY,
                minZ = minZ,
                maxX = maxX,
                maxY = maxY,
                maxZ = maxZ,
            )

            blocked.set(false)
            return
        }

        // check if the best sibling and proposed aabb could be represented as 1 aabb
        if (best.isLeaf()) {
            val bv = best.volume()
            val uc = unifiedCost(
                aMinX = best.minX(),
                aMinY = best.minY(),
                aMinZ = best.minZ(),
                aMaxX = best.maxX(),
                aMaxY = best.maxY(),
                aMaxZ = best.maxZ(),

                bMinX = minX,
                bMinY = minY,
                bMinZ = minZ,
                bMaxX = maxX,
                bMaxY = maxY,
                bMaxZ = maxZ,
            )

            if (abs(bv + ((maxX - minX) * (maxY - minY) * (maxZ - minZ)) - uc) < 1e-14) {
                best.setUnion(
                    aMinX = best.minX(),
                    aMinY = best.minY(),
                    aMinZ = best.minZ(),
                    aMaxX = best.maxX(),
                    aMaxY = best.maxY(),
                    aMaxZ = best.maxZ(),

                    bMinX = minX,
                    bMinY = minY,
                    bMinZ = minZ,
                    bMaxX = maxX,
                    bMaxY = maxY,
                    bMaxZ = maxZ,
                )

                best.parent().refitRecursively(
                    minX,
                    minY,
                    minZ,
                    maxX,
                    maxY,
                    maxZ,
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
            minZ = minZ,
            maxX = maxX,
            maxY = maxY,
            maxZ = maxZ,
        )

        val oldParent = best.parent()

        val newParent = newNode(
            parent = oldParent,
            isLeaf = false,
            c1 = -1,
            c2 = -1,
            minX = min(minX, best.minX()),
            minY = min(minY, best.minY()),
            minZ = min(minZ, best.minZ()),
            maxX = max(maxX, best.maxX()),
            maxY = max(maxY, best.maxY()),
            maxZ = max(maxZ, best.maxZ()),
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
            minZ,
            maxX,
            maxY,
            maxZ,
        )

        blocked.set(false)
    }

    private fun Int.refitRecursively(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ) {
        var p = this

        while (p != -1) {
            if (!p.tryMerge()) {
                p.refit(
                    minX,
                    minY,
                    minZ,
                    maxX,
                    maxY,
                    maxZ,
                )
            }

            if (!p.isLeaf()) p.rotate()

            p = p.parent()
        }
    }

    /**
     * Finds the best sibling of a given AABB and returns its index; -1 if no node is found
     */
    private fun bestSibling(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ): Int {
        if (rootIdx == -1) return -1
        if (rootIdx.isLeaf()) return rootIdx

        var bestCost = unifiedCost(
            minX,
            minY,
            minZ,
            maxX,
            maxY,
            maxZ,
            rootIdx.minX(),
            rootIdx.minY(),
            rootIdx.minZ(),
            rootIdx.maxX(),
            rootIdx.maxY(),
            rootIdx.maxZ(),
        )

        var bestNode = rootIdx

        val q = IntHeapPriorityQueue(this)

        q.enqueue(rootIdx)

        while (!q.isEmpty) {
            val node = q.dequeueInt()
            val c = node.insertionCost(
                minX,
                minY,
                minZ,
                maxX,
                maxY,
                maxZ,
            )

            if (c < bestCost) {
                bestNode = node
                bestCost = c
            }

            if (!node.isLeaf()) {
                val e = node.minChildrenCost(
                    minX,
                    minY,
                    minZ,
                    maxX,
                    maxY,
                    maxZ,
                )

                if (e < bestCost) {
                    val c1 = node.child1()
                    val c2 = node.child2()
                    c1.exploredCost(e)
                    c2.exploredCost(e)

                    q.enqueue(c1)
                    q.enqueue(c2)
                }
            }
        }

        return bestNode
    }

    private fun Int.minChildrenCost(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ): Double {
        return ((maxX - minX) * (maxY - minY) * (maxZ - minZ)) + refittingCost(
            minX,
            minY,
            minZ,
            maxX,
            maxY,
            maxZ,
        ) + exploredCost()
    }

    /**
     * Modifiers exploredCost to store inherited cost
     */
    private fun Int.insertionCost(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ): Double {
        val parent = parent()
        val inherited = if (parent == -1) 0.0 else parent.recursiveRefittingCost(
            minX,
            minY,
            minZ,
            maxX,
            maxY,
            maxZ,
        )

        exploredCost(inherited)

        return unifiedCost(
            aMinX = minX(),
            aMinY = minY(),
            aMinZ = minZ(),
            aMaxX = maxX(),
            aMaxY = maxY(),
            aMaxZ = maxZ(),

            bMinX = minX,
            bMinY = minY,
            bMinZ = minZ,
            bMaxX = maxX,
            bMaxY = maxY,
            bMaxZ = maxZ,
        ) + inherited
    }

    private fun Int.recursiveRefittingCost(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ): Double {
        var c = 0.0
        var p = this
        while (p != -1) {
            c += p.refittingCost(
                minX,
                minY,
                minZ,
                maxX,
                maxY,
                maxZ,
            )

            p = p.parent()
        }

        return c
    }

    private fun Int.refittingCost(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ): Double {
        return unifiedCost(
            aMinX = minX(),
            aMinY = minY(),
            aMinZ = minZ(),
            aMaxX = maxX(),
            aMaxY = maxY(),
            aMaxZ = maxZ(),

            bMinX = minX,
            bMinY = minY,
            bMinZ = minZ,
            bMaxX = maxX,
            bMaxY = maxY,
            bMaxZ = maxZ,
        ) - cost()
    }

    private fun Int.cost(): Double {
        return if (isLeaf()) volume() else {
            val c1 = child1()
            val c2 = child2()
            unifiedCost(
                aMinX = c1.minX(),
                aMinY = c1.minY(),
                aMinZ = c1.minZ(),
                aMaxX = c1.maxX(),
                aMaxY = c1.maxY(),
                aMaxZ = c1.maxZ(),

                bMinX = c2.minX(),
                bMinY = c2.minY(),
                bMinZ = c2.minZ(),
                bMaxX = c2.maxX(),
                bMaxY = c2.maxY(),
                bMaxZ = c2.maxZ(),
            )
        }
    }

    private fun Int.volume(): Double {
        return (maxX() - minX()) * (maxY() - minY()) * (maxZ() - minZ())
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

    fun isNodeFree(i: Int): Boolean {
        return (arr[i * DATA_SIZE + NODE_STATUS_OFFSET].toRawBits() ushr 32).toInt() == NODE_UNUSED_STATUS
    }

    fun isNodeLeaf(i: Int): Boolean {
        return (arr[i * DATA_SIZE + IS_LEAF_OFFSET].toRawBits() ushr 32).toInt() == 1
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

    private fun Int.minZ(): Double {
        return arr[this * DATA_SIZE + MIN_Z_OFFSET]
    }

    private fun Int.minZ(d: Double) {
        arr[this * DATA_SIZE + MIN_Z_OFFSET] = d
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

    private fun Int.maxZ(): Double {
        return arr[this * DATA_SIZE + MAX_Z_OFFSET]
    }

    private fun Int.maxZ(d: Double) {
        arr[this * DATA_SIZE + MAX_Z_OFFSET] = d
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
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
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
            f.minZ(minZ)
            f.maxX(maxX)
            f.maxY(maxY)
            f.maxZ(maxZ)

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
        currFreeIdx.minZ(minZ)
        currFreeIdx.maxX(maxX)
        currFreeIdx.maxY(maxY)
        currFreeIdx.maxZ(maxZ)

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
                aMinZ = c1.minZ(),
                aMaxX = c1.maxX(),
                aMaxY = c1.maxY(),
                aMaxZ = c1.maxZ(),

                bMinX = c2.minX(),
                bMinY = c2.minY(),
                bMinZ = c2.minZ(),
                bMaxX = c2.maxX(),
                bMaxY = c2.maxY(),
                bMaxZ = c2.maxZ(),
            )

        val c1v = c1.volume()
        val c2v = c2.volume()

        if (abs(c1v + c2v - c) < 1e-14) {
            setUnion(
                aMinX = c1.minX(),
                aMinY = c1.minY(),
                aMinZ = c1.minZ(),
                aMaxX = c1.maxX(),
                aMaxY = c1.maxY(),
                aMaxZ = c1.maxZ(),

                bMinX = c2.minX(),
                bMinY = c2.minY(),
                bMinZ = c2.minZ(),
                bMaxX = c2.maxX(),
                bMaxY = c2.maxY(),
                bMaxZ = c2.maxZ(),
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

    private fun Int.refit(
        minX: Double,
        minY: Double,
        minZ: Double,
        maxX: Double,
        maxY: Double,
        maxZ: Double,
    ) {
        minX(min(minX(), minX))
        minY(min(minY(), minY))
        minZ(min(minZ(), minZ))
        maxX(max(maxX(), maxX))
        maxY(max(maxY(), maxY))
        maxZ(max(maxZ(), maxZ))
    }

    private fun Int.rotate() {
        val pp = parent()
        if (pp == -1) return

        if (isLeaf()) {
            return
        }

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
            aMinZ = other.minZ(),
            aMaxX = other.maxX(),
            aMaxY = other.maxY(),
            aMaxZ = other.maxZ(),

            bMinX = c2.minX(),
            bMinY = c2.minY(),
            bMinZ = c2.minZ(),
            bMaxX = c2.maxX(),
            bMaxY = c2.maxY(),
            bMaxZ = c2.maxZ(),
        )
        val c2Swap = unifiedCost(
            aMinX = other.minX(),
            aMinY = other.minY(),
            aMinZ = other.minZ(),
            aMaxX = other.maxX(),
            aMaxY = other.maxY(),
            aMaxZ = other.maxZ(),

            bMinX = c1.minX(),
            bMinY = c1.minY(),
            bMinZ = c1.minZ(),
            bMaxX = c1.maxX(),
            bMaxY = c1.maxY(),
            bMaxZ = c1.maxZ(),
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
                aMinZ = nc1.minZ(),
                aMaxX = nc1.maxX(),
                aMaxY = nc1.maxY(),
                aMaxZ = nc1.maxZ(),

                bMinX = nc2.minX(),
                bMinY = nc2.minY(),
                bMinZ = nc2.minZ(),
                bMaxX = nc2.maxX(),
                bMaxY = nc2.maxY(),
                bMaxZ = nc2.maxZ(),
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
                aMinZ = nc1.minZ(),
                aMaxX = nc1.maxX(),
                aMaxY = nc1.maxY(),
                aMaxZ = nc1.maxZ(),

                bMinX = nc2.minX(),
                bMinY = nc2.minY(),
                bMinZ = nc2.minZ(),
                bMaxX = nc2.maxX(),
                bMaxY = nc2.maxY(),
                bMaxZ = nc2.maxZ(),
            )
        }
    }

    private fun Int.setUnion(
        aMinX: Double,
        aMinY: Double,
        aMinZ: Double,
        aMaxX: Double,
        aMaxY: Double,
        aMaxZ: Double,

        bMinX: Double,
        bMinY: Double,
        bMinZ: Double,
        bMaxX: Double,
        bMaxY: Double,
        bMaxZ: Double,
    ) {
        minX(min(bMinX, aMinX))
        minY(min(bMinY, aMinY))
        minZ(min(bMinZ, aMinZ))
        maxX(max(bMaxX, aMaxX))
        maxY(max(bMaxY, aMaxY))
        maxZ(max(bMaxZ, aMaxZ))
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

    private fun Int.removeNode() {
        nextFree(freeIdx)
        free()

        freeIdx = this
    }

    fun visualize(world: World) {
        var i = 0
        val numNodes = arr.size / DATA_SIZE
        while (i < numNodes) {
            if (i.isFree()) {
                i++
                continue
            }

            val dustOptions = if (i.isLeaf()) DUST_OPTIONS_LEAF else DUST_OPTIONS

            world.debugConnect(
                start = Vector3d(i.minX(), i.minY(), i.minZ()),
                end = Vector3d(i.maxX(), i.minY(), i.minZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.minX(), i.maxY(), i.minZ()),
                end = Vector3d(i.maxX(), i.maxY(), i.minZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.minX(), i.maxY(), i.maxZ()),
                end = Vector3d(i.maxX(), i.maxY(), i.maxZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.minX(), i.minY(), i.maxZ()),
                end = Vector3d(i.maxX(), i.minY(), i.maxZ()),
                options = dustOptions,
            )

            world.debugConnect(
                start = Vector3d(i.minX(), i.minY(), i.minZ()),
                end = Vector3d(i.minX(), i.maxY(), i.minZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.maxX(), i.minY(), i.minZ()),
                end = Vector3d(i.maxX(), i.maxY(), i.minZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.maxX(), i.minY(), i.maxZ()),
                end = Vector3d(i.maxX(), i.maxY(), i.maxZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.minX(), i.minY(), i.maxZ()),
                end = Vector3d(i.minX(), i.maxY(), i.maxZ()),
                options = dustOptions,
            )

            world.debugConnect(
                start = Vector3d(i.minX(), i.minY(), i.minZ()),
                end = Vector3d(i.minX(), i.minY(), i.maxZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.minX(), i.maxY(), i.minZ()),
                end = Vector3d(i.minX(), i.maxY(), i.maxZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.maxX(), i.maxY(), i.minZ()),
                end = Vector3d(i.maxX(), i.maxY(), i.maxZ()),
                options = dustOptions,
            )
            world.debugConnect(
                start = Vector3d(i.maxX(), i.minY(), i.minZ()),
                end = Vector3d(i.maxX(), i.minY(), i.maxZ()),
                options = dustOptions,
            )

            i++
        }
    }
}

private val DUST_OPTIONS = Particle.DustOptions(Color.RED, 0.25f)
private val DUST_OPTIONS_LEAF = Particle.DustOptions(Color.FUCHSIA, 0.3f)

internal const val DATA_SIZE = 9
private const val PARENT_IDX_OFFSET = 0 // low 32 bits; -1 if no parent
private const val IS_LEAF_OFFSET = 0 // high 32 bits
private const val CHILD_1_IDX_OFFSET = 1 // low 32 bits
private const val CHILD_2_IDX_OFFSET = 1 // high 32 bits
private const val MIN_X_OFFSET = 2
private const val MIN_Y_OFFSET = 3
private const val MIN_Z_OFFSET = 4
private const val MAX_X_OFFSET = 5
private const val MAX_Y_OFFSET = 6
private const val MAX_Z_OFFSET = 7
private const val EXPLORED_COST_OFFSET = 8

private const val NEXT_FREE_IDX_OFFSET = 0 // low 32 bits
private const val NODE_STATUS_OFFSET =
    0 // high 32 bits; normally represents boolean, however is -1 if it's unused

private const val NODE_UNUSED_STATUS = -1

private fun unifiedCost(
    aMinX: Double,
    aMinY: Double,
    aMinZ: Double,
    aMaxX: Double,
    aMaxY: Double,
    aMaxZ: Double,

    bMinX: Double,
    bMinY: Double,
    bMinZ: Double,
    bMaxX: Double,
    bMaxY: Double,
    bMaxZ: Double,
): Double {
    val minX = min(aMinX, bMinX)
    val minY = min(aMinY, bMinY)
    val minZ = min(aMinZ, bMinZ)
    val maxX = max(aMaxX, bMaxX)
    val maxY = max(aMaxY, bMaxY)
    val maxZ = max(aMaxZ, bMaxZ)

    return (maxX - minX) * (maxY - minY) * (maxZ - minZ)
}

private const val LOWER_MASK = 0xFFFFFFFFL
private const val UPPER_MASK = LOWER_MASK.inv()

fun Double.withLower(i: Int): Double {
    return Double.fromBits((this.toRawBits() and UPPER_MASK) or i.toLong())
}

fun Double.withHigher(i: Int): Double {
    return Double.fromBits((this.toRawBits() and LOWER_MASK) or (i.toLong() shl 32))
}