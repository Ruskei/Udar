package com.ixume.udar.physics

import it.unimi.dsi.fastutil.ints.IntArrayList
import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap

class Long2IntUnionFind(
    size: Int,
) {
    val map = Long2IntOpenHashMap(size)
    val elems = IntArrayList(size)
    var size = 0

    private val sizes = IntArray(size)
    var islands = size
        private set

    fun idxOf(elem: Long): UnionIdx {
        return UnionIdx(map[elem])
    }

    fun find(i: UnionIdx): UnionIdx {
        var idx = i
        var idx2 = idx

        while (idx.value != elems.getInt(idx.value)) {
            idx = UnionIdx(elems.getInt(idx.value))
        }

        var t: Int
        while (idx2 != idx) {
            t = elems.getInt(idx2.value)
            elems.set(idx2.value, idx.value)
            idx2 = UnionIdx(t)
        }

        return idx
    }

    fun connected(a: UnionIdx, b: UnionIdx): Boolean {
        return find(a) == find(b)
    }

    fun islandSize(a: UnionIdx): Int {
        return sizes[find(a).value]
    }

    fun union(a: UnionIdx, b: UnionIdx) {
        val aI = find(a)
        val bI = find(b)

        if (aI == bI) return

        if (sizes[aI.value] < sizes[bI.value]) {
            sizes[bI.value] += sizes[aI.value]
            elems.set(aI.value, bI.value)
        } else {
            sizes[aI.value] += sizes[bI.value]
            elems.set(bI.value, aI.value)
        }

        islands--
    }

    fun insert(e: Long): UnionIdx {
        elems.add(idxOf(e).value)
        map.put(e, size)
        size++

        return UnionIdx(size - 1)
    }
}

@JvmInline
value class UnionIdx(val value: Int)