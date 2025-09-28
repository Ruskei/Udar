package com.ixume.udar.physics

import it.unimi.dsi.fastutil.ints.Int2IntMap

class I2IUnionFind(
    val map: Int2IntMap,
    val arr: IntArray,
    val size: Int,
) {
    private val sizes = IntArray(size)
    var islands = size
        private set

    fun idxOf(elem: Int): UnionIdx {
        return UnionIdx(map[elem])
    }

    fun find(i: UnionIdx): UnionIdx {
        var idx = i
        var idx2 = idx

        while (idx.value != arr[idx.value]) {
            idx = UnionIdx(arr[idx.value])
        }

        var t: Int
        while (idx2 != idx) {
            t = arr[idx2.value]
            arr[idx2.value] = idx.value
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
            arr[aI.value] = bI.value
        } else {
            sizes[aI.value] += sizes[bI.value]
            arr[bI.value] = aI.value
        }

        islands--
    }
}

@JvmInline
value class UnionIdx(val value: Int)