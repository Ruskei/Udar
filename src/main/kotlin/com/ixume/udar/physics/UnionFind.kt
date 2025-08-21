package com.ixume.udar.physics

import com.ixume.udar.body.active.ActiveBody

class UnionFind(
    elems: Collection<ActiveBody>,
) {
    private val map = HashMap<ActiveBody, Int>(elems.size)
    private val arr = IntArray(elems.size)
    private val sizes = IntArray(elems.size)
    var islands = elems.size
        private set

    init {
        var i = 0
        for (e in elems) {
            map[e] = i
            arr[i] = i
            sizes[i] = 1

            ++i
        }
    }

    fun idxOf(body: ActiveBody): Int? {
        return map[body]
    }

    fun find(i: Int): Int {
        var idx = i
        var idx2 = idx

        while (idx != arr[idx]) {
            idx = arr[idx]
        }

        var t = 0
        while (idx2 != idx) {
            t = arr[idx2]
            arr[idx2] = idx
            idx2 = t
        }

        return idx
    }

    fun connected(a: Int, b: Int): Boolean {
        return find(a) == find(b)
    }

    fun islandSize(a: Int): Int {
        return sizes[find(a)]
    }

    fun union(a: Int, b: Int) {
        val aI = find(a)
        val bI = find(b)

        if (aI == bI) return

        if (sizes[aI] < sizes[bI]) {
            sizes[bI] += sizes[aI]
            arr[aI] = bI
        } else {
            sizes[aI] += sizes[bI]
            arr[bI] = aI
        }

        islands--
    }
}