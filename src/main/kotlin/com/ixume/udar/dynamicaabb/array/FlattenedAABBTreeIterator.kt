package com.ixume.udar.dynamicaabb.array

class FlattenedAABBTreeIterator(
    val tree: FlattenedAABBTree,
) {
    private var cursor = 0
    
    fun hasNext(): Boolean {
        val s = tree.arr.size
        if (cursor >= s) return false
        
        var n = cursor / DATA_SIZE
        // skip to a leaf node
        while (n < s / DATA_SIZE) {
            if (!tree.isNodeFree(n) && tree.isNodeLeaf(n)) {
                cursor = n * DATA_SIZE
                return true
            }
            
            n++
        }
        
        return false
    }

    fun next(_bb: DoubleArray) {
        _bb[0] = tree.arr[cursor + 2]
        _bb[1] = tree.arr[cursor + 2 + 1]
        _bb[2] = tree.arr[cursor + 2 + 2]
        _bb[3] = tree.arr[cursor + 2 + 3]
        _bb[4] = tree.arr[cursor + 2 + 4]
        _bb[5] = tree.arr[cursor + 2 + 5]
        
        cursor += DATA_SIZE
    }
}