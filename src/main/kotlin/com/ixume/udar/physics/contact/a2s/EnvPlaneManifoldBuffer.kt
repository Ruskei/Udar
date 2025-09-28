package com.ixume.udar.physics.contact.a2s

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.I2IUnionFind
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldArray
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldCollection
import it.unimi.dsi.fastutil.ints.Int2IntOpenHashMap

class EnvPlaneManifoldBuffer(numContacts: Int) {
    val buffer = A2SManifoldArray(numContacts)
    val bufferMap = Int2IntOpenHashMap() // maps from union idx -> idx in buffer

    init {
        bufferMap.defaultReturnValue(-1)
    }

    lateinit var graph: I2IUnionFind

    fun addManifold(
        activeBody: ActiveBody,
        contactID: Long,
        buf: A2SContactDataBuffer,
        faceIdx: Int,
    ) {
        val depth = buf.maxDepth()
        val unionIdx = graph.find(graph.idxOf(faceIdx)).value
        val existingIdx = bufferMap.get(unionIdx)
        if (existingIdx != -1) {
            val existingDepth = buffer.maxDepth(existingIdx)
            if (depth < existingDepth) {
                // this is hilariously hacky but i don't want to make a whole "set" method
//                println("USURPING")
                val currCursor = buffer.cursor
                buffer.cursor = existingIdx

                buffer.addManifold(activeBody, contactID, buf)

                buffer.cursor = currCursor

            } else {
//                println("USURPED")
            }

            return
        }

//        println("BORN")
        bufferMap.put(unionIdx, buffer.cursor)

        buffer.addManifold(activeBody, contactID, buf)
    }

    fun clear() {
        buffer.clear()
        bufferMap.clear()
    }

    fun post(out: A2SManifoldCollection) {
        var i = 0
        while (i < buffer.size()) {
            out.load(buffer, i)
            i++
        }
    }
}