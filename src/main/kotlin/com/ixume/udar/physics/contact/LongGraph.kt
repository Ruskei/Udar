package com.ixume.udar.physics.contact

import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
import it.unimi.dsi.fastutil.longs.LongOpenHashSet
import it.unimi.dsi.fastutil.longs.LongSet

typealias LongGraph = Long2ObjectOpenHashMap<LongSet>

fun LongGraph.undirected(first: Long, second: Long) {
    var firstExisting = get(first)
    if (firstExisting == null) {
        firstExisting = LongOpenHashSet()
        put(first, firstExisting)
    }

    var secondExisting = get(second)
    if (secondExisting == null) {
        secondExisting = LongOpenHashSet()
        put(second, secondExisting)
    }
    
    firstExisting.add(second)
    secondExisting.add(first)
}