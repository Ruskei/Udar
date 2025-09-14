package com.ixume.udar

import com.ixume.udar.body.active.ActiveBody
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap
import java.util.*
import java.util.concurrent.locks.ReentrantReadWriteLock
import kotlin.concurrent.read
import kotlin.concurrent.write

class ActiveBodiesCollection {
    private val bodiesList = mutableListOf<ActiveBody>()
    private val lookupMap = mutableMapOf<UUID, ActiveBody>()
    private val idLookup = Long2ObjectOpenHashMap<ActiveBody>()

    private val lock = ReentrantReadWriteLock()

    fun bodies(): List<ActiveBody> = lock.read { bodiesList.toList() }

    fun size(): Int = lock.read { bodiesList.size }

    fun add(body: ActiveBody): Unit = lock.write {
        if (lookupMap.containsKey(body.uuid)) return

        body.idx = bodiesList.size

        bodiesList.add(body)
        lookupMap[body.uuid] = body
        idLookup.put(body.id, body)
    }

    fun remove(uuid: UUID): ActiveBody? = lock.write {
        val bodyToRemove = lookupMap[uuid] ?: return null

        val indexToRemove = bodyToRemove.idx
        val lastBody = bodiesList.last()

        if (bodyToRemove.uuid != lastBody.uuid) {
            bodiesList[indexToRemove] = lastBody
            lastBody.idx = indexToRemove
        }

        bodiesList.removeAt(bodiesList.size - 1)
        lookupMap.remove(uuid)
        idLookup.remove(bodyToRemove.id)

        bodyToRemove.idx = -1

        return bodyToRemove
    }

    operator fun get(idx: Int): ActiveBody? = lock.read {
        return bodiesList.getOrNull(idx)
    }
    
    fun fastGet(idx: Int): ActiveBody? {
        return bodiesList.getOrNull(idx)
    }
    
    operator fun get(uuid: UUID): ActiveBody? = lock.read {
        return lookupMap[uuid]
    }

    operator fun get(uuid: Long): ActiveBody? = lock.read {
        return idLookup[uuid]
    }

    fun fastGet(uuid: Long): ActiveBody? {
        return idLookup[uuid]
    }

    operator fun contains(body: ActiveBody): Boolean = lock.read {
        return this[body.uuid] != null
    }

    operator fun contains(uuid: UUID): Boolean = lock.read {
        return this[uuid] != null
    }
}