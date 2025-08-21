package com.ixume.udar.physics

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody

class BodyIDMap(
    val world: PhysicsWorld,
) {
    var bodyIDMap: Array<ActiveBody>

    var bodyCount: Int

    init {
        val snapshot = world.bodiesSnapshot()
        bodyCount = snapshot.size
        bodyIDMap = world.bodiesSnapshot().let {
            Array(it.size) { i ->
                val b = it[i]
                b.id = i
                b
            }
        }
    }

    fun update() {
        val ls = world.bodiesSnapshot()
        bodyIDMap = Array(ls.size) {
            val b = ls[it]
            b.id = it
            b
        }
    }
}