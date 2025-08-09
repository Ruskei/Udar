package com.ixume.udar.physics

import com.ixume.udar.collisiondetection.broadphase.aabb.AABB
import com.ixume.udar.collisiondetection.mesh.Mesh
import org.bukkit.World
import java.util.concurrent.ConcurrentLinkedQueue

class RealWorldGetter(
    private val world: World
) {
    private val queue = ConcurrentLinkedQueue<BBRequest>()

    fun fetchBBs(request: BBRequest) {
        queue += request
    }

    fun tick() {
        var rq: BBRequest?
        while (queue.poll().also { rq = it } != null) {
            rq ?: return
            rq.callback(Mesh.Companion.mesh(world, rq.bb))
        }
    }
}

data class BBRequest(
    val bb: AABB,
    val callback: (Mesh) -> (Unit)
)