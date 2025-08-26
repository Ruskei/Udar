package com.ixume.udar.physics

import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.AABB
import org.bukkit.World
import java.util.concurrent.ConcurrentLinkedQueue

class RealWorldGetter2(
    private val world: World
) {
    private val queue = ConcurrentLinkedQueue<MeshRequest>()

    fun fetchBBs(request: MeshRequest) {
        queue += request
    }

    fun tick() {
        var rq: MeshRequest?
        while (queue.poll().also { rq = it } != null) {
            rq ?: return
            rq.callback(rq.mesher.mesh(world, rq.bb))
        }
    }
}

class MeshRequest(
    val bb: AABB,
    val mesher: LocalMesher,
    val callback: (LocalMesher.Mesh2?) -> (Unit),
)