package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.mesh.mesh2.MeshFaceSortedList
import com.ixume.udar.collisiondetection.mesh.quadtree.EdgeQuadtree
import com.ixume.udar.collisiondetection.mesh.quadtree.FlattenedEdgeQuadtree
import com.ixume.udar.dynamicaabb.AABB
import org.bukkit.Bukkit
import org.bukkit.scheduler.BukkitTask
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.math.floor
import kotlin.system.measureNanoTime

/*
this will return a list of face lists and a list of edge trees to the requester
 */

class WorldMeshesManager(
    val physicsWorld: PhysicsWorld,
) {
    val world = physicsWorld.world

    private val mesher = LocalMesher()
    private val positionedMeshes = ConcurrentHashMap<MeshPosition, LocalMesher.Mesh2>()

    private val queue = ConcurrentLinkedQueue<MeshRequest>()
    private var syncTask: BukkitTask? = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, ::tick, 1, 1)

    fun request(prevBB: AABB?, currentBB: AABB, envContactGenerator: EnvironmentContactGenerator2) {
        /*
        go through every possible mesh position that the requested bb could occupy and add a request for that

        1d situation:

        grid:
          *-----*-----*-----*-----*
        requested:
             *----------*

        we only need to modify mesh of contact generator if its current meshes are not equal to new meshes
        if no generation is needed, do nothing; however if even 1 mesh needs to be generated, we put in a request for the whole thing
        
        if toGen is not empty
            add request
        else
            if bounds of current bb == old bb
                do nothing
            else
                modify contact gen immediately
        end
            
         */

        val toGen = mutableListOf<MeshPosition>()

        val faces = mutableListOf<MeshFaceSortedList>()
        val edges = mutableListOf<FlattenedEdgeQuadtree>()

        if (prevBB != null) {
            val prevMinX = floor((prevBB.minX - BB_SAFETY) / MESH_SIZE).toInt()
            val prevMinY = floor((prevBB.minY - BB_SAFETY) / MESH_SIZE).toInt()
            val prevMinZ = floor((prevBB.minZ - BB_SAFETY) / MESH_SIZE).toInt()

            val prevMaxX = floor((prevBB.maxX + BB_SAFETY) / MESH_SIZE).toInt()
            val prevMaxY = floor((prevBB.maxY + BB_SAFETY) / MESH_SIZE).toInt()
            val prevMaxZ = floor((prevBB.maxZ + BB_SAFETY) / MESH_SIZE).toInt()

            val currentMinX = floor((currentBB.minX - BB_SAFETY) / MESH_SIZE).toInt()
            val currentMinY = floor((currentBB.minY - BB_SAFETY) / MESH_SIZE).toInt()
            val currentMinZ = floor((currentBB.minZ - BB_SAFETY) / MESH_SIZE).toInt()

            val currentMaxX = floor((currentBB.maxX + BB_SAFETY) / MESH_SIZE).toInt()
            val currentMaxY = floor((currentBB.maxY + BB_SAFETY) / MESH_SIZE).toInt()
            val currentMaxZ = floor((currentBB.maxZ + BB_SAFETY) / MESH_SIZE).toInt()

            // check if current is either equal to or contained
            val isStillValid = prevMinX >= currentMinX && prevMinY >= currentMinY && prevMinZ >= currentMinZ &&
                               prevMaxX <= currentMaxX && prevMaxY <= currentMaxY && prevMaxZ <= currentMaxZ

            if (isStillValid) return

            for (x in currentMinX..currentMaxX) {
                for (y in currentMinY..currentMaxY) {
                    for (z in currentMinZ..currentMaxZ) {
                        val meshPos = MeshPosition(x, y, z)

                        val mesh = positionedMeshes[meshPos]

                        if (mesh == null) {
                            toGen += meshPos
                        } else {
                            mesh.faces?.xFaces?.let { faces += it }
                            mesh.faces?.yFaces?.let { faces += it }
                            mesh.faces?.zFaces?.let { faces += it }

                            mesh.xEdges2?.let { edges += it }
                            mesh.yEdges2?.let { edges += it }
                            mesh.zEdges2?.let { edges += it }
                        }
                    }
                }
            }
        }

        if (toGen.isEmpty()) {
            envContactGenerator.meshFaces.set(faces)
            envContactGenerator.meshEdges.set(edges)
        } else {
            queue += MeshRequest(
                faces = faces,
                edges = edges,
                toGen = toGen,
                envContactGenerator = envContactGenerator,
            )
        }
    }

    private fun tick() {
        positionedMeshes.values.forEach { it.visualize(world) }
        
        var rq: MeshRequest?
        while (queue.poll().also { rq = it } != null) {
            rq ?: return

            for (tg in rq.toGen) {
                genIfNecessary(tg, rq.faces, rq.edges)?.let { positionedMeshes[tg] = it }
            }

            rq.envContactGenerator.meshFaces.set(rq.faces)
            rq.envContactGenerator.meshEdges.set(rq.edges)
        }
    }

    private fun genIfNecessary(
        meshPos: MeshPosition,
        outFaces: MutableList<MeshFaceSortedList>,
        outEdges: MutableList<FlattenedEdgeQuadtree>,
    ): LocalMesher.Mesh2? {
        val existingMesh = positionedMeshes[meshPos]
        if (existingMesh != null) {
            existingMesh.faces?.xFaces?.let { outFaces += it }
            existingMesh.faces?.yFaces?.let { outFaces += it }
            existingMesh.faces?.zFaces?.let { outFaces += it }
            
            existingMesh.xEdges2?.let { outEdges += it }
            existingMesh.yEdges2?.let { outEdges += it }
            existingMesh.zEdges2?.let { outEdges += it }
            
            return null
        }
        
        val meshBB = AABB(
            minX = meshPos.x * MESH_SIZE.toDouble(),
            minY = meshPos.y * MESH_SIZE.toDouble(),
            minZ = meshPos.z * MESH_SIZE.toDouble(),
            maxX = meshPos.x * MESH_SIZE.toDouble() + MESH_SIZE - 1,
            maxY = meshPos.y * MESH_SIZE.toDouble() + MESH_SIZE - 1,
            maxZ = meshPos.z * MESH_SIZE.toDouble() + MESH_SIZE - 1,
        )
        
        val mesh: LocalMesher.Mesh2
        
        val t = measureNanoTime {
            mesh = mesher.mesh(
                world = world,
                boundingBox = meshBB
            )
        }
        
        println("Generated mesh in ${t / 1_000_000.0}us!")
        
        mesh.faces?.xFaces?.let { outFaces += it }
        mesh.faces?.yFaces?.let { outFaces += it }
        mesh.faces?.zFaces?.let { outFaces += it }

        mesh.xEdges2?.let { outEdges += it }
        mesh.yEdges2?.let { outEdges += it }
        mesh.zEdges2?.let { outEdges += it }
        
        return mesh
    }
    
    fun kill() {
        positionedMeshes.clear()
        syncTask?.cancel()
        syncTask = null
    }
}

private class MeshRequest(
    val faces: MutableList<MeshFaceSortedList>,
    val edges: MutableList<FlattenedEdgeQuadtree>,
    val toGen: List<MeshPosition>,
    val envContactGenerator: EnvironmentContactGenerator2,
)

/**
 * In mesh coordinates; real position is origin * MESH_SIZE
 */
class MeshPosition(
    var x: Int,
    var y: Int,
    var z: Int,
) {
    override fun equals(other: Any?): Boolean {
        return other != null && other is MeshPosition && other.x == x && other.y == y && other.z == z
    }

    override fun hashCode(): Int {
        return ((x.toLong() * 713533L) xor (y.toLong() * 328121) xor (z.toLong() * 164341L)).toInt()
    }
}

private const val MESH_SIZE = 16
private const val BB_SAFETY = 1.0