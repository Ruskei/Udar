package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.contactgeneration.Meshes
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.AABB
import org.bukkit.Bukkit
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3i
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.math.floor
import kotlin.system.measureNanoTime
import kotlin.time.DurationUnit
import kotlin.time.toDuration

/*
this will return a list of face lists and a list of edge trees to the requester

every tick, run a checksum on every mesh, see if it changed; if it did, then update that chunk. for this we must also maintain a mapping MeshPosition -> EnvContactGen
 */

class WorldMeshesManager(
    val physicsWorld: PhysicsWorld,
) {
    val world = physicsWorld.world

    private val mesher = LocalMesher()
    private val positionedMeshes = ConcurrentHashMap<MeshPosition, LocalMesher.Mesh2>()
    private val envContactGenerators = ConcurrentHashMap.newKeySet<EnvironmentContactGenerator2>()
    //TODO: fix mem leak

    private val queue = ConcurrentLinkedQueue<MeshRequest>()
    private var syncTask: BukkitTask? = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, ::tick, 1, 1)

    fun request(prevBB: AABB?, currentBB: AABB, envContactGenerator: EnvironmentContactGenerator2) {
        envContactGenerators.add(envContactGenerator)
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

        val meshes = Meshes()

        if (prevBB != null) {
//            val prevMinX = floor((prevBB.minX - BB_SAFETY) / MESH_SIZE).toInt()
//            val prevMinY = floor((prevBB.minY - BB_SAFETY) / MESH_SIZE).toInt()
//            val prevMinZ = floor((prevBB.minZ - BB_SAFETY) / MESH_SIZE).toInt()
//
//            val prevMaxX = floor((prevBB.maxX + BB_SAFETY) / MESH_SIZE).toInt()
//            val prevMaxY = floor((prevBB.maxY + BB_SAFETY) / MESH_SIZE).toInt()
//            val prevMaxZ = floor((prevBB.maxZ + BB_SAFETY) / MESH_SIZE).toInt()
//
            val currentMinX = floor((currentBB.minX - BB_SAFETY) / MESH_SIZE).toInt()
            val currentMinY = floor((currentBB.minY - BB_SAFETY) / MESH_SIZE).toInt()
            val currentMinZ = floor((currentBB.minZ - BB_SAFETY) / MESH_SIZE).toInt()
            val currentMaxX = floor((currentBB.maxX + BB_SAFETY) / MESH_SIZE).toInt()
            val currentMaxY = floor((currentBB.maxY + BB_SAFETY) / MESH_SIZE).toInt()
            val currentMaxZ = floor((currentBB.maxZ + BB_SAFETY) / MESH_SIZE).toInt()

//            // check if current is either equal to or contained
//            val isStillValid = prevMinX >= currentMinX && prevMinY >= currentMinY && prevMinZ >= currentMinZ &&
//                               prevMaxX <= currentMaxX && prevMaxY <= currentMaxY && prevMaxZ <= currentMaxZ
//
//            if (isStillValid)

            for (x in currentMinX..currentMaxX) {
                for (y in currentMinY..currentMaxY) {
                    for (z in currentMinZ..currentMaxZ) {
                        val meshPos = MeshPosition(x, y, z, world)

                        val mesh = positionedMeshes[meshPos]

                        if (mesh == null) {
                            toGen += meshPos
                        } else {
                            // we need to regenerate mesh if any of the blocks around the body have changed; for this we can simply keep a map of hashes of blockstate
                            // if a difference is found here, then we order it for regeneration
                            meshes.addMesh(meshPos, mesh)
                        }
                    }
                }
            }
        }

        if (toGen.isEmpty()) {
            envContactGenerator.meshes.set(meshes)
        } else {
            queue += MeshRequest(
                meshes = meshes,
                toGen = toGen,
                envContactGenerator = envContactGenerator,
            )
        }
    }

    private fun tick() {
        val t = measureNanoTime {
//            positionedMeshes.forEach { it.value.visualize(world) }

            var rq: MeshRequest?
            while (queue.poll().also { rq = it } != null) {
                rq ?: return

                for (tg in rq.toGen) {
                    genAndAdd2Meshes(tg, rq.meshes)
                }

                rq.envContactGenerator.meshes.set(rq.meshes)
            }

            val mpsToUpdate = mutableMapOf<MeshPosition, MutableList<EnvironmentContactGenerator2>>()
            for (gen in envContactGenerators) {
                val currState = mutableMapOf<Vector3i, Long>()
                val bb = gen.activeBody.tightBB
                for (x in (floor(bb.minX).toInt() - 2)..(floor(bb.maxX).toInt() + 2)) {
                    for (y in (floor(bb.minY).toInt() - 2)..(floor(bb.maxY).toInt() + 2)) {
                        for (z in (floor(bb.minZ).toInt() - 2)..(floor(bb.maxZ).toInt() + 2)) {
                            var sum = 1L

                            val block = world.getBlockAt(x, y, z)
                            if (block.isPassable) {
                                currState[Vector3i(x, y, z)] = 0L
                                continue
                            }

                            for (boundingBox in block.collisionShape.boundingBoxes) {
                                sum = rollingVec3Checksum(
                                    sum,
                                    x + boundingBox.minX,
                                    y + boundingBox.minY,
                                    z + boundingBox.minZ,
                                )
                                sum = rollingVec3Checksum(
                                    sum,
                                    x + boundingBox.maxX,
                                    y + boundingBox.maxY,
                                    z + boundingBox.maxZ,
                                )
                            }

                            currState[Vector3i(x, y, z)] = sum
                        }
                    }
                }

                var diff = false
                for ((key, state) in currState) {
                    for (x in floor((key.x.toDouble() - 1) / MESH_SIZE).toInt()..floor((key.x.toDouble() + 1) / MESH_SIZE).toInt()) {
                        for (y in floor((key.y.toDouble() - 1) / MESH_SIZE).toInt()..floor((key.y.toDouble() + 1) / MESH_SIZE).toInt()) {
                            for (z in floor((key.z.toDouble() - 1) / MESH_SIZE).toInt()..floor((key.z.toDouble() + 1) / MESH_SIZE).toInt()) {
                                val mp =  MeshPosition(x, y, z, world)
                                val sum = (positionedMeshes[mp] ?: continue).states[key]
                                if (sum != null && sum != state) {
//                                    println("DIFF!")
                                    diff = true
                                    mpsToUpdate.getOrPut(mp) { ArrayList() } += gen
                                }
                            }
                        }
                    }

                }

                if (diff) {
                    gen.startle()
                }
            }

            var updated = 0
            for ((mp, gens) in mpsToUpdate) {
                updated++
                val mesh = mp.genMesh()
                positionedMeshes[mp] = mesh
                for (g in gens) {
                    g.meshes.updateAndGet {
                        val c = it.clone()
                        c.addMesh(mp, mesh)
                        c
                    }
                }
            }
//
//            println("Updated $updated meshes!")
        }

        val d = t.toDuration(DurationUnit.NANOSECONDS)
//        println("Mesh manager tick took $d")
    }

    private fun genAndAdd2Meshes(
        meshPos: MeshPosition,
        outMeshes: Meshes,
    ): LocalMesher.Mesh2? {
        val existingMesh = positionedMeshes[meshPos]
        if (existingMesh != null) {
            outMeshes.addMesh(meshPos, existingMesh)
            return null
        }

        val mesh = meshPos.genMesh()
        positionedMeshes[meshPos] = mesh

        outMeshes.addMesh(meshPos, mesh)
        return mesh
    }

    fun MeshPosition.genMesh(): LocalMesher.Mesh2 {
        val meshBB = AABB(
            minX = x * MESH_SIZE.toDouble(),
            minY = y * MESH_SIZE.toDouble(),
            minZ = z * MESH_SIZE.toDouble(),
            maxX = x * MESH_SIZE.toDouble() + MESH_SIZE,
            maxY = y * MESH_SIZE.toDouble() + MESH_SIZE,
            maxZ = z * MESH_SIZE.toDouble() + MESH_SIZE,
        )

        val mesh: LocalMesher.Mesh2

        val t = measureNanoTime {
            mesh = mesher.mesh(
                world = world,
                boundingBox = meshBB
            )
        }

//        println("Generated mesh in ${t / 1_000_000.0}ms!")

        return mesh
    }

    fun kill() {
        positionedMeshes.clear()
        syncTask?.cancel()
        syncTask = null
    }
}

private class MeshRequest(
    val meshes: Meshes,
    val toGen: List<MeshPosition>,
    val envContactGenerator: EnvironmentContactGenerator2,
)

/**
 * In mesh coordinates; real position is origin * MESH_SIZE
 */
class MeshPosition(
    val x: Int,
    val y: Int,
    val z: Int,
    val world: World,
) {
    override fun equals(other: Any?): Boolean {
        return other != null && other is MeshPosition && other.x == x && other.y == y && other.z == z
    }

    override fun hashCode(): Int {
        return ((x.toLong() * 713533L) xor (y.toLong() * 328121) xor (z.toLong() * 164341L)).toInt()
    }
}

fun rollingVec3Checksum(sum: Long, x: Double, y: Double, z: Double): Long {
    var result = sum
    val prime = 31L

    // Fold in the vector's components one by one
    result = result * prime + x.toRawBits()
    result = result * prime + y.toRawBits()
    result = result * prime + z.toRawBits()

    return result
}

private const val MESH_SIZE = 32
private const val BB_SAFETY = 4.0