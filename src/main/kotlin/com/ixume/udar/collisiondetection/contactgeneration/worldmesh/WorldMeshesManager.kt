package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.AABB
import org.bukkit.Bukkit
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CopyOnWriteArraySet
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
    val envContactGenerators = CopyOnWriteArraySet<EnvironmentContactGenerator2>()

    private var syncTask: BukkitTask? = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, ::tick, 1, 1)

    private fun tick() {
        val t = measureNanoTime {
//            positionedMeshes.forEach { it.value.visualize(world) }
            val mpsToGen = mutableMapOf<MeshPosition, MutableSet<EnvironmentContactGenerator2>>()
            for (gen in envContactGenerators) {
                val bb = gen.activeBody.tightBB

                val minX = floor(bb.minX).toInt()
                val minY = floor(bb.minY).toInt()
                val minZ = floor(bb.minZ).toInt()

                val maxX = (floor(bb.maxX).toInt() + 1)
                val maxY = (floor(bb.maxY).toInt() + 1)
                val maxZ = (floor(bb.maxZ).toInt() + 1)

                var diff = false

                for (x in minX..maxX) {
                    for (y in minY..maxY) {
                        for (z in minZ..maxZ) {
                            var sum = 1L
                            var calculatedSum = false

                            val block = world.getBlockAt(x, y, z)
                            val isPassable = block.isPassable

                            forEachMeshPosAt(x, y, z) {
                                val mesh = positionedMeshes[it]
                                if (mesh == null) {
                                    mpsToGen.getOrPut(it) { HashSet() } += gen
                                } else {
                                    val state = mesh.stateAt(x, y, z)
                                    if (isPassable && state != 0L) {
                                        // is now air
                                        diff = true
                                        mpsToGen.getOrPut(it) { HashSet() } += gen
                                    } else if (isPassable) {
                                        // is air and was air
                                        if (!gen.meshes.map.containsKey(it)) {
                                            gen.meshes.addMesh(it, mesh)
                                        }

                                        return@forEachMeshPosAt
                                    }

                                    if (state == 0L) {
                                        // was air
                                        diff = true
                                        mpsToGen.getOrPut(it) { HashSet() } += gen
                                        return@forEachMeshPosAt
                                    }

                                    if (!calculatedSum) {
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

                                        calculatedSum = true
                                    }

                                    if (sum != state) { // diff block
                                        diff = true
                                        mpsToGen.getOrPut(it) { HashSet() } += gen
                                        return@forEachMeshPosAt
                                    }

                                    // everything is the same and mesh exists, so do nothing if it's already contained in map, otherwise, add it
                                    if (!gen.meshes.map.containsKey(it)) {
                                        gen.meshes.addMesh(it, mesh)
                                    }
                                }
                            }
                        }
                    }
                }

                // get everything nearby
                forEachOverlappingMeshPos(
                    minX - BB_SAFETY, minX - BB_SAFETY, minX - BB_SAFETY,
                    maxX + BB_SAFETY, maxX + BB_SAFETY, maxX + BB_SAFETY,
                ) {
                    val mesh = positionedMeshes[it]
                    if (mesh == null) {
                        mpsToGen.getOrPut(it) { HashSet() } += gen
                    } else {
                        if (!gen.meshes.map.containsKey(it)) {
                            gen.meshes.addMesh(it, mesh)
                        }
                    }
                }

                if (diff) {
                    gen.startle()
                }
            }

            var updated = 0
            for ((mp, gens) in mpsToGen) {
                updated++
                val mesh = mp.genMesh()
                positionedMeshes[mp] = mesh
                for (g in gens) {
                    g.meshes.addMesh(mp, mesh)
                }
            }
//
//            println("Updated $updated meshes!")
        }

//        val d = t.toDuration(DurationUnit.NANOSECONDS)
//        println("Mesh manager tick took $d")
    }

    private inline fun forEachMeshPosAt(x: Int, y: Int, z: Int, action: (MeshPosition) -> Unit) {
        for (meshX in floor((x.toDouble() - 1) / MESH_SIZE).toInt()..floor((x.toDouble() + 1) / MESH_SIZE).toInt()) {
            for (meshY in floor((y.toDouble() - 1) / MESH_SIZE).toInt()..floor((y.toDouble() + 1) / MESH_SIZE).toInt()) {
                for (meshZ in floor((z.toDouble() - 1) / MESH_SIZE).toInt()..floor((z.toDouble() + 1) / MESH_SIZE).toInt()) {
                    if (
                        x in (meshX * MESH_SIZE)..(meshX * MESH_SIZE + MESH_SIZE) &&
                        y in (meshY * MESH_SIZE)..(meshY * MESH_SIZE + MESH_SIZE) &&
                        z in (meshZ * MESH_SIZE)..(meshZ * MESH_SIZE + MESH_SIZE)
                    ) {
                        action(MeshPosition(meshX, meshY, meshZ, world))
                    }
                }
            }
        }
    }

    private inline fun forEachOverlappingMeshPos(
        minX: Int,
        minY: Int,
        minZ: Int,
        maxX: Int,
        maxY: Int,
        maxZ: Int,
        action: (MeshPosition) -> Unit,
    ) {
        for (meshX in floor(minX.toDouble() / MESH_SIZE).toInt()..floor(maxX.toDouble() / MESH_SIZE).toInt()) {
            for (meshY in floor(minY.toDouble() / MESH_SIZE).toInt()..floor(maxY.toDouble() / MESH_SIZE).toInt()) {
                for (meshZ in floor(minZ.toDouble() / MESH_SIZE).toInt()..floor(maxZ.toDouble() / MESH_SIZE).toInt()) {
                    action(MeshPosition(meshX, meshY, meshZ, world))
                }
            }
        }
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

        println("Generated mesh in ${t.toDuration(DurationUnit.NANOSECONDS)}!")
        println("| at: ${x * MESH_SIZE.toDouble()} ${y * MESH_SIZE.toDouble()} ${z * MESH_SIZE.toDouble()}")
        return mesh
    }

    fun kill() {
        positionedMeshes.clear()
        syncTask?.cancel()
        syncTask = null
    }
}

/**
 * In mesh coordinates; real position is origin * MESH_SIZE
 */
@JvmRecord
data class MeshPosition(
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
private const val BB_SAFETY = 8