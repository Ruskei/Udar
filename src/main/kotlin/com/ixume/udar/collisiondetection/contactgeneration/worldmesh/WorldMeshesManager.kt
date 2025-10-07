package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.dynamicaabb.AABB
import net.minecraft.core.BlockPos
import net.minecraft.server.level.ServerLevel
import net.minecraft.world.level.block.state.BlockState
import org.bukkit.Bukkit
import org.bukkit.World
import org.bukkit.craftbukkit.CraftWorld
import org.bukkit.scheduler.BukkitTask
import java.util.concurrent.CopyOnWriteArraySet
import java.util.concurrent.atomic.AtomicBoolean
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
    private val positionedMeshes = HashMap<MeshPosition, LocalMesher.Mesh2>()
    val envContactGenerators = CopyOnWriteArraySet<EnvironmentContactGenerator2>()

    private var syncTask: BukkitTask? = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, ::tick, 1, 1)

    private val busy = AtomicBoolean(false)
    private val _bp = BlockPos(0, 0, 0).mutable()

    private fun tick() {
        if (!busy.compareAndSet(false, true)) {
            return
        }

        val nmsWorld = (world as CraftWorld).handle
//        println("TICK")
        val t = measureNanoTime {
//            positionedMeshes.forEach { it.value.visualize(world) }
            val mpsToGen = mutableMapOf<MeshPosition, MutableSet<EnvironmentContactGenerator2>>()
            for (gen in envContactGenerators) {
//                println(" * ${gen.activeBody.idx}")
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

                            _bp.set(x, y, z)
                            val nmsBlock = nmsWorld.fastBlockAt(_bp)
                            val isPassable = !nmsBlock.block.hasCollision

                            for ((mp, mesh) in gen.meshes.map) {
                                if (!(
                                            x in (mp.x * MESH_SIZE)..(mp.x * MESH_SIZE + MESH_SIZE) &&
                                            y in (mp.y * MESH_SIZE)..(mp.y * MESH_SIZE + MESH_SIZE) &&
                                            z in (mp.z * MESH_SIZE)..(mp.z * MESH_SIZE + MESH_SIZE))
                                ) {
                                    continue
                                }


                                val state = mesh.stateAt(x, y, z)
                                if (isPassable && state != 0L) {
                                    // is now air
                                    diff = true
//                                    println("Diff at ($x $y $z), mp: (${mp.x} ${mp.y} ${mp.z})")
                                    val existingMesh = positionedMeshes[mp]
                                    if (existingMesh != null && existingMesh.stateAt(x, y, z) != 0L) {
                                        mpsToGen.getOrPut(mp) { HashSet() } += gen
                                    } else if (existingMesh != null) {
                                        // update object's mesh to match positionedMeshes
                                        gen.meshes.addMesh(mp, existingMesh)
                                    }

                                    continue
                                } else if (isPassable) {
                                    // is air and was air
                                    continue
                                }

                                if (state == 0L) {
                                    // was air
                                    diff = true
//                                    println("Diff at ($x $y $z), mp: (${mp.x} ${mp.y} ${mp.z})")
                                    val existingMesh = positionedMeshes[mp]
                                    if (existingMesh != null) {
                                        if (!calculatedSum) {
                                            nmsBlock.getCollisionShape(
                                                nmsWorld,
                                                _bp,
                                            )
                                                .forAllBoxes { minX, minY, minZ, maxX, maxY, maxZ ->
                                                    sum = rollingVec3Checksum(
                                                        sum,
                                                        x + minX,
                                                        y + minY,
                                                        z + minZ,
                                                    )
                                                    sum = rollingVec3Checksum(
                                                        sum,
                                                        x + maxX,
                                                        y + maxY,
                                                        z + maxZ,
                                                    )
                                                }

                                            calculatedSum = true
                                        }

                                        if (existingMesh.stateAt(x, y, z) == sum) {
                                            gen.meshes.addMesh(mp, existingMesh)
                                        } else {
                                            mpsToGen.getOrPut(mp) { HashSet() } += gen
                                        }
                                    }

                                    continue
                                }

                                if (!calculatedSum) {

                                    nmsBlock.getCollisionShape(
                                        nmsWorld,
                                        _bp,
                                    ).forAllBoxes { minX, minY, minZ, maxX, maxY, maxZ ->
                                        sum = rollingVec3Checksum(
                                            sum,
                                            x + minX,
                                            y + minY,
                                            z + minZ,
                                        )
                                        sum = rollingVec3Checksum(
                                            sum,
                                            x + maxX,
                                            y + maxY,
                                            z + maxZ,
                                        )
                                    }

                                    calculatedSum = true
                                }

                                if (sum != state) { // diff block
                                    diff = true
//                                    println("Diff at ($x $y $z), mp: (${mp.x} ${mp.y} ${mp.z})")
                                    val existingMesh = positionedMeshes[mp]
                                    if (existingMesh != null && existingMesh.stateAt(x, y, z) != sum) {
                                        mpsToGen.getOrPut(mp) { HashSet() } += gen
                                    } else if (existingMesh != null) {
                                        // update object's mesh to match positionedMeshes
                                        gen.meshes.addMesh(mp, existingMesh)
                                    }
                                    continue
                                }
                            }
                        }
                    }
                }

                // get everything nearby
                forEachOverlappingMeshPos(
                    minX - BB_SAFETY, minY - BB_SAFETY, minZ - BB_SAFETY,
                    maxX + BB_SAFETY, maxY + BB_SAFETY, maxZ + BB_SAFETY,
                ) {
//                    println(" | $it")
                    val mesh = positionedMeshes[it]
                    if (mesh == null) {
//                        println("${gen.activeBody.idx} genned (${it.x} ${it.y} ${it.z})")
                        mpsToGen.getOrPut(it) { HashSet() } += gen
                    } else {
                        if (!gen.meshes.map.containsKey(it)) {
//                            println("${gen.activeBody.idx} needed (${it.x} ${it.y} ${it.z})")
                            gen.meshes.addMesh(it, mesh)
                        }
                    }
                }

                val toRemove = mutableListOf<MeshPosition>()
                for (mp in gen.meshes.map.keys) {
                    if (!mp.isRelevant(
                            minX - BB_SAFETY, minY - BB_SAFETY, minZ - BB_SAFETY,
                            maxX + BB_SAFETY, maxY + BB_SAFETY, maxZ + BB_SAFETY,
                        )
                    ) {
                        toRemove += mp
                    }
                }

                var i = 0
                while (i < toRemove.size) {
                    gen.meshes.removeMesh(toRemove[i])
                    i++
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
//
//        val d = t.toDuration(DurationUnit.NANOSECONDS)
//        println("Mesh manager tick took $d")
        busy.set(false)
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
    fun isRelevant(
        minX: Int,
        minY: Int,
        minZ: Int,
        maxX: Int,
        maxY: Int,
        maxZ: Int,
    ): Boolean {
        return x in floor(minX.toDouble() / MESH_SIZE).toInt()..floor(maxX.toDouble() / MESH_SIZE).toInt() &&
               y in floor(minY.toDouble() / MESH_SIZE).toInt()..floor(maxY.toDouble() / MESH_SIZE).toInt() &&
               z in floor(minZ.toDouble() / MESH_SIZE).toInt()..floor(maxZ.toDouble() / MESH_SIZE).toInt()
    }

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

const val MESH_SIZE = 32
private const val BB_SAFETY = 8
/*
    public BlockState getBlock(Location location) {
        Preconditions.checkNotNull(location);

        int x = location.getBlockX();
        int y = location.getBlockY();
        int z = location.getBlockZ();
        final ServerLevel handle = getServerLevel(location.getWorld());
        LevelChunk chunk = handle.getChunk(x >> 4, z >> 4);
        final BlockPos blockPos = new BlockPos(x, y, z);
        final net.minecraft.world.level.block.state.BlockState blockData = chunk.getBlockState(blockPos);
        BlockState state = adapt(blockData);
        if (state == null) {
            org.bukkit.block.Block bukkitBlock = location.getBlock();
            state = BukkitAdapter.adapt(bukkitBlock.getBlockData());
        }
        return state;
    }

 */

fun ServerLevel.fastBlockAt(bp: BlockPos): BlockState {
    val chunk = getChunk(bp.x shr 4, bp.z shr 4)
    return chunk.getBlockState(bp)
}