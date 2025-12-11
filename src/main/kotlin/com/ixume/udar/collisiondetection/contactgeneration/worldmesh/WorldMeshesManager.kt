package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import com.ixume.udar.collisiondetection.multithreading.runPartitioned
import net.minecraft.core.BlockPos
import net.minecraft.server.level.ServerLevel
import net.minecraft.world.level.block.state.BlockState
import org.bukkit.Bukkit
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
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
    val diffingProcessors: Int,
    val meshingProcessors: Int,
) {
    val world = physicsWorld.world

    private val diffingExecutor = Executors.newFixedThreadPool(diffingProcessors)
    private val diffingCallables = Array(diffingProcessors) { DiffingRunnable() }

    private val meshingExecutor = Executors.newFixedThreadPool(meshingProcessors)
    private val mesherCallables = Array(meshingProcessors) { MeshRunnable() }

    private val positionedMeshes = HashMap<MeshPosition, LocalMesher.Mesh2>()
    val envContactGenerators = CopyOnWriteArrayList<EnvironmentContactGenerator2>()

    private var task: BukkitTask? = Bukkit.getScheduler().runTaskTimerAsynchronously(Udar.INSTANCE, ::tick, 1, 1)

    private val busy = AtomicBoolean(false)

    private var averageTickTime = 0.0
    private var time = 0

    private fun tick() {
        if (!busy.compareAndSet(false, true)) {
            return
        }
        time++

        val t = measureNanoTime {
            val snapshot = envContactGenerators.toTypedArray()
            time("diffing_multi") {
                diffingExecutor.runPartitioned(snapshot.size, diffingCallables) { start, end ->
                    this.out.clear()
                    this.targets = snapshot
                    this.world = this@WorldMeshesManager.world
                    this.positionedMeshes = this@WorldMeshesManager.positionedMeshes
                    this.start = start
                    this.end = end
                }
            }

            val mpsToGen = mutableMapOf<MeshPosition, MutableList<EnvironmentContactGenerator2>>()

            time("diffing_join") {
                var i = 0
                while (i < diffingCallables.size) {
                    val callable = diffingCallables[i]
                    for ((mp, arr) in callable.out) {
                        val existingList = mpsToGen[mp]
                        if (existingList == null) {
                            val ls = mutableListOf<EnvironmentContactGenerator2>()
                            for (j in 0..<arr.size) {
                                ls += snapshot[arr.getInt(j)]
                            }
                            mpsToGen[mp] = ls
                        } else {
                            for (j in 0..<arr.size) {
                                existingList += snapshot[arr.getInt(j)]
                            }
                        }
                    }

                    i++
                }
            }

            val mpList = mpsToGen.keys.toList()
            time("meshing_multi") {
                meshingExecutor.runPartitioned(mpList.size, mesherCallables) { start, end ->
                    out.clear()
                    this.mps = mpList
                    this.start = start
                    this.end = end
                }
            }

            time("meshing_merge") {
                for (proc in 0..<meshingProcessors) {
                    val callable = mesherCallables[proc]
                    var meshIdx = 0
                    if (callable.start >= callable.end) continue
                    for (i in callable.start..<callable.end) {
                        val mp = callable.mps[i]
                        positionedMeshes[mp] = callable.out[meshIdx]

                        val gens = mpsToGen[mp]!!
                        for (g in gens) {
                            g.meshes.addMesh(mp, callable.out[meshIdx])
                        }

                        meshIdx++
                    }
                }
            }
        }
//
        averageTickTime += t.toDouble() / Udar.CONFIG.debug.timingsWorldReportInterval
        if (time % Udar.CONFIG.debug.timingsWorldReportInterval == 0) {
            if (Udar.CONFIG.debug.timings) {
                println("Average tick time: ${averageTickTime.toDuration(DurationUnit.NANOSECONDS)}")
            }

            averageTickTime = 0.0
        }
        busy.set(false)
    }

    fun kill() {
        positionedMeshes.clear()
        task?.cancel()
        task = null
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
const val BB_SAFETY = 8
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

@OptIn(ExperimentalContracts::class)
private inline fun time(name: String, block: () -> Unit) {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }

    block()
}