package com.ixume.udar.collisiondetection.contactgeneration.worldmesh

import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher
import it.unimi.dsi.fastutil.ints.IntArrayList
import net.minecraft.core.BlockPos
import org.bukkit.World
import org.bukkit.craftbukkit.CraftWorld
import kotlin.math.floor

class DiffingRunnable : Runnable {
    lateinit var targets: Array<EnvironmentContactGenerator2>
    lateinit var world: World
    lateinit var positionedMeshes: Map<MeshPosition, LocalMesher.Mesh2>
    var start = 0
    var end = 0

    private val _bp = BlockPos(0, 0, 0).mutable()

    // mp -> target idx
    val out = mutableMapOf<MeshPosition, IntArrayList>()

    override fun run() {
        if (start >= end) {
            return
        }

        val nmsWorld = (world as CraftWorld).handle

        var i = start
        while (i < end) {
            val gen = targets[i]

            val bb = gen.activeBody.tightBB

            val minX = floor(bb.minX).toInt() - 1
            val minY = floor(bb.minY).toInt() - 1
            val minZ = floor(bb.minZ).toInt() - 1

            val maxX = (floor(bb.maxX).toInt() + 2)
            val maxY = (floor(bb.maxY).toInt() + 2)
            val maxZ = (floor(bb.maxZ).toInt() + 2)

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
                            if (state == -1L) {
                                continue
                            }

                            if (isPassable && state != 0L) {
                                // is now air
                                diff = true
                                val existingMesh = positionedMeshes[mp]
                                if (existingMesh != null && existingMesh.stateAt(x, y, z) != 0L) {
                                    out.getOrPut(mp) { IntArrayList() } += i
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
                                        out.getOrPut(mp) { IntArrayList() } += i
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
                                val existingMesh = positionedMeshes[mp]
                                if (existingMesh != null && existingMesh.stateAt(x, y, z) != sum) {
                                    out.getOrPut(mp) { IntArrayList() } += i
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
            world.forEachOverlappingMeshPos(
                minX - BB_SAFETY, minY - BB_SAFETY, minZ - BB_SAFETY,
                maxX + BB_SAFETY, maxY + BB_SAFETY, maxZ + BB_SAFETY,
            ) {
                val mesh = positionedMeshes[it]
                if (mesh == null) {
                    out.getOrPut(it) { IntArrayList() } += i
                } else {
                    if (!gen.meshes.map.containsKey(it)) {
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

            var j = 0
            while (j < toRemove.size) {
                gen.meshes.removeMesh(toRemove[j])
                j++
            }

            if (diff) {
                gen.startle()
            }

            i++
        }
    }
}

private inline fun World.forEachOverlappingMeshPos(
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
                action(MeshPosition(meshX, meshY, meshZ, this))
            }
        }
    }
}
