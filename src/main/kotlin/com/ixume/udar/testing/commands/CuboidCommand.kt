package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.physicsWorld
import org.bukkit.command.Command
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaterniond
import org.joml.Vector3d

object CuboidCommand : com.ixume.udar.testing.commands.Command {
    override val arg: String = "cube"
    override val description: String = "--velocity|-v <x> <y> <z> --dims|-d <x> <y> <z> --omega|-o <x> <y> <z> --rot|-r <x> <y> <z> --density <d> --gravity|-g --no-gravity|-ng --true-omega|-to; Spawns cuboid"

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        alias: String,
        args: Array<out String>
    ): Boolean {
        if (sender !is Player) return true

        var v0 = Vector3d(0.0)
        var dims = Vector3d(0.99)
        var l = Vector3d(0.0)
        var rot0 = Vector3d(0.0)

        var density = 1.0
        var hasGravity = true
        var SAT = false
        var trueOmega = false

        var i = 0
        while (i < args.size) {
            val arg = args[i]

            if (arg == "--velocity" || arg == "-v") {
                val buf = mutableListOf<Double>()
                i++
                while (i < args.size) {
                    val arg2 = args[i]
                    val d = arg2.toDoubleOrNull()
                    if (d == null) {
                        i--;
                        break
                    }

                    buf += d

                    if (buf.size == 3) {
                        v0 = Vector3d(buf[0], buf[1], buf[2])
                        break
                    }

                    i++
                }
            }

            if (arg == "--dims" || arg == "-d") {
                val buf = mutableListOf<Double>()
                i++
                while (i < args.size) {
                    val arg2 = args[i]
                    val d = arg2.toDoubleOrNull()
                    if (d == null) {
                        i--;
                        break
                    }

                    buf += d

                    if (buf.size == 3) {
                        dims = Vector3d(buf[0], buf[1], buf[2])
                        break
                    }

                    i++
                }
            }

            if (arg == "--omega" || arg == "-o") {
                val buf = mutableListOf<Double>()
                i++
                while (i < args.size) {
                    val arg2 = args[i]
                    val d = arg2.toDoubleOrNull()
                    if (d == null) {
                        i--;
                        break
                    }

                    buf += d

                    if (buf.size == 3) {
                        l = Vector3d(buf[0], buf[1], buf[2])
                        break
                    }

                    i++
                }
            }

            if (arg == "--rot" || arg == "-r") {
                val buf = mutableListOf<Double>()
                i++
                while (i < args.size) {
                    val arg2 = args[i]
                    val d = arg2.toDoubleOrNull()
                    if (d == null) {
                        i--;
                        break
                    }

                    buf += d

                    if (buf.size == 3) {
                        rot0 = Vector3d(buf[0], buf[1], buf[2])
                        break
                    }

                    i++
                }
            }

            if (arg == "--density") {
                i++
                val arg2 = args[i]
                val d = arg2.toDoubleOrNull()
                if (d == null) {
                    i--
                } else {
                    density = d
                }
            }

            if (arg == "--gravity" || arg == "-g") {
                hasGravity = true
            }

            if (arg == "--no-gravity" || arg == "-ng") {
                hasGravity = false
            }

            if (arg == "--sat") {
                SAT = true
            }

            if (arg == "--true-omega" || arg == "-to") {
                trueOmega = true
            }

            i++
        }

        val origin = sender.location.toVector().toVector3d()
        val q = Quaterniond().rotateXYZ(rot0.x, rot0.y, rot0.z)
        val o = if (trueOmega) l.rotate(Quaterniond(q).conjugate()) else l
        val rb = if (SAT) Cuboid(
            world = sender.world,
            pos = Vector3d(origin),
            velocity = Vector3d(),
            width = dims.x,
            height = dims.y,
            length = dims.z,
            q = Quaterniond(),
            omega = Vector3d(),
            density = density,
            hasGravity = hasGravity,
        ) else Cuboid(
            world = sender.world,
            pos = Vector3d(origin),
            velocity = Vector3d(v0),
            width = dims.x,
            height = dims.y,
            length = dims.z,
            q = q,
            omega = o,
            density = density,
            hasGravity = hasGravity,
        )

        sender.world.physicsWorld?.activeBodies += rb

        return true
    }

    override fun onTabComplete(
        sender: CommandSender,
        command: Command,
        label: String,
        args: Array<out String>
    ): List<String>? {
        return emptyList()
    }
}