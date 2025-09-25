package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.CompositeImpl
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.body.active.blockEntity
import com.ixume.udar.physicsWorld
import org.bukkit.Material
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaterniond
import org.joml.Vector3d

object CompositeCommand : Command {
    override val arg: String = "composite"
    override val description: String = "Composite object testing"

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?,
    ): List<String> {
        return listOf("t-handle", "coalesce")
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true

        if (args.isEmpty()) return true

        val physicsWorld = sender.world.physicsWorld ?: return true
        val origin = sender.location.toVector().toVector3d()

        when (args[0]) {
            "t-handle" -> {
                val opts =
                    if (args.size < 2) BodyOptions.default() else BodyOptions.fromArgs(args.copyOfRange(1, args.size))

                val q = Quaterniond().rotateXYZ(opts.rot0.x, opts.rot0.y, opts.rot0.z)
                val o = if (opts.trueOmega) opts.l.rotate(Quaterniond(q).conjugate()) else opts.l

                val cuboid = Cuboid(
                    world = sender.world,
                    pos = Vector3d(origin),
                    velocity = Vector3d(0.0),
                    width = 1.0,
                    height = 8.0,
                    length = 1.0,
                    q = Quaterniond(),
                    omega = Vector3d(0.0, 0.0, 0.0),
                    density = 1.0,
                    hasGravity = false,
                ).blockEntity(Material.DARK_PRISMARINE)

                cuboid.isChild = true

                val cuboid2 = Cuboid(
                    world = sender.world,
                    pos = Vector3d(origin).add(0.0, 0.0, 3.0),
                    velocity = Vector3d(0.0),
                    width = 1.0,
                    height = 1.0,
                    length = 5.0,
                    q = Quaterniond(),
                    omega = Vector3d(0.0, 0.0, 0.0),
                    density = 1.0,
                    hasGravity = false,
                ).blockEntity(Material.DARK_PRISMARINE)

                cuboid2.isChild = true

                val composite = CompositeImpl(
                    world = sender.world,
                    velocity = opts.v0,
                    q = Quaterniond().rotateXYZ(
                        Math.PI * 0.5,
                        0.0,
                        0.0
                    ),
                    omega = o,
                    hasGravity = opts.hasGravity,
                    parts = listOf(cuboid, cuboid2),
                    origin = origin
                )

                physicsWorld.registerBody(composite)
            }

            "coalesce" -> {
                if (args.size < 2) return true

                val num = args[1].toIntOrNull() ?: return true

                val ss = physicsWorld.bodiesSnapshot().sortedBy { it.pos.distance(origin) }

                val bodies = ss.take(num)

                val opts = if (args.size < 3) BodyOptions.default() else BodyOptions.fromArgs(
                    args.copyOfRange(
                        2,
                        args.size - 1
                    )
                )
                val q = Quaterniond().rotateXYZ(opts.rot0.x, opts.rot0.y, opts.rot0.z)
                val o = if (opts.trueOmega) opts.l.rotate(Quaterniond(q).conjugate()) else opts.l

                bodies.forEach { it.isChild = true }

                val rb = CompositeImpl(
                    world = sender.world,
                    velocity = Vector3d(opts.v0),
                    q = q,
                    omega = o,
                    hasGravity = opts.hasGravity,
                    parts = bodies,
                    origin = origin,
                )

                sender.world.physicsWorld?.registerBody(rb)
            }
        }

        return true
    }
}