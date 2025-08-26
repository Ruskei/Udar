package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.BlockEntityCuboid
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.physicsWorld
import org.bukkit.Material
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

        val opts = BodyOptions.fromArgs(args)

        val origin = sender.location.toVector().toVector3d()
        val q = Quaterniond().rotateXYZ(opts.rot0.x, opts.rot0.y, opts.rot0.z)
        val o = if (opts.trueOmega) opts.l.rotate(Quaterniond(q).conjugate()) else opts.l
        val rb = Cuboid(
            world = sender.world,
            pos = Vector3d(origin),
            velocity = Vector3d(opts.v0),
            width = opts.dims.x,
            height = opts.dims.y,
            length = opts.dims.z,
            q = q,
            omega = o,
            density = opts.density,
            hasGravity = opts.hasGravity,
        )

        sender.world.physicsWorld?.registerBody(BlockEntityCuboid(rb, Material.GLASS))

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