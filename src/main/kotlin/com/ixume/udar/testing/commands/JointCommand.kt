package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.BlockEntityCuboid
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.body.active.tag.Tag
import com.ixume.udar.physicsWorld
import org.bukkit.Material
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaterniond
import org.joml.Vector3d
import kotlin.random.Random

object JointCommand : Command {
    override val arg: String = "joint"
    override val description: String = ""

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        alias: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true

        val opts = BodyOptions.fromArgs(args)

        val tag = Tag("joint_" + Random.nextInt(), collide = false);
        val origin = sender.location.toVector().toVector3d()
        val q = Quaterniond().rotateXYZ(opts.rot0.x, opts.rot0.y, opts.rot0.z)
        val o = if (opts.trueOmega) opts.l.rotate(Quaterniond(q).conjugate()) else opts.l
        val bottom = Cuboid(
            world = sender.world,
            pos = Vector3d(origin),
            velocity = Vector3d(opts.v0),
            width = opts.dims.x,
            height = opts.dims.y,
            length = opts.dims.z,
            q = Quaterniond(q),
            omega = Vector3d(o),
            density = opts.density,
            hasGravity = opts.hasGravity,
        )

        bottom.tags += tag

        val top = Cuboid(
            world = sender.world,
            pos = Vector3d(origin).add(0.0, 1.0, 0.0),
            velocity = Vector3d(opts.v0),
            width = opts.dims.x,
            height = opts.dims.y,
            length = opts.dims.z,
            q = Quaterniond(q),
            omega = Vector3d(o),
            density = opts.density,
            hasGravity = opts.hasGravity,
        )

        top.tags += tag
        
        val ph = sender.world.physicsWorld ?: return false

        ph.registerBody(BlockEntityCuboid(bottom, Material.GLASS))
        ph.registerBody(BlockEntityCuboid(top, Material.GLASS))
        ph.sphericalJointConstraints.addConstraint(
            a = bottom,
            ra = Vector3d(0.5, 0.5, 0.5),
            b = top,
            rb = Vector3d(0.5, -0.5, 0.5),
        )

        return true
    }

    override fun onTabComplete(
        p0: CommandSender,
        p1: org.bukkit.command.Command,
        p2: String,
        p3: Array<out String>,
    ): List<String?>? {
        return emptyList()
    }
}