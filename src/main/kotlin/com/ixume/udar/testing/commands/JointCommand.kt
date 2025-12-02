package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.BlockEntityCuboid
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.body.active.tag.Tag
import com.ixume.udar.physics.hinge.HingeConstraint
import com.ixume.udar.physics.position.PointConstraint
import com.ixume.udar.physicsWorld
import org.bukkit.Material
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaterniond
import org.joml.Vector3d
import kotlin.math.PI
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
        val spinner = Cuboid(
            world = sender.world,
            pos = Vector3d(origin).add(1.0, 0.0, 0.0),
            velocity = Vector3d(opts.v0),
            width = 1.0,
            height = 0.5,
            length = 0.5,
            q = Quaterniond(q),
            omega = Vector3d(0.0, 0.0, 0.0),
            density = 1.0,
            hasGravity = true,
        )

        spinner.tags += tag

        val holder = Cuboid(
            world = sender.world,
            pos = Vector3d(origin),
            velocity = Vector3d(opts.v0),
            width = 1.0,
            height = 1.0,
            length = 1.0,
            q = Quaterniond(q),
            omega = Vector3d(o),
            density = 1_000.0,
            hasGravity = false,
        )

        holder.tags += tag

        val ph = sender.world.physicsWorld ?: return false

        val b1 = BlockEntityCuboid(spinner, Material.GLASS)
        val b2 = BlockEntityCuboid(holder, Material.GLASS)

        ph.registerBody(b1)
        ph.registerBody(b2)
        ph.constraintManager.constrain(
            PointConstraint(
                b1 = b1,
                b2 = b2,

                r1x = -0.25f,
                r1y = 0f,
                r1z = 0f,

                r2x = 0.75f,
                r2y = 0f,
                r2z = 0f
            )
        )
//        ph.constraintManager.constrain(
//            HingeConstraint(
//                b1 = b1,
//                b2 = b2,
//
//                a1x = 0f,
//                a1y = 0f,
//                a1z = 1f,
//
//                a2x = 0f,
//                a2y = 0f,
//                a2z = 1f,
//
//                n1x = 0f,
//                n1y = 1f,
//                n1z = 0f,
//
//                n2x = 0f,
//                n2y = 1f,
//                n2z = 0f,
//
//                min = -45f * PI.toFloat() / 180f,
//                max = 45f * PI.toFloat() / 180f,
//
//                p1x = -0.25f,
//                p1y = 0f,
//                p1z = 0f,
//
//                p2x = 0.75f,
//                p2y = 0f,
//                p2z = 0f,
//            )
//        )
//        ph.sphericalJointConstraints.addConstraint(
//            a = holder,
//            ra = Vector3d(0.75, 0.0, 0.0),
//            b = spinner,
//            rb = Vector3d(-0.25, 0.0, 0.0),
//        )
//        ph.angularConstraints.addConstraint(
//            a = holder,
//            bodyAAxis = Vector3f(0f, 0f, 1f),
//            b = spinner,
//            bodyBAxis = Vector3f(0f, 0f, 1f),
//        )
//        ph.angularConstraints.addConstraint(
//            a = holder,
//            b = spinner,
//            jA = Vector3f(0f, 0f, 1f),
//            jB = Vector3f(0f, 0f, 1f),
//            gA = Vector3f(0f, -1f, 0f),
//            gB = Vector3f(0f, -1f, 0f),
//            minAngle = Math.toRadians(-30.0).toFloat(),
//            maxAngle = Math.toRadians(30.0).toFloat(),
//        )
//        ph.angularConstraints.addConstraint(
//            a = holder,
//            b = spinner,
//            jA = Vector3f(1.0f, 0.0f, 0.0f), jB = Vector3f(1.0f, 0.0f, 0.0f),
//            gA = Vector3f(0.0f, 0.0f, 1.0f),
//            gB = Vector3f(0.0f, 0.0f, 1.0f),
//            swingAngle = Math.toRadians(30.0).toFloat(),
//            minTwistAngle = -Math.toRadians(45.0).toFloat(),
//            maxTwistAngle = Math.toRadians(45.0).toFloat(),
//        )

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