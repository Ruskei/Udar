package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.JavaModelBody
import com.ixume.udar.physicsWorld
import com.ixume.udar.rp.RPManager
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaterniond

object ModelCommand : Command {
    override val arg: String = "model"
    override val description: String = "Spawn a model"

    override fun onTabComplete(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<String>
    ): List<String> {
        return RPManager.modelMap.keys.toList()
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<String>
    ): Boolean {
        if (sender !is Player) return true

        if (args.isEmpty()) return true

        val modelName = args[0]
        val model = RPManager.modelMap[modelName] ?: return true

        val physicsWorld = sender.world.physicsWorld ?: return true
        val origin = sender.location.toVector().toVector3d()

        val opts = if (args.size < 2) BodyOptions.default() else BodyOptions.fromArgs(args.copyOfRange(1, args.size))

        val q = Quaterniond().rotateXYZ(opts.rot0.x, opts.rot0.y, opts.rot0.z)
        val o = if (opts.trueOmega) opts.l.rotate(Quaterniond(q).conjugate()) else opts.l

        val output = JavaModelBody.construct(physicsWorld, origin, model)

        output.omega.set(o)
        output.q.set(q)
        output.hasGravity = opts.hasGravity
        output.velocity.set(opts.v0)

        physicsWorld.registerBody(output)

        return true
    }
}