package com.ixume.udar.testing.commands

import com.ixume.udar.body.active.Composite
import com.ixume.udar.body.active.Cuboid
import com.ixume.udar.physicsWorld
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
        args: Array<out String>?
    ): List<String> {
        return emptyList()
    }

    override fun onCommand(
        sender: CommandSender,
        command: org.bukkit.command.Command,
        label: String,
        args: Array<out String>?
    ): Boolean {
        if (sender !is Player) return true

        val physicsWorld = sender.world.physicsWorld ?: return true
        val origin = sender.location.toVector().toVector3d()

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
        )

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
        )

        val composite = Composite(
            world = sender.world,
            velocity = Vector3d(),
            q = Quaterniond().rotateXYZ(
                Math.PI * 0.5,
                0.0,
                0.0
            ),
            omega = Vector3d(0.0000001, 0.0000001, 10.0),
            hasGravity = false,
            parts = listOf(cuboid, cuboid2)
        )

        physicsWorld.activeBodies += composite

        return true
    }
}