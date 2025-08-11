package com.ixume.udar.model

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.model.element.instance.ModelElement
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.body.active.Composite
import org.joml.Quaterniond
import org.joml.Vector3d

class JavaModel(
    val name: String,
    val elems: List<ModelElement>,
) {
    fun realize(pw: PhysicsWorld, origin: Vector3d): ActiveBody {
        val bs = elems.map {
            it.realize(pw, origin).apply {
                isChild = true
            }
        }

        return Composite(
            world = pw.world,
            velocity = Vector3d(),
            q = Quaterniond(),
            omega = Vector3d(0.0),
            hasGravity = true,
            parts = bs,
        )
    }
}