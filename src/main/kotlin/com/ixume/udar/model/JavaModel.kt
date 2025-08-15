package com.ixume.udar.model

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.model.element.instance.ModelElement
import com.ixume.udar.body.active.CompositeImpl
import org.joml.Quaterniond
import org.joml.Vector3d

class JavaModel(
    val id: String,
    val elems: List<ModelElement>,
    val index: Int,
) {
    fun realize(pw: PhysicsWorld, origin: Vector3d): CompositeImpl {
        val bs = elems.map {
            it.realize(pw, origin).apply {
                isChild = true
            }
        }

        return CompositeImpl(
            world = pw.world,
            velocity = Vector3d(),
            q = Quaterniond(),
            omega = Vector3d(0.0),
            hasGravity = true,
            parts = bs,
            origin = origin,
        )
    }
}