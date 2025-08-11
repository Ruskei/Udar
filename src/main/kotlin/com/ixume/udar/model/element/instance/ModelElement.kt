package com.ixume.udar.model.element.instance

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import org.joml.Vector3d

sealed interface ModelElement {
    fun realize(pw: PhysicsWorld, origin: Vector3d): ActiveBody
}