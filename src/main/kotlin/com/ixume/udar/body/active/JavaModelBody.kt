package com.ixume.udar.body.active

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.model.JavaModel
import org.joml.Vector3d

class JavaModelBody private constructor(
    val composite: CompositeImpl,
) : ActiveBody by composite, Composite by composite {
    companion object {
        fun construct(pw: PhysicsWorld, origin: Vector3d, model: JavaModel): JavaModelBody {
            val composite = model.realize(pw, origin)
            return JavaModelBody(composite)
        }
    }
}