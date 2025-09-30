package com.ixume.udar.body.active

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.model.JavaModel
import com.ixume.udar.rp.RPManager
import org.bukkit.Location
import org.bukkit.entity.EntityType
import org.bukkit.entity.ItemDisplay
import org.bukkit.util.Transformation
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f

class JavaModelBody private constructor(
    val composite: CompositeImpl,
    model: JavaModel,
) : Composite by composite {
    private val display: ItemDisplay

    init {
        val p = Vector3d(composite.pos).sub(Vector3d(composite.comOffset)).rotate(composite.q)
        display = world.spawnEntity(
            Location(world, p.x, p.y, p.z),
            EntityType.ITEM_DISPLAY
        ) as ItemDisplay

        display.setItemStack(RPManager.item(model.id))

        display.transformation = createTransformation()
        display.interpolationDuration = 3
        display.interpolationDelay = 0
        display.teleportDuration = 2
    }

    private fun createTransformation(): Transformation {
//        val rot = Quaternionf(q.x.toFloat(), q.y.toFloat(), q.z.toFloat(), q.w.toFloat())

        /*
        process:
        - multiply by pi around y axis
	    - matrices.translate(-0.5F, -0.5F, -0.5F);
         */

        val rot = Quaternionf(composite.q).rotateY(Math.PI.toFloat())
        return Transformation(
            Vector3f(-0.5f, 0.5f, -0.5f).rotate(rot),
            rot,
            Vector3f(1f),
            Quaternionf(),
        )
    }

    override fun visualize() {
        val pos2 = Vector3d(composite.pos).sub(Vector3d(composite.comOffset).rotate(composite.q))
        display.interpolationDuration = 3
        display.interpolationDelay = 0
        display.teleportDuration = 2
        display.transformation = createTransformation()
        display.teleport(Location(world, pos2.x, pos2.y, pos2.z))

//        composite.visualize()
    }

    override fun onKill() {
        composite.onKill()
        display.remove()
    }

    companion object {
        fun construct(pw: PhysicsWorld, origin: Vector3d, model: JavaModel): JavaModelBody {
            val composite = model.realize(pw, origin)
            return JavaModelBody(composite, model)
        }
    }
}