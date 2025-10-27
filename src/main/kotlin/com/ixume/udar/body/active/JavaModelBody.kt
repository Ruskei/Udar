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
    val scale: Double,
) : Composite by composite {
    private val display: ItemDisplay

    private val _v = Vector3f()
    private val _s = Vector3f(scale.toFloat())
    private val _r = Quaternionf()
    private val _eq = Quaternionf()

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
        display.teleportDuration = 3
    }

    private fun createTransformation(): Transformation {
        val rot = _r.set(composite.q).rotateY(Math.PI.toFloat())
        return Transformation(
            _v,
            rot,
            _s,
            _eq,
        )
    }

    @JvmField
    val modelPos = Vector3d()

    private val _rp = Vector3d()

    override fun visualize() {
        if (!awake.get()) return

        val pos2 = modelPos.set(composite.pos).sub(_rp.set(composite.comOffset).rotate(composite.q))
        display.interpolationDuration = 3
        display.interpolationDelay = 0
        display.teleportDuration = 3
        display.transformation = createTransformation()
        display.teleport(Location(world, pos2.x, pos2.y, pos2.z))
//        composite.visualize()
    }

    override fun onKill() {
        composite.onKill()
        display.remove()
    }

    companion object {
        fun construct(
            pw: PhysicsWorld,
            origin: Vector3d,
            model: JavaModel,
            scale: Double,
        ): JavaModelBody {
            val composite = model.realize(pw, origin, scale)
            return JavaModelBody(composite, model, scale)
        }
    }
}