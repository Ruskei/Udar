package com.ixume.udar.body.active

import com.ixume.udar.body.Body
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.jacobiEigenDecomposition
import com.ixume.udar.physics.IContact
import org.bukkit.World
import org.bukkit.util.BoundingBox
import org.joml.Matrix3d
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.*
import kotlin.math.max
import kotlin.math.min

class Composite(
    override val world: World,
    override var velocity: Vector3d,

    override val q: Quaterniond,

    override val omega: Vector3d,

    override val hasGravity: Boolean,
    val parts: List<ActiveBody>
) : ActiveBody {
    init {
        require(parts.isNotEmpty())
    }

    override val id: UUID = UUID.randomUUID()

    data class RelativePose(
        val pos: Vector3d,
        val rot: Quaterniond,
    )

    private fun calcCOM(): Vector3d {
        val p = Vector3d()
        var massSum = 0.0
        for (part in parts) {
            p.add(Vector3d(part.pos).mul(part.mass))
            massSum += part.mass
        }

        p.div(massSum)

        return p
    }

    override var pos: Vector3d = calcCOM()

    private val relativePoses: Map<UUID, RelativePose> = parts.associateBy(
        keySelector = { it.id },
        valueTransform = { RelativePose(Vector3d(it.pos).sub(pos), Quaterniond(it.q)) }
    )

    private fun calcVertices(): List<Vector3d> {
        return parts.flatMap { it.vertices }
    }

    override var vertices: List<Vector3d> = calcVertices()

    private fun calcBoundingBox(): BoundingBox {
        val bbs = parts.map { it.boundingBox }

        var xMin = Double.MAX_VALUE
        var xMax = -Double.MAX_VALUE
        var yMin = Double.MAX_VALUE
        var yMax = -Double.MAX_VALUE
        var zMin = Double.MAX_VALUE
        var zMax = -Double.MAX_VALUE

        for (bb in bbs) {
            xMin = min(xMin, bb.minX)
            xMax = max(xMax, bb.maxX)
            yMin = min(yMin, bb.minY)
            yMax = max(yMax, bb.maxY)
            zMin = min(zMin, bb.minZ)
            zMax = max(zMax, bb.maxZ)
        }

        return BoundingBox(xMin, yMin, zMin, xMax, yMax, zMax)
    }

    override var boundingBox: BoundingBox = calcBoundingBox()

    override val prevQ: Quaterniond = Quaterniond(q)

    override fun globalToLocal(vec: Vector3d): Vector3d {
        return Vector3d(vec).sub(pos).rotate(Quaterniond(q).conjugate())
    }

    private val rotationIntegrator = RigidbodyRotationIntegrator(this)

    override fun step() {
        prevQ.set(q)

        pos.add(Vector3d(velocity).mul(TIME_STEP))
        rotationIntegrator.process()

        for (part in parts) {
            val pose = relativePoses[part.id]!!
            part.pos.set(Vector3d(pos).add(Vector3d(pose.pos).rotate(q)))
            part.q.set(Quaterniond(q).mul(pose.rot))

            part.visualize()
        }

        inverseInertia.set(calcInverseInertia())

        vertices = calcVertices()
        boundingBox = calcBoundingBox()
    }

    override fun intersect(
        origin: Vector3d,
        end: Vector3d
    ): List<Pair<Vector3d, Vector3d>> {
        return parts.flatMap { it.intersect(origin, end) }
    }

    override fun kill() {
        parts.forEach { it.kill() }
    }

    override fun applyImpulse(point: Vector3d, normal: Vector3d, impulse: Vector3d) {}

    override val contacts: MutableList<IContact> = mutableListOf()
    override val previousContacts: List<IContact> = mutableListOf()

    override val torque: Vector3d = Vector3d()

    private fun totalMass(): Double {
        return parts.fold(0.0) { sum, body -> sum + body.mass }
    }

    data class InertialData(
        val localInertia: Vector3d,
        val inverseInertia: Matrix3d
    )

    private fun calcInitialInertialData(): InertialData {
        val matrix = Matrix3d(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        )

        for (part in parts) {
            val lI = part.localInertia
            val rm = Matrix3d().rotation(part.q)
            val gI = Matrix3d(rm).mul(
                Matrix3d(
                    lI.x, 0.0, 0.0,
                    0.0, lI.y, 0.0,
                    0.0, 0.0, lI.z,
                )
            ).mul(rm.transpose())

            val r = Vector3d(part.pos).sub(pos).rotate(Quaterniond(q).conjugate())
            val r2 = r.lengthSquared()
            //parallel axis theorem
            val addon = Matrix3d(
                r2 - r.x * r.x, -r.y * r.x, -r.z * r.x,
                -r.x * r.y, r2 - r.y * r.y, -r.z * r.y,
                -r.x * r.z, -r.y * r.z, r2 - r.z * r.z,
            ).scale(part.mass)

            matrix.add(gI).add(addon)
        }

        val (ev, _) = jacobiEigenDecomposition(Matrix3d(matrix))
        val iI = (matrix).invert()

        return InertialData(ev, iI)
    }

    private fun calcInverseInertia(): Matrix3d {
        return Matrix3d(localInverseInertia).rotate(q)
    }

    override val mass: Double = totalMass()
    override val inverseMass: Double = 1.0 / mass
    override val localInertia: Vector3d
    override val inverseInertia: Matrix3d
    private val localInverseInertia: Matrix3d

    init {
        val (lI, iI) = calcInitialInertialData()
        localInertia = lI
        localInverseInertia = iI

        inverseInertia = calcInverseInertia()
    }

    override val isConvex: Boolean = false

    override fun capableCollision(other: Body): Capability {
        return Capability()
    }

    override fun collides(other: Body): List<IContact> {
        return emptyList()
    }
}