package com.ixume.udar.body.active

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.Body
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.contactgeneration.CompositeCompositeContactGenerator
import com.ixume.udar.jacobiEigenDecomposition
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.IContact
import com.ixume.udar.physicsWorld
import org.bukkit.World
import org.bukkit.util.BoundingBox
import org.joml.Matrix3d
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.*
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.acos
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
    override val physicsWorld: PhysicsWorld = world.physicsWorld!!

    override var age: Int = 0
    override var awake = AtomicBoolean(true)
    override var startled = AtomicBoolean(true)

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

    private fun calcRadius(): Double {
        var r = -Double.MAX_VALUE
        for (part in parts) {
            val candidate = part.pos.distance(pos) + part.radius
            r = max(candidate, r)
        }

        return r
    }

    override val radius: Double = calcRadius()

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
    private val prevP = Vector3d(pos)
    override val linearDelta: Vector3d = Vector3d()
    override var angularDelta: Double = 0.0

    override fun globalToLocal(vec: Vector3d): Vector3d {
        return Vector3d(vec).sub(pos).rotate(Quaterniond(q).conjugate())
    }

    private val rotationIntegrator = RigidbodyRotationIntegrator(this)

    override fun step() {
        prevQ.set(q)
        prevP.set(pos)

        pos.add(Vector3d(velocity).mul(TIME_STEP))
        rotationIntegrator.process()

        for (part in parts) {
            val pose = relativePoses[part.id]!!
            val np = Vector3d(pos).add(Vector3d(pose.pos).rotate(q))
            val v = Vector3d(np).sub(part.pos).div(TIME_STEP)
            part.pos.set(np)
            part.velocity.set(v)
            part.q.set(Quaterniond(q).mul(pose.rot))

            part.update()
        }

        linearDelta.set(pos).sub(prevP)

        val dQ = Quaterniond(q).mul(Quaterniond(q).conjugate())
        if (dQ.w < 0.0) dQ.mul(-1.0)
        dQ.normalize()
        angularDelta = 2.0 * acos(dQ.w.coerceIn(-1.0, 1.0))

        inverseInertia.set(calcInverseInertia())

        previousContacts.clear()
        previousContacts += contacts
        contacts.clear()

        vertices = calcVertices()
        boundingBox = calcBoundingBox()
    }

    override fun visualize() {
        parts.forEach { it.visualize() }
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

    override fun applyImpulse(point: Vector3d, normal: Vector3d, impulse: Vector3d) {
        awake.set(true)
        startled.set(true)
        val localNormal =
            Vector3d(normal).rotate(Quaterniond(q).conjugate()).normalize()!!.mul(inertialPrincipleRotation)
        val localPoint = globalToLocal(point).mul(inertialPrincipleRotation)

        val angularMass = Vector3d(localPoint)
            .cross(localNormal)
            .mul(localInverseInertia)
            .cross(localPoint)
            .dot(localNormal)

        val j = Vector3d(normal).dot(impulse) / (inverseMass + angularMass)

        val effectiveImpulse = Vector3d(localNormal).mul(j)

        val t = Vector3d(localPoint).cross(effectiveImpulse)

        omega.add(Vector3d(1.0 / localInertia.x, 1.0 / localInertia.y, 1.0 / localInertia.z).mul(t))

        val linear = Vector3d(normal).mul(j * inverseMass)
        velocity.add(linear)
    }

    override val contacts: MutableList<IContact> = mutableListOf()
    override val previousContacts: MutableList<IContact> = mutableListOf()

    override val torque: Vector3d = Vector3d()

    private fun totalMass(): Double {
        return parts.fold(0.0) { sum, body -> sum + body.mass }
    }

    data class InertialData(
        val localInertia: Vector3d,
        val inverseInertia: Matrix3d,
        val principleRotation: Matrix3d
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

        val (ev, eV) = jacobiEigenDecomposition(Matrix3d(matrix))
        val iI = matrix.invert()

        return InertialData(ev, iI, eV)
    }

    private fun calcInverseInertia(): Matrix3d {
        val rm = Matrix3d().rotation(q)
        return Matrix3d(rm).transpose().mul(Matrix3d(localInverseInertia)).mul(rm)
    }

    override val mass: Double = totalMass()
    override val inverseMass: Double = 1.0 / mass
    override val localInertia: Vector3d
    override val inverseInertia: Matrix3d
    val inertialPrincipleRotation: Matrix3d
    val localInverseInertia: Matrix3d

    init {
        val (lI, iI, eV) = calcInitialInertialData()
        localInertia = lI
        localInverseInertia = iI
        inertialPrincipleRotation = eV.transpose()

        inverseInertia = calcInverseInertia()
    }

    override val isConvex: Boolean = false

    private val compositeContactGenerator = CompositeCompositeContactGenerator(this)

    override fun capableCollision(other: Body): Capability {
        if (other is Composite) {
            return Capability(true, 0)
        }

        var highestPriority: Int? = null
        for (part in parts) {
            val (capable, priority) = part.capableCollision(other)
            if (!capable) return Capability(false, 0)

            highestPriority = if (highestPriority == null) {
                priority
            } else {
                max(highestPriority, priority)
            }
        }

        if (highestPriority == null) return Capability(false, 0)

        return Capability(true, highestPriority)
    }

    override fun collides(other: Body): List<IContact> {
        if (other is Composite) {
            return compositeContactGenerator.collides(other)
                .map {
                    Contact(
                        first = this,
                        second = other,
                        result = it.result,
                    )
                }
        } else if (other is ActiveBody) {
            val cs = mutableListOf<Contact>()
            for (part in parts) {
                if (!part.boundingBox.overlaps(other.boundingBox)) continue
                val d = part.pos.distance(other.pos)
                if (d > part.radius + other.radius) continue

                val r = part.collides(other)
                cs += r.map { Contact(
                    first = this,
                    second = other,
                    result = it.result,
                ) }
            }

            return cs
        } else {
            return parts
                .flatMap {
                    it.collides(other) }
                .map {
                    Contact(
                        first = this,
                        second = other,
                        result = it.result,
                    )
                }
        }
    }
}