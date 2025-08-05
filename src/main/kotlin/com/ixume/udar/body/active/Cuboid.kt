package com.ixume.udar.body.active

import com.ixume.udar.body.Body
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.body.active.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.capability.GJKCapable
import com.ixume.udar.collisiondetection.capability.SDFCapable
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentSATContactGenerator
import com.ixume.udar.collisiondetection.contactgeneration.SATContactGenerator
import com.ixume.udar.physics.IContact
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.World
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.entity.TextDisplay
import org.bukkit.util.BoundingBox
import org.bukkit.util.Transformation
import org.joml.*
import java.util.*
import kotlin.math.*

class Cuboid(
    override val world: World,
    override var pos: Vector3d,
    override var velocity: Vector3d,

    override val q: Quaterniond,

    override val omega: Vector3d,

    val width: Double,
    val height: Double,
    val length: Double,
    val density: Double,
    override val hasGravity: Boolean,
) : ActiveBody, GJKCapable, SDFCapable {
    override val id = UUID.randomUUID()!!

    val scale = Vector3d(width, height, length)

    override val radius: Double = Vector3d(width, height, length).mul(0.5).mul(scale).length()

    fun localToGlobal(vec: Vector3d): Vector3d {
        return Vector3d(vec).mul(scale).rotate(q).add(pos)
    }

    override fun globalToLocal(vec: Vector3d): Vector3d {
        return Vector3d(vec).sub(pos).rotate(Quaterniond(q).conjugate()).div(scale)
    }

    private val rawVertices: List<Vector3d> = listOf(
        Vector3d(-0.5, -0.5, -0.5),
        Vector3d(-0.5, -0.5, 0.5),
        Vector3d(0.5, -0.5, 0.5),
        Vector3d(0.5, -0.5, -0.5),

        Vector3d(-0.5, 0.5, -0.5),
        Vector3d(-0.5, 0.5, 0.5),
        Vector3d(0.5, 0.5, 0.5),
        Vector3d(0.5, 0.5, -0.5),
    )

    private fun calcVertices(): List<Vector3d> {
        return rawVertices.map { localToGlobal(it) }
    }

    override var vertices: List<Vector3d> = calcVertices()

    private val edges: List<Pair<Vector3d, Vector3d>>
        get() {
            return listOf(
                //bottom face
                vertices[0] to vertices[1],
                vertices[1] to vertices[2],
                vertices[2] to vertices[3],
                vertices[3] to vertices[0],
                //top face
                vertices[4] to vertices[5],
                vertices[5] to vertices[6],
                vertices[6] to vertices[7],
                vertices[7] to vertices[4],
                //connection
                vertices[0] to vertices[4],
                vertices[1] to vertices[5],
                vertices[2] to vertices[6],
                vertices[3] to vertices[7],
            )
        }

    private fun calcBoundingBox(): BoundingBox {
        var xMin = Double.MAX_VALUE
        var xMax = -Double.MAX_VALUE
        var yMin = Double.MAX_VALUE
        var yMax = -Double.MAX_VALUE
        var zMin = Double.MAX_VALUE
        var zMax = -Double.MAX_VALUE

        val myVertices = vertices

        for (vertex in myVertices) {
            xMin = min(xMin, vertex.x)
            xMax = max(xMax, vertex.x)
            yMin = min(yMin, vertex.y)
            yMax = max(yMax, vertex.y)
            zMin = min(zMin, vertex.z)
            zMax = max(zMax, vertex.z)
        }

        return BoundingBox(xMin, yMin, zMin, xMax, yMax, zMax)
    }

    override var boundingBox: BoundingBox = calcBoundingBox()

    private val display: BlockDisplay = world.spawnEntity(
        Location(world, pos.x, pos.y, pos.z),
        EntityType.BLOCK_DISPLAY
    ) as BlockDisplay

    private var debugDisplay: TextDisplay? = null

    private fun createTransformation(): Transformation {
        val scale = Vector3f(scale.x.toFloat(), scale.y.toFloat(), scale.z.toFloat())
        val rot = Quaternionf(q.x.toFloat(), q.y.toFloat(), q.z.toFloat(), q.w.toFloat())
        return Transformation(
            Vector3f(-0.5f).mul(scale).rotate(rot),
            rot,
            scale,
            Quaternionf(),
        )
    }

    override val isConvex: Boolean = true

    override val contacts: MutableList<IContact> = mutableListOf()
    override val previousContacts: MutableList<IContact> = mutableListOf()

    private val envContactGen = EnvironmentSATContactGenerator(this)
    private val SATContactGen = SATContactGenerator(this)

    init {
        val material = VALID_MATERIALS.random()

        display.block = material.createBlockData()

        display.transformation = createTransformation()
        display.interpolationDuration = 2
    }

    override fun kill() {
        display.remove()
        debugDisplay?.remove()
    }

    private val volume = width * height * length
    override val mass: Double = volume * density
    override val inverseMass = 1.0 / (volume * density)
    override var torque = Vector3d()

    private fun calcInertia(): Vector3d {
        val f = density * volume / 12.0
        return Vector3d(
            (height * height + length * length) * f,
            (width * width + length * length) * f,
            (width * width + height * height) * f,
        )
    }

    override val localInertia: Vector3d = calcInertia()
    val localInverseInertia: Vector3d = Vector3d(1.0 / localInertia.x, 1.0 / localInertia.y, 1.0 / localInertia.z)

    private fun calcInverseInertia(): Matrix3d {
        val rm = Matrix3d().rotation(q)
        return Matrix3d(rm).transpose().mul(
            Matrix3d(
                1.0 / localInertia.x, 0.0, 0.0,
                0.0, 1.0 / localInertia.y, 0.0,
                0.0, 0.0, 1.0 / localInertia.z,
            )
        ).mul(rm)
    }

    override val inverseInertia: Matrix3d = calcInverseInertia()

    private val rotationIntegrator = RigidbodyRotationIntegrator(this)

    override var prevQ = Quaterniond(q)
    override fun step() {
        prevQ = Quaterniond(q)

        pos.add(Vector3d(velocity).mul(TIME_STEP))
        rotationIntegrator.process()

        torque = Vector3d()

        update()
        visualize()

        localInertia.set(calcInertia())
        localInverseInertia.set(1.0 / localInertia.x, 1.0 / localInertia.y, 1.0 / localInertia.z)
        inverseInertia.set(calcInverseInertia())
    }

    override fun update() {
        previousContacts.clear()
        previousContacts += contacts
        contacts.clear()

        vertices = calcVertices()
        boundingBox = calcBoundingBox()
    }

    override fun visualize() {
        display.transformation = createTransformation()
        display.teleport(Location(world, pos.x, pos.y, pos.z))
    }

    override fun intersect(origin: Vector3d, end: Vector3d): List<Pair<Vector3d, Vector3d>> {
        // face point to face normal
        // 0 : x
        // 1 : y
        // 2 : z
        val normals = listOf(
            Triple(Vector3d(-0.5), Vector3d(0.0, -1.0, 0.0), 1),
            Triple(Vector3d(-0.5, 0.5, -0.5), Vector3d(0.0, 1.0, 0.0), 1),
            Triple(Vector3d(0.5, -0.5, -0.5), Vector3d(1.0, 0.0, 0.0), 0),
            Triple(Vector3d(-0.5), Vector3d(-1.0, 0.0, 0.0), 0),
            Triple(Vector3d(-0.5, -0.5, 0.5), Vector3d(0.0, 0.0, 1.0), 2),
            Triple(Vector3d(-0.5), Vector3d(0.0, 0.0, -1.0), 2),
        )

        val localOrigin = globalToLocal(origin)
        val localEnd = globalToLocal(end)

        val nDir = Vector3d(localEnd).sub(localOrigin).normalize()

        val intersections = mutableListOf<Pair<Vector3d, Vector3d>>() // global space

        for ((p, n, axis) in normals) {
            val den = nDir.dot(n)

            if (den == 0.0) continue //parallel

            val d = (Vector3d(p).sub(localOrigin).dot(n)) / den
            if (d == 0.0) continue //TODO: line contained

            val localIntersection = localOrigin.add(Vector3d(nDir).mul(d))

            //check if actually inside face
            if (axis == 0) {
                //x axis
                if (localIntersection.y !in -0.5..0.5 || localIntersection.z !in -0.5..0.5) continue
            } else if (axis == 1) {
                //y axis
                if (localIntersection.x !in -0.5..0.5 || localIntersection.z !in -0.5..0.5) continue
            } else {
                //z axis
                if (localIntersection.x !in -0.5..0.5 || localIntersection.y !in -0.5..0.5) continue
            }

            intersections += localToGlobal(localIntersection) to Vector3d(n).rotate(q)
        }

        return intersections
    }

    /**
     * Has a bug with application of direction of impulse
     * @param point Point on body in world space
     * @param normal Normal of the point at which the impulse is being applied
     */
    override fun applyImpulse(
        point: Vector3d,
        normal: Vector3d,
        impulse: Vector3d,
    ) {
        val localNormal = Vector3d(normal).rotate(Quaterniond(q).conjugate()).normalize()!!
        val localPoint = globalToLocal(point)

        val angularMass = Vector3d(localPoint)
            .cross(localNormal)
            .mul(localInverseInertia)
            .cross(localPoint)
            .dot(localNormal)

        val j = Vector3d(normal).dot(impulse) / (inverseMass + angularMass)

        val effectiveImpulse = Vector3d(localNormal).mul(j)

        val t = Vector3d(localPoint).cross(effectiveImpulse)

        omega.add(Vector3d(localInverseInertia).mul(t))

        val linear = Vector3d(normal).mul(j * inverseMass)
        velocity.add(linear)
    }

    override fun capableCollision(other: Body): Capability {
        return when (other) {
            is EnvironmentBody -> {
                envContactGen.capableCollision(other)
            }

            is Cuboid -> {
                SATContactGen.capableCollision(other)
            }

            else -> {
                Capability(false, 0)
            }
        }
    }

    override fun collides(other: Body): List<IContact> {
        return when (other) {
            is EnvironmentBody -> {
                envContactGen.collides(other)
            }

            is Cuboid -> {
                SATContactGen.collides(other)
            }

            else -> {
                emptyList()
            }
        }
    }

    override fun support(dir: Vector3d): Vector3d {
        val vertices = vertices
        var maxDot = -Double.MAX_VALUE
        var maxVertex = vertices[0]

        for (vertex in vertices) {
            val dot = vertex.dot(dir)

            if (dot > maxDot) {
                maxDot = dot
                maxVertex = vertex
            }
        }

        return maxVertex
    }

    override fun distance(p: Vector3d): Double {
        val ed = globalToLocal(p).let {
            Vector3d(
                abs(it.x),
                abs(it.y),
                abs(it.z),
            ).mul(scale)
        }.sub(width * 0.5, height * 0.5, length * 0.5)
        return Vector3d(ed).max(Vector3d(0.0, 0.0, 0.0)).length() + min(max(max(ed.x, ed.y), ed.z), 0.0)
    }

    override fun gradient(p: Vector3d): Vector3d {
        val lp = globalToLocal(p).mul(scale)
        val ed = Vector3d(lp).let {
            Vector3d(
                abs(it.x),
                abs(it.y),
                abs(it.z),
            )
        }.sub(width * 0.5, height * 0.5, length * 0.5)

        val d = distance(p)
        check(!d.isNaN())

        val l = (if (d > 0.0) {
            Vector3d(
                max(ed.x, 0.0) * sign(lp.x),
                max(ed.y, 0.0) * sign(lp.y),
                max(ed.z, 0.0) * sign(lp.z),
            )
        } else if (d == 0.0) {
            Vector3d(
                if (ed.x == 0.0) sign(lp.x) else 0.0,
                if (ed.y == 0.0) sign(lp.y) else 0.0,
                if (ed.z == 0.0) sign(lp.z) else 0.0,
            )
        } else {
            if (ed.x.absoluteValue < ed.y.absoluteValue) {
                //x < y
                if (ed.z.absoluteValue < ed.x.absoluteValue) {
                    //z < x < y
                    Vector3d(
                        0.0,
                        0.0,
                        ed.z.absoluteValue * sign(lp.z),
                    )
                } else {
                    //x < y, z
                    Vector3d(
                        ed.x.absoluteValue * sign(lp.x),
                        0.0,
                        0.0,
                    )
                }
            } else {
                //y < x
                if (ed.z.absoluteValue < ed.y.absoluteValue) {
                    //z < y < x
                    Vector3d(
                        0.0,
                        0.0,
                        ed.z.absoluteValue * sign(lp.z),
                    )
                } else {
                    //y < x, z
                    Vector3d(
                        0.0,
                        ed.y.absoluteValue * sign(lp.y),
                        0.0,
                    )
                }
            }
        })

        return l.rotate(q).apply {
            normalize();
            if (!isFinite)
                set(0.0)
        }
    }

    override val startPoints: List<Vector3d>
        get() {
            return vertices.map { Vector3d(it) }
        }

    override fun ensureNonAligned() {
        val tiny = 1e-14
        val perturbation = 1e-10
        val myAxiss = listOf(
            Vector3d(1.0, 0.0, 0.0).rotate(q).normalize(),
            Vector3d(0.0, 1.0, 0.0).rotate(q).normalize(),
            Vector3d(0.0, 0.0, 1.0).rotate(q).normalize(),
        )

        if (myAxiss.any {
                it.distance(1.0, 0.0, 0.0) < tiny || it.distance(-1.0, 0.0, 0.0) < tiny ||
                        it.distance(0.0, 1.0, 0.0) < tiny || it.distance(0.0, -1.0, 0.0) < tiny ||
                        it.distance(0.0, 0.0, 1.0) < tiny || it.distance(0.0, 0.0, -1.0) < tiny
            }) {

            q.rotateXYZ(perturbation, perturbation, perturbation)
            ensureNonAligned()
        }
    }

//    private val contactGenerator: ContactGenerator = SATContactGenerator(this)


    companion object {
        private val VALID_MATERIALS = listOf(
            Material.GLASS,
        )
    }
}

private const val EPSILON = 1e-11
private const val FUDGE = 4.0
fun Vector3d.toStringFull(): String {
    return "( $x, $y, $z )"
}