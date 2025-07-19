package com.ixume.udar.body

import com.ixume.udar.applyIf
import com.ixume.udar.body.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.capability.Capability
import com.ixume.udar.collisiondetection.capability.GJKCapable
import com.ixume.udar.collisiondetection.capability.SDFCapable
import com.ixume.udar.collisiondetection.contactgeneration.GJKEPAContactGenerator
import com.ixume.udar.collisiondetection.contactgeneration.SATContactGenerator
import com.ixume.udar.collisiondetection.contactgeneration.SDFContactGenerator
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.IContact
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.World
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.entity.TextDisplay
import org.bukkit.util.BoundingBox
import org.bukkit.util.Transformation
import org.joml.Quaterniond
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
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

    override val edges: List<Pair<Vector3d, Vector3d>>
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
//
//            xMin += (velocity.x * TIME_STEP).coerceAtMost(0.0)
//            xMax += (velocity.x * TIME_STEP).coerceAtLeast(0.0)
//            yMin += (velocity.y * TIME_STEP).coerceAtMost(0.0)
//            yMax += (velocity.y * TIME_STEP).coerceAtLeast(0.0)
//            zMin += (velocity.z * TIME_STEP).coerceAtMost(0.0)
//            zMax += (velocity.z * TIME_STEP).coerceAtLeast(0.0)

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

    override var contacts: MutableList<IContact> = mutableListOf()
    override val previousContacts: List<Contact> = mutableListOf()

    private val contactGenerators = listOf(
        SATContactGenerator(this),
        GJKEPAContactGenerator(this),
        SDFContactGenerator(this),
    )

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
    override val inverseMass = 1.0 / (volume * density)
    private var torque = Vector3d()

    private val inertia: Vector3d = Vector3d(
        height * height + length * length,
        width * width + length * length,
        width * width + height * height,
    ).mul(density * volume / 12.0)

    override val inverseInertia: Vector3d = Vector3d(
        1.0 / (height * height + length * length),
        1.0 / (width * width + length * length),
        1.0 / (width * width + height * height),
    ).div(density * volume / 12.0)

    override var prevQ = Quaterniond(q)
    override fun step() {
        prevQ = Quaterniond(q)

        pos.add(Vector3d(velocity).mul(TIME_STEP))

        val h2 = TIME_STEP / 2.0

        val (dK1Q, dK1O) = calcDerivatives(q, omega)

        val k2Q = Quaterniond(q).add(Quaterniond(dK1Q).scale(h2)).normalize()
        val k2O = Vector3d(omega).add(Vector3d(dK1O).mul(h2))
        val (dK2Q, dK2O) = calcDerivatives(k2Q, k2O)

        val k3Q = Quaterniond(q).add(Quaterniond(dK2Q).scale(h2)).normalize()
        val k3O = Vector3d(omega).add(Vector3d(dK2O).mul(h2))
        val (dK3Q, dK3O) = calcDerivatives(k3Q, k3O)

        val k4Q = Quaterniond(q).add(Quaterniond(dK3Q).scale(TIME_STEP)).normalize()
        val k4O = Vector3d(omega).add(Vector3d(dK3O).mul(TIME_STEP))
        val (dK4Q, dK4O) = calcDerivatives(k4Q, k4O)

        val fDO = Vector3d(dK1O)
            .add(Vector3d(dK2O).mul(2.0))
            .add(Vector3d(dK3O).mul(2.0))
            .add(dK4O)
            .mul(TIME_STEP / 6.0)

        val fDQ = Quaterniond(dK1Q)
            .add(Quaterniond(dK2Q).scale(2.0))
            .add(Quaterniond(dK3Q).scale(2.0))
            .add(dK4Q)
            .mul(TIME_STEP / 6.0)

        omega.add(fDO)

        q.add(fDQ)
        q.normalize()

        torque = Vector3d()

        vertices = calcVertices()
        boundingBox = calcBoundingBox()

        display.transformation = createTransformation()
        display.teleport(Location(world, pos.x, pos.y, pos.z))

        handleDebug()
    }

    private var previousDebugLevel = 0
    private fun handleDebug() {
//        if (PhysicsCommand.DEBUG_LEVEL > 0 && previousDebugLevel == 0) {
//            debugDisplay = world.spawnEntity(display.location, EntityType.TEXT_DISPLAY) as TextDisplay
//            debugDisplay!!.text = "V: $velocity"
//            debugDisplay!!.billboard = Display.Billboard.CENTER
//        } else if (PhysicsCommand.DEBUG_LEVEL == 0 && previousDebugLevel > 0) {
//            debugDisplay?.remove()
//            debugDisplay = null
//        }
//
//        if (debugDisplay != null) {
//            debugDisplay!!.teleport(display.location)
//            debugDisplay!!.text = "V: $velocity"
//        }
//
//        previousDebugLevel = PhysicsCommand.DEBUG_LEVEL
    }

    private fun calcDerivatives(
        q: Quaterniond,
        o: Vector3d
    ): Pair<Quaterniond, Vector3d> {
        val dO = Vector3d(
            (inertia.y - inertia.z) / inertia.x * o.y * o.z + torque.x / inertia.x,
            (inertia.z - inertia.x) / inertia.y * o.z * o.x + torque.y / inertia.y,
            (inertia.x - inertia.y) / inertia.z * o.x * o.y + torque.z / inertia.z,
        )

        val dQ = Quaterniond(q).mul(Quaterniond(o.x, o.y, o.z, 0.0)).mul(0.5)

        return dQ to dO
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
        val localNormal = normal.rotate(Quaterniond(q).conjugate()).normalize()!!
        val localPoint = globalToLocal(point)

        val j = Vector3d(normal).dot(impulse) / (inverseMass + (Vector3d(localPoint).cross(localNormal).mul(
            inverseInertia
        ).cross(localPoint).dot(localNormal)))

        val effectiveImpulse = Vector3d(localNormal).mul(j)

        val t = Vector3d(localPoint).cross(effectiveImpulse)

        omega.add(Vector3d(inverseInertia).mul(t))

        val linear = Vector3d(normal).mul(j * inverseMass)
        velocity.add(linear)
    }

    override fun capableCollision(other: Body): Capability {
        return contactGenerators
            .map { it.capableCollision(other) }
            .filter { it.capable }
            .maxByOrNull { it.priority }
            ?: Capability(false, 0)
    }

    override fun collides(other: Body): List<IContact> {
        val (contactGenerator, _) =
            contactGenerators
                .zip(contactGenerators.map { it.capableCollision(other) })
                .filter { it.second.capable }
                .maxByOrNull { it.second.priority } ?: return emptyList()

        return contactGenerator.collides(other)
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

        return (if (distance(p) > 0.0) Vector3d(
            max(ed.x, 0.0) * sign(lp.x),
            max(ed.y, 0.0) * sign(lp.y),
            max(ed.z, 0.0) * sign(lp.z),
        ) else {
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
        }).rotate(q).apply { normalize(); if (!isFinite) set(0.0)
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

    override fun visualize() {
//        if (PhysicsCommand.DEBUG_MESH_LEVEL > 0) {
//            cachedMesh.visualize(world, visualizeFaces = false, visualizeEdges = true)
//        }
    }

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