package com.ixume.udar.body.active

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.body.EnvironmentBody
import com.ixume.udar.collisiondetection.capability.GJKCapable
import com.ixume.udar.collisiondetection.capability.SDFCapable
import com.ixume.udar.collisiondetection.contactgeneration.CuboidSATContactGenerator
import com.ixume.udar.collisiondetection.contactgeneration.EnvironmentContactGenerator2
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.dynamicaabb.AABB
import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldCollection
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldCollection
import com.ixume.udar.physicsWorld
import org.bukkit.Material
import org.bukkit.World
import org.bukkit.entity.TextDisplay
import org.joml.Matrix3d
import org.joml.Quaterniond
import org.joml.Vector3d
import java.util.*
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.*

class Cuboid(
    override val world: World,
    override val pos: Vector3d,
    override val velocity: Vector3d,

    override val q: Quaterniond,

    override val omega: Vector3d,

    val width: Double,
    val height: Double,
    val length: Double,
    val density: Double,
    override var hasGravity: Boolean,
) : ActiveBody, GJKCapable, SDFCapable {
    override val uuid = UUID.randomUUID()!!
    override var idx: Int = -1
    override val physicsWorld: PhysicsWorld = world.physicsWorld!!
    override val id: Long = physicsWorld.createID()

    override var isChild: Boolean = false
    override var age: Int = 0
    override var awake = AtomicBoolean(true)
    override var startled = AtomicBoolean(true)
    override var idleTime: Int = 0

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

    override val vertices: Array<Vector3d> = Array(8) { Vector3d() }
    override val faces: Array<Face> = arrayOf(
        // wound counterclockwise
        Face(vertices = arrayOf(vertices[0], vertices[3], vertices[7], vertices[4])),
        Face(vertices = arrayOf(vertices[4], vertices[5], vertices[1], vertices[0])),
        Face(vertices = arrayOf(vertices[2], vertices[1], vertices[5], vertices[6])),
        Face(vertices = arrayOf(vertices[3], vertices[2], vertices[6], vertices[7])),
        Face(vertices = arrayOf(vertices[0], vertices[1], vertices[2], vertices[3])),
        Face(vertices = arrayOf(vertices[4], vertices[7], vertices[6], vertices[5])),
    )

    override val edges: Array<Edge> = arrayOf(
        Edge(vertices[0], vertices[1]),
        Edge(vertices[1], vertices[2]),
        Edge(vertices[2], vertices[3]),
        Edge(vertices[3], vertices[0]),

        Edge(vertices[4], vertices[5]),
        Edge(vertices[5], vertices[6]),
        Edge(vertices[6], vertices[7]),
        Edge(vertices[7], vertices[4]),

        Edge(vertices[0], vertices[4]),
        Edge(vertices[1], vertices[5]),
        Edge(vertices[2], vertices[6]),
        Edge(vertices[3], vertices[7]),
    )

    private fun updateVertices() {
        var i = 0
        while (i < vertices.size) {
            val v = vertices[i]
            v.set(rawVertices[i]).mul(scale).rotate(q).add(pos)
            ++i
        }

        var j = 0
        while (j < faces.size) {
            faces[j].updateNormal()
            j++
        }
    }

    private fun updateBB() {
        tightBB.minX = Double.MAX_VALUE
        tightBB.maxX = -Double.MAX_VALUE
        tightBB.minY = Double.MAX_VALUE
        tightBB.maxY = -Double.MAX_VALUE
        tightBB.minZ = Double.MAX_VALUE
        tightBB.maxZ = -Double.MAX_VALUE

        for (vertex in vertices) {
            tightBB.minX = min(tightBB.minX, vertex.x)
            tightBB.maxX = max(tightBB.maxX, vertex.x)
            tightBB.minY = min(tightBB.minY, vertex.y)
            tightBB.maxY = max(tightBB.maxY, vertex.y)
            tightBB.minZ = min(tightBB.minZ, vertex.z)
            tightBB.maxZ = max(tightBB.maxZ, vertex.z)
        }

        if (!isChild) {
            val fits = physicsWorld.bodyAABBTree.contains(
                fatBB,
                tightBB.minX,
                tightBB.minY,
                tightBB.minZ,
                tightBB.maxX,
                tightBB.maxY,
                tightBB.maxZ,
            )

            if (!fits) {
                physicsWorld.updateBB(this, tightBB)
            }
        }
    }

    override val tightBB: AABB = AABB(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    override var fatBB = -1

    private var debugDisplay: TextDisplay? = null

    override val isConvex: Boolean = true

    private val envContactGen = EnvironmentContactGenerator2(this)
    private val cuboidContactGen = CuboidSATContactGenerator(this)

    private val _rm = Matrix3d()
    private val _rmT = Matrix3d()

    private fun updateII() {
        _rm.rotation(q)
        _rmT.set(_rm).transpose()
        inverseInertia.set(
            _rmT.mul(
                inverseInertia.set(
                    1.0 / localInertia.x, 0.0, 0.0,
                    0.0, 1.0 / localInertia.y, 0.0,
                    0.0, 0.0, 1.0 / localInertia.z,
                )
            ).mul(_rm)
        )
    }

    private val volume = width * height * length
    override val mass: Double = volume * density
    override val inverseMass = 1.0 / (volume * density)
    override val torque = Vector3d()

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


    override val inverseInertia: Matrix3d = Matrix3d()

    init {
        updateVertices()
        updateII()
    }

    override fun onKill() {
        debugDisplay?.remove()
    }

    private val rotationIntegrator = RigidbodyRotationIntegrator(this)

    override val prevQ = Quaterniond(q)
    private val prevP = Vector3d(pos)

    private val _quat = Quaterniond()
    private val _quat2 = Quaterniond()

    override fun step() {
        prevQ.set(q)
        prevP.set(pos)

        pos.add(Vector3d(velocity).mul(Udar.CONFIG.timeStep))
        rotationIntegrator.process()

        torque.set(0.0)

        update()

        localInertia.set(calcInertia())
        localInverseInertia.set(1.0 / localInertia.x, 1.0 / localInertia.y, 1.0 / localInertia.z)
        updateII()

    }

    override fun update() {
        envContactGen.tick()

        updateVertices()
        updateBB()
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
        awake.set(true)
        startled.set(true)

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

    override fun capableCollision(other: ActiveBody): Int {
        return cuboidContactGen.capableCollision(other)
    }

    override fun capableCollision(other: EnvironmentBody): Int {
        return envContactGen.capableCollision(other)
    }

    override fun collides(other: EnvironmentBody, math: LocalMathUtil, out: A2SManifoldCollection): Boolean {
        return envContactGen.collides(other, math, out)
    }

    override fun collides(other: ActiveBody, math: LocalMathUtil, out: A2AManifoldCollection): Boolean {
        return cuboidContactGen.collides(other, math, out)
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
            normalize()
            if (!isFinite)
                set(0.0)
        }
    }

    override val startPoints: List<Vector3d>
        get() {
            return vertices.map { Vector3d(it) }
        }

    private val _naTemp = Array(3) { Vector3d() }

    override fun ensureNonAligned() {
        val tiny = 1e-14
        val perturbation = 1e-13

        _naTemp[0].set(1.0, 0.0, 0.0).rotate(q).normalize()
        _naTemp[1].set(0.0, 1.0, 0.0).rotate(q).normalize()
        _naTemp[2].set(0.0, 0.0, 1.0).rotate(q).normalize()

        if (_naTemp.any {
                it.distance(1.0, 0.0, 0.0) < tiny || it.distance(-1.0, 0.0, 0.0) < tiny ||
                it.distance(0.0, 1.0, 0.0) < tiny || it.distance(0.0, -1.0, 0.0) < tiny ||
                it.distance(0.0, 0.0, 1.0) < tiny || it.distance(0.0, 0.0, -1.0) < tiny
            }) {

            q.rotateXYZ(perturbation, perturbation, perturbation)
            ensureNonAligned()
        }
    }

//    private val contactGenerator: ContactGenerator = SATContactGenerator(this)

    override fun equals(other: Any?): Boolean {
        return other != null && other is Cuboid && other.uuid == uuid
    }

    override fun hashCode(): Int {
        return uuid.hashCode()
    }

    companion object {
        private val VALID_MATERIALS = listOf(
            Material.GLASS,
        )
    }
}

fun Vector3d.toStringFull(): String {
    return "( $x, $y, $z )"
}