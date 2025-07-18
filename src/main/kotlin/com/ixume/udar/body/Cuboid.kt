package com.ixume.udar.body

import com.ixume.udar.Udar
import com.ixume.udar.applyIf
import com.ixume.udar.body.ActiveBody.Companion.TIME_STEP
import com.ixume.udar.collisiondetection.contactgeneration.ContactGenerator
import com.ixume.udar.collisiondetection.contactgeneration.SATContactGenerator
import com.ixume.udar.collisiondetection.edgeCrosses
import com.ixume.udar.physics.CollisionResult
import com.ixume.udar.physics.Contact
import com.ixume.udar.collisiondetection.mesh.Axis
import com.ixume.udar.collisiondetection.mesh.Edge
import com.ixume.udar.collisiondetection.mesh.Mesh
import com.ixume.udar.collisiondetection.mesh.Mesh.Companion.INNER
import com.ixume.udar.collisiondetection.mesh.MeshFace
import com.ixume.udar.testing.debugConnect
import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.Particle
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
import org.joml.Vector3i
import java.util.UUID
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

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
) : ActiveBody {
    override val id = UUID.randomUUID()
    override val type: BodyType = BodyType.ACTIVE

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

    override var contacts: MutableList<Contact> = mutableListOf()
    override var previousContacts: MutableList<Contact> = mutableListOf()

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


        previousContacts.clear()
        previousContacts += contacts
        contacts.clear()

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

    private val contactGenerator: ContactGenerator = SATContactGenerator(this)

    override fun collidesEnvironment(): List<CollisionResult> { return contactGenerator.collidesEnvironment() }

    override fun collidesBody(other: ActiveBody): CollisionResult? {
        // map from difference to MINE to OTHER
        val originals = mutableMapOf<Vector3d, Pair<Vector3d, Vector3d>>()

        fun minkowski(dir: Vector3d): Vector3d {
            val a = support(dir)
            val b = other.support(Vector3d(dir).negate())
            val diff = Vector3d(a).sub(b)
            originals[diff] = a to b

//            println("DIFF: $diff FROM: $a $b")

            return diff
        }

        val initialAxis = Vector3d(other.pos).sub(pos)
        var a = minkowski(initialAxis)
        val s = mutableListOf(a)
        val d = Vector3d(a).negate()

        var i = 0
        repeat(64) {
            i++
            a = minkowski(d)
            if (a.dot(d) < 0.0) {
                return null
            }

            s += a
            if (nearestSimplex(s, d)) {
//                println("COLLISION after $i iterations")

                val aP = s[2]
                val bP = s[1]
                val cP = s[0]
                val dP = s[3]

//                println("tetrahedron: a: $aP b: $bP c: $cP d: $dP")

                //verify that norms are pointing outside
                val da = Vector3d(aP).sub(dP)
                val db = Vector3d(bP).sub(dP)
                val dc = Vector3d(cP).sub(dP)

                val ab = Vector3d(bP).sub(aP)
                val ac = Vector3d(cP).sub(aP)

                val dab = Vector3d(da).cross(db) //SHOULD FACE AWAY FROM C
                val dbc = Vector3d(db).cross(dc) //SHOULD FACE AWAY FROM A
                val dca = Vector3d(dc).cross(da) //SHOULD FACE AWAY FROM B
                val abc = Vector3d(ac).cross(ab) //SHOULD FACE AWAY FROM D

                val winding = abc.dot(dP) < 0.0 //TRUE: COUNTERCLOCKWISE, FALSE: CLOCKWISE
//                println("WINDING: ${if (winding) "COUNTERCLOCKWISE" else "CLOCKWISE"}")
                check(winding == (dab.dot(cP) < 0.0) == (dbc.dot(aP) < 0.0) == (dca.dot(bP) < 0.0)) //EVERYTHING MUST BE WOUND THE SAME WAY

                val shape = mutableListOf(
                    if (winding) Triple(aP, cP, bP) else Triple(aP, bP, cP),
                    if (winding) Triple(aP, bP, dP) else Triple(aP, dP, bP),
                    if (winding) Triple(bP, cP, dP) else Triple(bP, dP, cP),
                    if (winding) Triple(cP, aP, dP) else Triple(cP, dP, aP),
                )

                repeat(64) {
                    val dn = closestNormal(Vector3d(), shape)
                    if (dn != null) {
                        val (dis, norm, faces) = dn

                        val sup = minkowski(norm)

                        val dis2 = sup.dot(norm)
                        if (dis2 - dis < 0.0000001) {
                            var coefficients: Vector3d? = null
                            var closestFace: Triple<Vector3d, Vector3d, Vector3d>? = null
//                            println("faces.size: ${faces.size}")
                            for (face in faces) {
                                val r = toBarycentric(Vector3d(), face.first, face.second, face.third, true)
                                if (r != null) {
                                    coefficients = r
                                    closestFace = face
                                }
                            }

                            if (coefficients == null) {
                                throw IllegalStateException("No coefficients found")
//                                AbstractParticleEmitter.INSTANCE.logger.warning("No coefficients found")
//                                coefficients = toBarycentric(Vector3d(), faces[0].first, faces[0].second, faces[0].third, false)!!
//                                closestFace = faces[0]
                            }

                            val (a3, b3, c3) = closestFace!!
                            val (a3mine, a3other) = originals[a3]!!
                            val (b3mine, b3other) = originals[b3]!!
                            val (c3mine, c3other) = originals[c3]!!

                            val aC = coefficients.x
                            val bC = coefficients.y
                            val cC = coefficients.z
                            val doDebug = aC + bC + cC > 1.001
                            val myInt =
                                Vector3d(a3mine).mul(aC).add(Vector3d(b3mine).mul(bC)).add(Vector3d(c3mine).mul(cC))
                            val otherInt =
                                Vector3d(a3other).mul(aC).add(Vector3d(b3other).mul(bC))
                                    .add(Vector3d(c3other).mul(cC))
                            if (doDebug) {
                                println(
                                    """
                                    minkowski:
                                     - a: $a3
                                     - b: $b3
                                     - c: $c3
                                    mine: $myInt
                                     - a: $a3mine C: $aC
                                     - b: $b3mine C: $bC
                                     - c: $c3mine C: $cC
                                    other: $otherInt
                                     - a: $a3other C: $aC
                                     - b: $b3other C: $bC
                                     - c: $c3other C: $cC
                                    shape:${"\n"}${shape.joinToString(separator = "FACE:\n") { " - a: ${it.first}\n - b: ${it.second}\n - c: ${it.third}\n" }}
                                """.trimIndent()
                                )
                            }

                            return CollisionResult(
                                myInt,
                                norm,
                                dis2,
//                                shape,
//                                closestFace,
//                                originals,
                            )
                        } else {
                            addPoint(shape, sup)
                        }
                    } else {
                        //1d epa
                        var dClosest = Double.MAX_VALUE
                        var dNorm = Vector3d()
                        var vertexClosest: Vector3d? = null

                        for ((a2, b2, c2) in shape) {
                            val a2l = a2.length()
                            val b2l = b2.length()
                            val c2l = c2.length()

                            if (a2l < dClosest || b2l < dClosest || c2l < dClosest)
                                if (a2l < b2l) {
                                    // a < b
                                    if (c2l < a2l) {
                                        // c < a < b

                                        dClosest = c2l
                                        dNorm = Vector3d(c2).normalize()
                                        vertexClosest = c2
                                    } else {
                                        dClosest = a2l
                                        dNorm = Vector3d(a2).normalize()
                                        vertexClosest = a2
                                    }
                                } else {
                                    // b < a
                                    if (c2l < b2l) {
                                        // c < b < a
                                        dClosest = c2l
                                        dNorm = Vector3d(c2).normalize()
                                        vertexClosest = c2
                                    } else {
                                        dClosest = b2l
                                        dNorm = Vector3d(b2).normalize()
                                        vertexClosest = b2
                                    }
                                }
                        }

                        if (dNorm == Vector3d() || !dNorm.isFinite) return null
                        val (closestMine, _) = originals[vertexClosest!!]!!

                        return CollisionResult(
                            closestMine,
                            dNorm,
                            dClosest,
//                            shape,
//                            Triple(vertexClosest, vertexClosest, vertexClosest),
//                            originals,
                        )
                    }
                }

                return null

            }
        }

        return null
    }

    private fun nearestSimplex(s: MutableList<Vector3d>, dir: Vector3d): Boolean {
        when (s.size) {
            2 -> {
                val a = s[1]
                val b = s[0]
                val ab = Vector3d(b).sub(a)
                val ao = Vector3d(a).negate()

                dir.set(Vector3d(ab).cross(ao).cross(ab))

                return false
            }

            3 -> {
                val a = s[2]
                val b = s[1]
                val c = s[0]

                val ab = Vector3d(b).sub(a)
                val ac = Vector3d(c).sub(a)
                val ao = Vector3d(a).negate()

                val abc = Vector3d(ab).cross(ac)

                dir.set(Vector3d(abc).mul(if (abc.dot(ao) > 0.0) 1.0 else -1.0))

                return false
            }

            4 -> {
                val a = s[2]
                val b = s[1]
                val c = s[0]
                val d = s[3]

                val da = Vector3d(a).sub(d)
                val db = Vector3d(b).sub(d)
                val dc = Vector3d(c).sub(d)

                val d0 = Vector3d(d).negate()

                val dab = Vector3d(da).cross(db)
                val dbc = Vector3d(db).cross(dc)
                val dca = Vector3d(dc).cross(da)

                if (dab.dot(d0) > 0.0) {
                    s.remove(c)
                    dir.set(dab)

                    return false
                } else if (dbc.dot(d0) > 0.0) {
                    s.remove(a)
                    dir.set(dbc)

                    return false
                } else if (dca.dot(d0) > 0.0) {
                    s.remove(b)
                    dir.set(dca)

                    return false
                } else {
                    return true
                }
            }

            else -> throw IllegalArgumentException("Simplex not in R^(1..3)")
        }
    }


    data class ClosestResult(
        val distance: Double,
        val normal: Vector3d,
        val faces: List<Triple<Vector3d, Vector3d, Vector3d>>,
    )

    private fun addPoint(shape: MutableList<Triple<Vector3d, Vector3d, Vector3d>>, point: Vector3d) {
        // holds vertices associated with edges
        // non uniques will always be wound 2 ways, and will be removed
        // so unique leftovers are all wound correctly
        val uniqueEdges = mutableListOf<Pair<Vector3d, Vector3d>>()
        val facesToRemove = mutableListOf<Triple<Vector3d, Vector3d, Vector3d>>()

        for (face in shape) {
            val (a, b, c) = face
            val ab = Vector3d(b).sub(a)
            val ac = Vector3d(c).sub(a)

            val n = Vector3d(ab).cross(ac)
            val relP = Vector3d(point).sub(a)
            if (relP.dot(n) > 0.0) {
                //if any edge in 'uniqueEdges' is the same or the reverse of ab, ac, or cb, remove it
                //otherwise, add edge in wound order
                var foundAB = false
                var foundBC = false
                var foundCA = false

                val edgesToRemove = mutableListOf<Pair<Vector3d, Vector3d>>()
                for (uniqueEdge in uniqueEdges) {
                    if (
                        (uniqueEdge.first.distanceSquared(a) < EPSILON && uniqueEdge.second.distanceSquared(b) < EPSILON)
                        || (uniqueEdge.first.distanceSquared(b) < EPSILON && uniqueEdge.second.distanceSquared(a) < EPSILON)
                    ) {
                        foundAB = true
                        edgesToRemove += uniqueEdge
                        continue
                    }

                    if (
                        (uniqueEdge.first.distanceSquared(b) < EPSILON && uniqueEdge.second.distanceSquared(c) < EPSILON)
                        || (uniqueEdge.first.distanceSquared(c) < EPSILON && uniqueEdge.second.distanceSquared(b) < EPSILON)
                    ) {
                        foundBC = true
                        edgesToRemove += uniqueEdge
                        continue
                    }

                    if (
                        (uniqueEdge.first.distanceSquared(c) < EPSILON && uniqueEdge.second.distanceSquared(a) < EPSILON)
                        || (uniqueEdge.first.distanceSquared(a) < EPSILON && uniqueEdge.second.distanceSquared(c) < EPSILON)
                    ) {
                        foundCA = true
                        edgesToRemove += uniqueEdge
                        continue
                    }
                }

                uniqueEdges.removeAll(edgesToRemove)

                if (!foundAB) {
                    uniqueEdges += a to b
                }

                if (!foundBC) {
                    uniqueEdges += b to c
                }

                if (!foundCA) {
                    uniqueEdges += c to a
                }

                facesToRemove += face

            }
        }

        shape.removeAll(facesToRemove)
        facesToRemove.clear()

        for ((uniqueStart, uniqueEnd) in uniqueEdges) {
            shape += Triple(uniqueStart, uniqueEnd, point)
        }

//        if (!shape.isConvex()) {
//            println("CONCAVE SHAPE")
//        }
    }

    override fun visualize() {
//        if (PhysicsCommand.DEBUG_MESH_LEVEL > 0) {
//            cachedMesh.visualize(world, visualizeFaces = false, visualizeEdges = true)
//        }
    }

    companion object {
        private val VALID_MATERIALS = listOf(
            Material.SEA_LANTERN,
        )

        fun toBarycentric(
            point: Vector3d,
            a: Vector3d,
            b: Vector3d,
            c: Vector3d,
            failhard: Boolean = false,
        ): Vector3d? {
            //first make sure point is on same plane as a,b,c
            val ab = Vector3d(b).sub(a)
            val bc = Vector3d(c).sub(b)
            val ca = Vector3d(a).sub(c)

            val arp = Vector3d(point).sub(a)
            val n = Vector3d(ab).cross(ca).normalize()
//            val fac = if (n.dot(point))
            val d = Vector3d(arp).sub(Vector3d(n).mul(n.dot(arp))).add(a)

            val da = Vector3d(d).sub(a)
            val db = Vector3d(d).sub(b)
            val dc = Vector3d(d).sub(c)

            val total = 0.5 * Vector3d(ab).cross(ca).length()

            val dabA = 0.5 * Vector3d(da).cross(ab).length()
            val cA = dabA / total
            val dbcA = 0.5 * Vector3d(db).cross(bc).length()
            val cB = dbcA / total
            val dcaA = 0.5 * Vector3d(dc).cross(ca).length()
            val cC = dcaA / total

            if (cA + cB + cC > 1.001) {
//                println("WARNING: cA: $cA cB: $cB cC: $cC SUM: ${cA + cB + cC}\ntotal area: $total dab: $dabA dbc: $dbcA dca: $dcaA")
                if (failhard) return null
            }
//
            return Vector3d(cB, cC, cA)
        }

        fun closestNormal(point: Vector3d, shape: List<Triple<Vector3d, Vector3d, Vector3d>>): ClosestResult? {
//            println("CLOSEST NORMAL")
            var dClosest = Double.MAX_VALUE
            var nClosest = Vector3d()
            var closestFaces: MutableList<Triple<Vector3d, Vector3d, Vector3d>> = mutableListOf()

            var winding: Double? = null

            for ((a, b, c) in shape) {
                val relPoint = Vector3d(point).sub(a)
                val ab = Vector3d(b).sub(a)
                val ac = Vector3d(c).sub(a)

                val n = Vector3d(ab).cross(ac).normalize()
                if (!n.isFinite) {
                    //a,b,c are collinear, resolution vector is just 1d epa aka whichever direction is closest to origin
                    return null
                }

                val d = Vector3d(n).dot(relPoint)
//                println("d: $d winding: $winding")
                if (winding == null) winding = sign(d)
                else if (sign(d) != winding) println("WINDING MISMATCH")
//                println("  - a: $a b: $b c: $c n: $n d: $d")
//                println("d: $d")
//            println("  - a: $a b: $b c: $c n: $n d: $d")

                if (abs(d.absoluteValue - dClosest) < 0.0000001) {
                    closestFaces += Triple(a, b, c)
                } else if (d.absoluteValue < dClosest) {
                    dClosest = d.absoluteValue
                    nClosest = n
                    closestFaces = mutableListOf(Triple(a, b, c))
                }
            }

            check(nClosest.lengthSquared() > 0.0)

            return ClosestResult(dClosest, nClosest, closestFaces)
        }

        private fun Vector3d.inside(
            xRange: ClosedRange<Double>,
            yRange: ClosedRange<Double>,
            zRange: ClosedRange<Double>,
        ): Boolean {
            return x in xRange && y in yRange && z in zRange
        }

        /**
         * Finds closest distance between two lines defined by a0, a and b0, b
         * @return closest point on segment a, closest point on segment b, distance, null if parallel
         */
        fun closestPointsBetweenSegments(
            a0: Vector3d,
            a1: Vector3d,
            b0: Vector3d,
            b1: Vector3d
        ): Triple<Vector3d, Vector3d, Double> {
            val a = Vector3d(a1).sub(a0).normalize()
            val b = Vector3d(b1).sub(b0).normalize()

            val axRange = (min(a0.x, a1.x) - EPSILON)..(max(a0.x, a1.x) + EPSILON)
            val ayRange = (min(a0.y, a1.y) - EPSILON)..(max(a0.y, a1.y) + EPSILON)
            val azRange = (min(a0.z, a1.z) - EPSILON)..(max(a0.z, a1.z) + EPSILON)
            val bxRange = (min(b0.x, b1.x) - EPSILON)..(max(b0.x, b1.x) + EPSILON)
            val byRange = (min(b0.y, b1.y) - EPSILON)..(max(b0.y, b1.y) + EPSILON)
            val bzRange = (min(b0.z, b1.z) - EPSILON)..(max(b0.z, b1.z) + EPSILON)

            //closest points is either on the lines, on a vertex and a line, or between 2 vertices
            //if line-line is valid, then that's the answer
            //then check vertex-line
            //if vertex-line is valid then that's the answer
            //otherwise check vertex-vertex
            //check line-line
            val (onLLA, onLLB, llD) = closestPointsBetweenLines(a0, a, b0, b) ?: return let {
                val p1 = Vector3d(a0).mul(0.5).add(Vector3d(a1).mul(0.5))
                val p2 = Vector3d(b0).mul(0.5).add(Vector3d(b1).mul(0.5))
                Triple(
                    p1,
                    p2,
                    p1.distance(p2),
                )
            }

            if (onLLA.inside(axRange, ayRange, azRange) && onLLB.inside(bxRange, byRange, bzRange)) return Triple(
                onLLA,
                onLLB,
                llD
            )

            //test vertex-line:
            //a0-b, a1-b, b0-a, b1-a
            //return closest valid, because if it's valid then it must be closer than it is to a vertex, otherwise try vertex-vertex
            val vls = mutableListOf<Triple<Vector3d, Vector3d, Double>>()

            closestPointOnLine(b0, b, a0).let {
                if (it.first.inside(bxRange, byRange, bzRange)) vls += Triple(
                    a0,
                    it.first,
                    it.second
                )
            }
            closestPointOnLine(b0, b, a1).let {
                if (it.first.inside(bxRange, byRange, bzRange)) vls += Triple(
                    a1,
                    it.first,
                    it.second
                )
            }
            closestPointOnLine(a0, a, b0).let {
                if (it.first.inside(axRange, ayRange, azRange)) vls += Triple(
                    it.first,
                    b0,
                    it.second
                )
            }
            closestPointOnLine(a0, a, b1).let {
                if (it.first.inside(axRange, ayRange, azRange)) vls += Triple(
                    it.first,
                    b1,
                    it.second
                )
            }

            val vlMin = vls.minByOrNull { it.third }

            val vvs = listOf(
                Triple(a0, b0, a0.distance(b0)),
                Triple(a0, b1, a0.distance(b1)),
                Triple(a1, b0, a1.distance(b0)),
                Triple(a1, b1, a1.distance(b1)),
            )

            val vvsMin = vvs.minBy { it.third }

            if (vlMin == null) return vvsMin
            return if (vvsMin.third < vlMin.third) vvsMin else vlMin
        }

        /**
         * Finds closest distance between two lines defined by a0, a and b0, b
         * @return closest point on line a, closest point on line b, distance, null if parallel
         */
        private fun closestPointsBetweenLines(
            a0: Vector3d,
            a: Vector3d,
            b0: Vector3d,
            b: Vector3d
        ): Triple<Vector3d, Vector3d, Double>? {
            val an = Vector3d(a).normalize()
            val bn = Vector3d(b).normalize()
            val cn = Vector3d(an).cross(bn).normalize()
            if (!cn.isFinite) return null

            val d = Vector3d(b0).sub(a0)

            val r = Vector3d(d).sub(Vector3d(a).mul(d.dot(a))).sub(Vector3d(cn).mul(d.dot(cn)))
            if (r.length() < 0.0000001) return null
            val bClosest = Vector3d(b0).sub(Vector3d(b).mul(r.length() / b.dot(Vector3d(r).normalize())))
            val c = Vector3d(cn).mul(Vector3d(d).sub(Vector3d(a).mul(d.dot(a))).dot(cn))

            return Triple(Vector3d(bClosest).sub(c), bClosest, c.length())
        }

        /**
         * Closest point on a line to another point
         * @return the closest point, the distance
         */
        private fun closestPointOnLine(x0: Vector3d, x: Vector3d, p: Vector3d): Pair<Vector3d, Double> {
            val xn = Vector3d(x).normalize()
            val closestPoint = Vector3d(x0).add(Vector3d(x).mul(Vector3d(p).sub(x0).dot(xn)))

            return closestPoint to closestPoint.distance(p)
        }

        fun List<Triple<Vector3d, Vector3d, Vector3d>>.isConvex(): Boolean {
            if (this.size < 4) {
                return true
            }

            val allVertices = this.flatMap { listOf(it.first, it.second, it.third) }.toSet()

            if (allVertices.size < 4) {
                return true
            }

            val epsilon = 1e-7

            for (triangle in this) {
                val p1 = triangle.first
                val p2 = triangle.second
                val p3 = triangle.third

                val edge1 = p2.sub(p1, Vector3d())
                val edge2 = p3.sub(p1, Vector3d())

                val normal = edge1.cross(edge2, Vector3d())

                if (normal.lengthSquared() < epsilon) {
                    continue
                }

                normal.normalize()

                for (vertex in allVertices) {
                    val vectorToVertex = vertex.sub(p1, Vector3d())

                    val distance = vectorToVertex.dot(normal)

                    if (distance > epsilon) {
                        return false
                    }
                }
            }

            return true
        }
    }
}

private const val EPSILON = 1e-11
private const val FUDGE = 4.0
fun Vector3d.toStringFull(): String {
    return "( $x, $y, $z )"
}