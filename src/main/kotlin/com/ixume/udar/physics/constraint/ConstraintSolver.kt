package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.physics.contact.v2.ContactSolver
import com.ixume.udar.physics.hinge.HingeConstraint

import com.ixume.udar.physics.hinge.HingeConstraintSolver
import com.ixume.udar.physics.position.PointConstraint
import com.ixume.udar.physics.position.PointConstraintSolver
import org.joml.Quaterniond
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.nio.FloatBuffer
import kotlin.math.max

class ConstraintSolver(
    val physicsWorld: PhysicsWorld,
) {
    val contactSolver = ContactSolver(this)
    val pointConstraintSolver = PointConstraintSolver(this)
    val hingeConstraintSolver = HingeConstraintSolver(this)
    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    private var bodyCount: Int = physicsWorld.activeBodies.size()

    @JvmField
    var flatBodyData: FloatArray = FloatArray(1)

    private val _quatd = Quaterniond()

    fun setup(
        pointConstraints: List<PointConstraint>,
        hingeConstraints: List<HingeConstraint>,
    ) {
        bodyCount = physicsWorld.activeBodies.size()
        buildFlatBodyData()

        contactSolver.setup()

        pointConstraintSolver.setup(pointConstraints)
        hingeConstraintSolver.setup(hingeConstraints)
    }

    private fun buildFlatBodyData() {
        val n = physicsWorld.activeBodies.size()
        if (flatBodyData.size < n * BODY_DATA_FLOATS) { // resizing is fine, since all the valid bodies should set their data anyway
            flatBodyData = FloatArray(max(flatBodyData.size * 2, n * BODY_DATA_FLOATS))
        }

        val buf = FloatBuffer.wrap(flatBodyData)

        var i = 0
        while (i < bodyCount) {
            val b = physicsWorld.activeBodies.fastGet(i)!!

            buf.putVector3f(_vec3.set(b.velocity))
            buf.putVector3f(_vec3.set(b.omega).rotate(_quat.set(b.q)))

            i++
        }
    }


    fun solve(iteration: Int) {
        pointConstraintSolver.solveVelocity()
        hingeConstraintSolver.solveVelocity()
        contactSolver.solveNormals(iteration)
    }

    fun solvePost() {
        contactSolver.solveFrictions()
    }

    fun write() {
        var i = 0
        val n = bodyCount * BODY_DATA_FLOATS
        while (i < n) {
            val body = physicsWorld.activeBodies.fastGet(i / BODY_DATA_FLOATS)!!

            body.velocity.from(i + V_OFFSET, flatBodyData)
            check(body.velocity.isFinite)
            body.omega.from(i + O_OFFSET, flatBodyData).rotate(_quatd.set(body.q).conjugate())
            check(body.omega.isFinite)

            i += BODY_DATA_FLOATS
        }

        contactSolver.heatUp()
    }

    fun solvePositions() {
        pointConstraintSolver.solvePosition()
        hingeConstraintSolver.solvePosition()
    }
}

fun FloatBuffer.putVector3f(v: Vector3f): FloatBuffer {
    put(v.x)
    put(v.y)
    put(v.z)

    return this
}

inline fun Vector3f.from(idx: Int, arr: FloatArray): Vector3f {
    x = arr[idx]
    y = arr[idx + 1]
    z = arr[idx + 2]

    return this
}

inline fun Vector3d.from(idx: Int, arr: FloatArray): Vector3d {
    x = arr[idx].toDouble()
    y = arr[idx + 1].toDouble()
    z = arr[idx + 2].toDouble()

    return this
}

const val BODY_DATA_FLOATS = 6
const val V_OFFSET = 0
const val O_OFFSET = 3