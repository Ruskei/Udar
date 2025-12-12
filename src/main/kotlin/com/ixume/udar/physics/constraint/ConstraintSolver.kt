package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.physics.cone.ConeConstraint
import com.ixume.udar.physics.cone.ConeConstraintSolver
import com.ixume.udar.physics.constraint.QuatMath.transform
import com.ixume.udar.physics.contact.v2.ContactSolver
import com.ixume.udar.physics.hinge.HingeConstraint
import com.ixume.udar.physics.hinge.HingeConstraintSolver
import com.ixume.udar.physics.position.PointConstraint
import com.ixume.udar.physics.position.PointConstraintSolver
import org.joml.Quaterniond
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
    val coneConstraintSolver = ConeConstraintSolver(this)

    private var bodyCount: Int = physicsWorld.activeBodies.size()

    @JvmField
    var flatBodyData: FloatArray = FloatArray(1)

    val debugData = ImpulseDebugData()

    private val _quatd = Quaterniond()

    fun setup(
        pointConstraints: List<PointConstraint>,
        hingeConstraints: List<HingeConstraint>,
        coneConstraints: List<ConeConstraint>,
    ) {
        buildFlatBodyData()

        debugData.setup()

        contactSolver.setup()

        pointConstraintSolver.setup(pointConstraints)
        hingeConstraintSolver.setup(hingeConstraints)
        coneConstraintSolver.setup(coneConstraints)
    }

    fun reportLambdas() {
        debugData.reportLambdas()
    }

    private fun buildFlatBodyData() {
        bodyCount = physicsWorld.activeBodies.size()
        if (flatBodyData.size < bodyCount * BODY_DATA_FLOATS) { // resizing is fine, since all the valid bodies should set their data anyway
            val ns = max(flatBodyData.size * 2, bodyCount * BODY_DATA_FLOATS)
            flatBodyData = FloatArray(ns)
        }

        var i = 0
        while (i < bodyCount) {
            val b = physicsWorld.activeBodies.fastGet(i)!!
            flatBodyData[i * BODY_DATA_FLOATS + 0] = b.velocity.x.toFloat()
            flatBodyData[i * BODY_DATA_FLOATS + 1] = b.velocity.y.toFloat()
            flatBodyData[i * BODY_DATA_FLOATS + 2] = b.velocity.z.toFloat()
            b.q.transform(b.omega.x, b.omega.y, b.omega.z) { x, y, z ->
                flatBodyData[i * BODY_DATA_FLOATS + 3] = x.toFloat()
                flatBodyData[i * BODY_DATA_FLOATS + 4] = y.toFloat()
                flatBodyData[i * BODY_DATA_FLOATS + 5] = z.toFloat()
            }

            i++
        }
    }


    fun solve1() {
//        pointConstraintSolver.solveVelocity()
        contactSolver.solveVelocity()
        hingeConstraintSolver.solveVelocity()
        coneConstraintSolver.solveVelocity()
    }

    fun solve2() {
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
        hingeConstraintSolver.heatUp()
        coneConstraintSolver.heatUp()
    }

    fun solvePositions() {
//        pointConstraintSolver.solvePosition()
        hingeConstraintSolver.solvePosition()
        coneConstraintSolver.solvePosition()
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