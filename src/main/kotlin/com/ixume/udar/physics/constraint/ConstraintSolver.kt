package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.Udar
import com.ixume.udar.physics.cone.ConeConstraint
import com.ixume.udar.physics.cone.ConeConstraintSolver
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
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

class ConstraintSolver(
    val physicsWorld: PhysicsWorld,
) {
    val contactSolver = ContactSolver(this)
    val pointConstraintSolver = PointConstraintSolver(this)
    val hingeConstraintSolver = HingeConstraintSolver(this)
    val coneConstraintSolver = ConeConstraintSolver(this)
    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    private var bodyCount: Int = physicsWorld.activeBodies.size()

    @JvmField
    var flatBodyData: FloatArray = FloatArray(1)

    var debugDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)

    private val _quatd = Quaterniond()

    fun setup(
        pointConstraints: List<PointConstraint>,
        hingeConstraints: List<HingeConstraint>,
        coneConstraints: List<ConeConstraint>,
    ) {
        bodyCount = physicsWorld.activeBodies.size()
        buildFlatBodyData()

        if (debugDeltaLambdas.size != Udar.CONFIG.collision.normalIterations) {
            debugDeltaLambdas = FloatArray(Udar.CONFIG.collision.normalIterations)
        } else {
            debugDeltaLambdas.fill(0f)
        }

        contactSolver.setup()

        pointConstraintSolver.setup(pointConstraints)
        hingeConstraintSolver.setup(hingeConstraints)
        coneConstraintSolver.setup(coneConstraints)
    }

    fun reportLambdas() {
        val iterations = Udar.CONFIG.collision.normalIterations
        if (iterations <= 0) return
        for (i in 0..<debugDeltaLambdas.size) {
            debugDeltaLambdas[i]
        }

        val first = debugDeltaLambdas[0]
        println("∑(Δλ):")
        println("  %4d: %8.2e ".format(1, first) + "*".repeat(10))
        for (i in 2..debugDeltaLambdas.size) {
            val v = debugDeltaLambdas[i - 1]
            if (first == 0f) {
                println("  %4d: %8.2e ".format(i, v))
            } else {
                val num = abs(v / first * 10.0).roundToInt()
                val clamped = min(200, num)
                print("  %4d: %8.2e ".format(i, v) + "*".repeat(clamped))
                if (clamped < num) {
                    print("($num)")
                }
                println()
            }
        }
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
        pointConstraintSolver.solveVelocity(iteration)
        hingeConstraintSolver.solveVelocity()
        coneConstraintSolver.solveVelocity(iteration)
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