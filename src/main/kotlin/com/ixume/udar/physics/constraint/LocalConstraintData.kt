package com.ixume.udar.physics.constraint

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.physics.angular.LocalAngularConstraintSolver
import com.ixume.udar.physics.contact.LocalContactSolver
import com.ixume.udar.physics.sphericaljoint.LocalSphericalJointSolver
import com.ixume.udar.physics.splitting.MassSplittingConstraintSolver
import org.joml.*
import java.lang.Math.fma
import java.nio.FloatBuffer
import kotlin.math.max

class LocalConstraintData(
    val physicsWorld: PhysicsWorld,
) {
    private val massSplittingConstraintSolver = MassSplittingConstraintSolver(this)

    private val contactSolver = LocalContactSolver(this)
    private val sphericalJointSolver = LocalSphericalJointSolver(this)
    private val angularConstraintSolver = LocalAngularConstraintSolver(this)
    private val _vec3 = Vector3f()
    private val _quat = Quaternionf()

    private var bodyCount: Int = physicsWorld.activeBodies.size()

    @JvmField
    var flatBodyData: FloatArray = FloatArray(1)

    private val _quatd = Quaterniond()

    fun setup() {
        bodyCount = physicsWorld.activeBodies.size()
        buildFlatBodyData()

        massSplittingConstraintSolver.setup()

//        contactSolver.setup()
//        sphericalJointSolver.setup()
//        angularConstraintSolver.setup()
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


    fun solve() {
        massSplittingConstraintSolver.solve()
//        angularConstraintSolver.solve()
//        sphericalJointSolver.solve()
//        contactSolver.solveNormal()
    }

    fun solvePost() {
//        contactSolver.solveFriction()
//        sphericalJointSolver.solveFriction()
    }

    fun write() {
        physicsWorld.prevContactMap.clear()
        physicsWorld.prevContactData.clear()

        physicsWorld.prevEnvContactMap.clear()
        physicsWorld.prevEnvContactData.clear()

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

        contactSolver.write()
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


inline fun Vector3f._mul(mat: Matrix3d): Vector3f {
    val lx: Float = x
    val ly: Float = y
    val lz: Float = z
    this.x = fma(mat.m00.toFloat(), lx, fma(mat.m10.toFloat(), ly, mat.m20.toFloat() * lz))
    this.y = fma(mat.m01.toFloat(), lx, fma(mat.m11.toFloat(), ly, mat.m21.toFloat() * lz))
    this.z = fma(mat.m02.toFloat(), lx, fma(mat.m12.toFloat(), ly, mat.m22.toFloat() * lz))

    return this
}

inline fun Vector3f._mul(mat: Matrix3f): Vector3f {
    val lx: Float = x
    val ly: Float = y
    val lz: Float = z
    this.x = fma(mat.m00, lx, fma(mat.m10, ly, mat.m20 * lz))
    this.y = fma(mat.m01, lx, fma(mat.m11, ly, mat.m21 * lz))
    this.z = fma(mat.m02, lx, fma(mat.m12, ly, mat.m22 * lz))

    return this
}

const val BODY_DATA_FLOATS = 6
const val V_OFFSET = 0
const val O_OFFSET = 3

fun FloatArray.vx(idx: Int): Float {
    return this[idx * BODY_DATA_FLOATS]
}

fun FloatArray.vy(idx: Int): Float {
    return this[idx * BODY_DATA_FLOATS + 1]
}

fun FloatArray.vz(idx: Int): Float {
    return this[idx * BODY_DATA_FLOATS + 2]
}

fun FloatArray.ox(idx: Int): Float {
    return this[idx * BODY_DATA_FLOATS + 3]
}

fun FloatArray.oy(idx: Int): Float {
    return this[idx * BODY_DATA_FLOATS + 4]
}

fun FloatArray.oz(idx: Int): Float {
    return this[idx * BODY_DATA_FLOATS + 5]
}

fun FloatArray.vx(idx: Int, value: Float) {
    this[idx * BODY_DATA_FLOATS] = value
}

fun FloatArray.vy(idx: Int, value: Float) {
    this[idx * BODY_DATA_FLOATS + 1] = value
}

fun FloatArray.vz(idx: Int, value: Float) {
    this[idx * BODY_DATA_FLOATS + 2] = value
}

fun FloatArray.ox(idx: Int, value: Float) {
    this[idx * BODY_DATA_FLOATS + 3] = value
}

fun FloatArray.oy(idx: Int, value: Float) {
    this[idx * BODY_DATA_FLOATS + 4] = value
}

fun FloatArray.oz(idx: Int, value: Float) {
    this[idx * BODY_DATA_FLOATS + 5] = value
}
