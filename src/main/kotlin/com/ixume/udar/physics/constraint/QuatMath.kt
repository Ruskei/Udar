package com.ixume.udar.physics.constraint

import com.ixume.udar.Udar
import org.joml.Quaterniond
import java.lang.Math.fma
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
import kotlin.math.sqrt

@OptIn(ExperimentalContracts::class)
object QuatMath {
    inline fun quatTransform(
        qx: Double, qy: Double, qz: Double, qw: Double,
        x: Double, y: Double, z: Double,

        after: (x: Double, y: Double, z: Double) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val xx = qx * qx
        val yy = qy * qy
        val zz = qz * qz
        val ww = qw * qw
        val xy = qx * qy
        val xz = qx * qz
        val yz = qy * qz
        val xw = qx * qw
        val zw = qz * qw
        val yw = qy * qw
        val k = 1 / (xx + yy + zz + ww)
        if (k.isNaN()) {
            Udar.LOGGER.warning("Quaternion is 0!")
            after(x, y, z)
            return
        }

        after(
            fma((xx - yy - zz + ww) * k, x, fma(2 * (xy - zw) * k, y, (2 * (xz + yw) * k) * z)),
            fma(2 * (xy + zw) * k, x, fma((yy - xx - zz + ww) * k, y, (2 * (yz - xw) * k) * z)),
            fma(2 * (xz - yw) * k, x, fma(2 * (yz + xw) * k, y, ((zz - xx - yy + ww) * k) * z))
        )
    }

    inline fun Quaterniond.transform(
        x: Double, y: Double, z: Double,

        after: (x: Double, y: Double, z: Double) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        quatTransform(
            this.x, this.y, this.z, this.w,
            x, y, z
        ) { x, y, z ->
            after(x, y, z)
        }
    }

    /**
     * q1 * q2
     */
    inline fun quatMul(
        q1x: Double, q1y: Double, q1z: Double, q1w: Double,
        q2x: Double, q2y: Double, q2z: Double, q2w: Double,
        after: (x: Double, y: Double, z: Double, w: Double) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        after(
            fma(q1w, q2x, fma(q1x, q2w, fma(q1y, q2z, -q1z * q2y))),
            fma(q1w, q2y, fma(-q1x, q2z, fma(q1y, q2w, q1z * q2x))),
            fma(q1w, q2z, fma(q1x, q2y, fma(-q1y, q2x, q1z * q2w))),
            fma(q1w, q2w, fma(-q1x, q2x, fma(-q1y, q2y, -q1z * q2z)))
        )
    }

    fun Quaterniond.safeNormalize(epsilon: Double = 1e-8): Quaterniond {
        val len = sqrt(lengthSquared())
        if (len < epsilon) {
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0
        } else {
            val inv = 1 / len
            x *= inv
            y *= inv
            z *= inv
            w *= inv
        }

        return this
    }
}