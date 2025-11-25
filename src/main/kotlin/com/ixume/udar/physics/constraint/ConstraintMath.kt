package com.ixume.udar.physics.constraint

import com.ixume.udar.body.active.ActiveBody
import java.lang.Math.fma
import kotlin.math.abs

object ConstraintMath {
    inline fun addData(
        b1: ActiveBody,
        b2: ActiveBody,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        nx: Float,
        ny: Float,
        nz: Float,

        bias: Float,
        lambda: Float,

        set: (
            j10: Float,
            j11: Float,
            j12: Float,
            j13: Float,
            j14: Float,
            j15: Float,

            j23: Float,
            j24: Float,
            j25: Float,

            ej13: Float,
            ej14: Float,
            ej15: Float,

            ej23: Float,
            ej24: Float,
            ej25: Float,

            lambda: Float,
            iden: Float,
            bias: Float,
        ) -> Unit,
    ) {
//        contract {
//            callsInPlace(set, InvocationKind.EXACTLY_ONCE)
//        }

        val im1 = b1.inverseMass.toFloat()
        val im2 = b2.inverseMass.toFloat()

        val ii1 = b1.inverseInertia
        val ii2 = b2.inverseInertia

        val j10 = nx
        val j11 = ny
        val j12 = nz
        val j13 = fma(r1y, nz, -r1z * ny)
        val j14 = fma(r1z, nx, -r1x * nz)
        val j15 = fma(r1x, ny, -r1y * nx)

        val j20 = -nx
        val j21 = -ny
        val j22 = -nz
        val j23 = -fma(r2y, nz, -r2z * ny)
        val j24 = -fma(r2z, nx, -r2x * nz)
        val j25 = -fma(r2x, ny, -r2y * nx)

        val ej10 = nx * im1
        val ej11 = ny * im1
        val ej12 = nz * im1
        val ej13 = fma(ii1.m00.toFloat(), j13, fma(ii1.m10.toFloat(), j14, ii1.m20.toFloat() * j15))
        val ej14 = fma(ii1.m01.toFloat(), j13, fma(ii1.m11.toFloat(), j14, ii1.m21.toFloat() * j15))
        val ej15 = fma(ii1.m02.toFloat(), j13, fma(ii1.m12.toFloat(), j14, ii1.m22.toFloat() * j15))

        val ej20 = -nx * im2
        val ej21 = -ny * im2
        val ej22 = -nz * im2
        val ej23 = fma(ii2.m00.toFloat(), j23, fma(ii2.m10.toFloat(), j24, ii2.m20.toFloat() * j25))
        val ej24 = fma(ii2.m01.toFloat(), j23, fma(ii2.m11.toFloat(), j24, ii2.m21.toFloat() * j25))
        val ej25 = fma(ii2.m02.toFloat(), j23, fma(ii2.m12.toFloat(), j24, ii2.m22.toFloat() * j25))

        var iden =
            ej10 * j10 + ej11 * j11 + ej12 * j12 +
            j13 * ej13 + j14 * ej14 + j15 * ej15 +
            ej20 * j20 + ej21 * j21 + ej22 * j22 +
            j23 * ej23 + j24 * ej24 + j25 * ej25
        
        iden = if (abs(iden) < 1e-6) 0f else (1f / iden)

        set(
            j10,
            j11,
            j12,
            j13,
            j14,
            j15,

            j23,
            j24,
            j25,

            ej13,
            ej14,
            ej15,

            ej23,
            ej24,
            ej25,

            lambda,
            iden,
            bias,
        )
    }
}