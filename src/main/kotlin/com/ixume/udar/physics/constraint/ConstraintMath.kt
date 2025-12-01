package com.ixume.udar.physics.constraint

import com.ixume.udar.body.active.ActiveBody
import java.lang.Math.fma
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
import kotlin.math.abs
import kotlin.math.sqrt

@OptIn(ExperimentalContracts::class)
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
        contract {
            callsInPlace(set, InvocationKind.EXACTLY_ONCE)
        }

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

    inline fun solveSymmetric6x6(
        m11: Float, m12: Float, m13: Float, m14: Float, m15: Float, m16: Float,
        m22: Float, m23: Float, m24: Float, m25: Float, m26: Float,
        m33: Float, m34: Float, m35: Float, m36: Float,
        m44: Float, m45: Float, m46: Float,
        m55: Float, m56: Float,
        m66: Float,

        v1: Float, v2: Float, v3: Float, v4: Float, v5: Float, v6: Float,

        after: (s1: Float, s2: Float, s3: Float, s4: Float, s5: Float, s6: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val l00 = sqrt(m11)

        val l10 = m12 / l00
        val l11 = sqrt(m22 - l10 * l10)

        val l20 = m13 / l00
        val l21 = (m23 - l20 * l10) / l11
        val l22 = sqrt(m33 - (l20 * l20 + l21 * l21))

        val l30 = m14 / l00
        val l31 = (m24 - l30 * l10) / l11
        val l32 = (m34 - (l30 * l20 + l31 * l21)) / l22
        val l33 = sqrt(m44 - (l30 * l30 + l31 * l31 + l32 * l32))

        val l40 = m15 / l00
        val l41 = (m25 - l40 * l10) / l11
        val l42 = (m35 - (l40 * l20 + l41 * l21)) / l22
        val l43 = (m45 - (l40 * l30 + l41 * l31 + l42 * l32)) / l33
        val l44 = sqrt(m55 - (l40 * l40 + l41 * l41 + l42 * l42 + l43 * l43))

        val l50 = m16 / l00
        val l51 = (m26 - l50 * l10) / l11
        val l52 = (m36 - (l50 * l20 + l51 * l21)) / l22
        val l53 = (m46 - (l50 * l30 + l51 * l31 + l52 * l32)) / l33
        val l54 = (m56 - (l50 * l40 + l51 * l41 + l52 * l42 + l53 * l43)) / l44
        val l55 = sqrt(m66 - (l50 * l50 + l51 * l51 + l52 * l52 + l53 * l53 + l54 * l54))

        val y0 = v1 / l00
        val y1 = (v2 - l10 * y0) / l11
        val y2 = (v3 - (l20 * y0 + l21 * y1)) / l22
        val y3 = (v4 - (l30 * y0 + l31 * y1 + l32 * y2)) / l33
        val y4 = (v5 - (l40 * y0 + l41 * y1 + l42 * y2 + l43 * y3)) / l44
        val y5 = (v6 - (l50 * y0 + l51 * y1 + l52 * y2 + l53 * y3 + l54 * y4)) / l55

        val s6 = y5 / l55
        val s5 = (y4 - l54 * s6) / l44
        val s4 = (y3 - (l43 * s5 + l53 * s6)) / l33
        val s3 = (y2 - (l32 * s4 + l42 * s5 + l52 * s6)) / l22
        val s2 = (y1 - (l21 * s3 + l31 * s4 + l41 * s5 + l51 * s6)) / l11
        val s1 = (y0 - (l10 * s2 + l20 * s3 + l30 * s4 + l40 * s5 + l50 * s6)) / l00

        after(s1, s2, s3, s4, s5, s6)
    }

    inline fun solveSymmetric5x5(
        m11: Float, m12: Float, m13: Float, m14: Float, m15: Float,
        m22: Float, m23: Float, m24: Float, m25: Float,
        m33: Float, m34: Float, m35: Float,
        m44: Float, m45: Float,
        m55: Float,

        v1: Float, v2: Float, v3: Float, v4: Float, v5: Float,

        after: (s1: Float, s2: Float, s3: Float, s4: Float, s5: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val l00 = sqrt(m11)

        val l10 = m12 / l00
        val l11 = sqrt(m22 - l10 * l10)

        val l20 = m13 / l00
        val l21 = (m23 - l20 * l10) / l11
        val l22 = sqrt(m33 - (l20 * l20 + l21 * l21))

        val l30 = m14 / l00
        val l31 = (m24 - l30 * l10) / l11
        val l32 = (m34 - (l30 * l20 + l31 * l21)) / l22
        val l33 = sqrt(m44 - (l30 * l30 + l31 * l31 + l32 * l32))

        val l40 = m15 / l00
        val l41 = (m25 - l40 * l10) / l11
        val l42 = (m35 - (l40 * l20 + l41 * l21)) / l22
        val l43 = (m45 - (l40 * l30 + l41 * l31 + l42 * l32)) / l33
        val l44 = sqrt(m55 - (l40 * l40 + l41 * l41 + l42 * l42 + l43 * l43))

        val y0 = v1 / l00
        val y1 = (v2 - l10 * y0) / l11
        val y2 = (v3 - (l20 * y0 + l21 * y1)) / l22
        val y3 = (v4 - (l30 * y0 + l31 * y1 + l32 * y2)) / l33
        val y4 = (v5 - (l40 * y0 + l41 * y1 + l42 * y2 + l43 * y3)) / l44

        val s4 = y4 / l44
        val s3 = (y3 - l43 * s4) / l33
        val s2 = (y2 - (l32 * s3 + l42 * s4)) / l22
        val s1 = (y1 - (l21 * s2 + l31 * s3 + l41 * s4)) / l11
        val s0 = (y0 - (l10 * s1 + l20 * s2 + l30 * s3 + l40 * s4)) / l00

        after(s0, s1, s2, s3, s4)
    }
}