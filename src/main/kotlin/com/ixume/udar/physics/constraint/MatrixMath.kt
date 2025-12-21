package com.ixume.udar.physics.constraint

import com.ixume.udar.physics.constraint.MiscMath.sqrte
import org.joml.Matrix3d
import java.lang.Math.fma
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
import kotlin.math.sqrt

@OptIn(ExperimentalContracts::class)
object MatrixMath {
    inline fun solveSymmetric3x3(
        m11: Float, m12: Float, m13: Float,
        m22: Float, m23: Float,
        m33: Float,

        v1: Float, v2: Float, v3: Float,

        after: (s1: Float, s2: Float, s3: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val l00 = sqrte(m11)

        val l10 = m12 / l00
        val l11 = sqrte(m22 - l10 * l10)

        val l20 = m13 / l00
        val l21 = (m23 - l20 * l10) / l11
        val l22 = sqrte(m33 - (l20 * l20 + l21 * l21))

        val y0 = v1 / l00
        val y1 = (v2 - l10 * y0) / l11
        val y2 = (v3 - (l20 * y0 + l21 * y1)) / l22

        val s3 = y2 / l22
        val s2 = (y1 - s3 * l21) / l11
        val s1 = (y0 - s2 * l10 - s3 * l20) / l00

        after(s1, s2, s3)
    }

    inline fun solveSymmetric4x4(
        m11: Float, m12: Float, m13: Float, m14: Float,
        m22: Float, m23: Float, m24: Float,
        m33: Float, m34: Float,
        m44: Float,
        v1: Float, v2: Float, v3: Float, v4: Float,
        after: (s1: Float, s2: Float, s3: Float, s4: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }
        val l00 = sqrte(m11)

        val l10 = m12 / l00
        val l11 = sqrte(m22 - l10 * l10)

        val l20 = m13 / l00
        val l21 = (m23 - l20 * l10) / l11
        val l22 = sqrte(m33 - (l20 * l20 + l21 * l21))

        val l30 = m14 / l00
        val l31 = (m24 - l30 * l10) / l11
        val l32 = (m34 - (l30 * l20 + l31 * l21)) / l22
        val l33 = sqrte(m44 - (l30 * l30 + l31 * l31 + l32 * l32))

        val y0 = v1 / l00
        val y1 = (v2 - l10 * y0) / l11
        val y2 = (v3 - (l20 * y0 + l21 * y1)) / l22
        val y3 = (v4 - (l30 * y0 + l31 * y1 + l32 * y2)) / l33

        val s3 = y3 / l33
        val s2 = (y2 - l32 * s3) / l22
        val s1 = (y1 - (l21 * s2 + l31 * s3)) / l11
        val s0 = (y0 - (l10 * s1 + l20 * s2 + l30 * s3)) / l00

        after(s0, s1, s2, s3)
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

        val l00 = sqrte(m11)

        val l10 = m12 / l00
        val l11 = sqrte(m22 - l10 * l10)

        val l20 = m13 / l00
        val l21 = (m23 - l20 * l10) / l11
        val l22 = sqrte(m33 - (l20 * l20 + l21 * l21))

        val l30 = m14 / l00
        val l31 = (m24 - l30 * l10) / l11
        val l32 = (m34 - (l30 * l20 + l31 * l21)) / l22
        val l33 = sqrte(m44 - (l30 * l30 + l31 * l31 + l32 * l32))

        val l40 = m15 / l00
        val l41 = (m25 - l40 * l10) / l11
        val l42 = (m35 - (l40 * l20 + l41 * l21)) / l22
        val l43 = (m45 - (l40 * l30 + l41 * l31 + l42 * l32)) / l33
        val l44 = sqrte(m55 - (l40 * l40 + l41 * l41 + l42 * l42 + l43 * l43))

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

        val l00 = sqrte(m11)

        val l10 = m12 / l00
        val l11 = sqrte(m22 - l10 * l10)

        val l20 = m13 / l00
        val l21 = (m23 - l20 * l10) / l11
        val l22 = sqrte(m33 - (l20 * l20 + l21 * l21))

        val l30 = m14 / l00
        val l31 = (m24 - l30 * l10) / l11
        val l32 = (m34 - (l30 * l20 + l31 * l21)) / l22
        val l33 = sqrte(m44 - (l30 * l30 + l31 * l31 + l32 * l32))

        val l40 = m15 / l00
        val l41 = (m25 - l40 * l10) / l11
        val l42 = (m35 - (l40 * l20 + l41 * l21)) / l22
        val l43 = (m45 - (l40 * l30 + l41 * l31 + l42 * l32)) / l33
        val l44 = sqrte(m55 - (l40 * l40 + l41 * l41 + l42 * l42 + l43 * l43))

        val l50 = m16 / l00
        val l51 = (m26 - l50 * l10) / l11
        val l52 = (m36 - (l50 * l20 + l51 * l21)) / l22
        val l53 = (m46 - (l50 * l30 + l51 * l31 + l52 * l32)) / l33
        val l54 = (m56 - (l50 * l40 + l51 * l41 + l52 * l42 + l53 * l43)) / l44
        val l55 = sqrte(m66 - (l50 * l50 + l51 * l51 + l52 * l52 + l53 * l53 + l54 * l54))

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

    inline fun cross(
        ax: Float, ay: Float, az: Float,
        bx: Float, by: Float, bz: Float,

        after: (cx: Float, cy: Float, cz: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val cx = fma(ay, bz, -az * by)
        val cy = fma(az, bx, -ax * bz)
        val cz = fma(ax, by, -ay * bx)

        after(cx, cy, cz)
    }

    fun dot(
        ax: Float, ay: Float, az: Float,
        bx: Float, by: Float, bz: Float,
    ): Float {
        return fma(ax, bx, fma(ay, by, az * bz))
    }

    inline fun normalize(
        x: Float, y: Float, z: Float,

        after: (x: Float, y: Float, z: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val l = sqrt(fma(x, x, fma(y, y, z * z)))
        after(x / l, y / l, z / l)
    }

    /**
     * Rejects a from b
     */
    inline fun reject(
        ax: Float, ay: Float, az: Float,
        bx: Float, by: Float, bz: Float,

        after: (cx: Float, cy: Float, cz: Float) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        val m = fma(ax, bx, fma(ay, by, az * bz)) / sqrte(fma(ax, ax, fma(ay, ay, az * az)))
        val cx = bx - ax * m
        val cy = by - ay * m
        val cz = bz - az * m

        after(cx, cy, cz)
    }

    inline fun Matrix3d.mul(
        x0: Double, x1: Double, x2: Double,

        after: (y0: Double, y1: Double, y2: Double) -> Unit,
    ) {
        contract {
            callsInPlace(after, InvocationKind.EXACTLY_ONCE)
        }

        after(
            fma(m00, x0, fma(m10, x1, m20 * x2)),
            fma(m01, x0, fma(m11, x1, m21 * x2)),
            fma(m02, x0, fma(m12, x1, m22 * x2)),
        )
    }
}