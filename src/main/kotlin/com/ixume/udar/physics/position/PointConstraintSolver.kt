package com.ixume.udar.physics.position

import com.ixume.udar.Udar
import com.ixume.udar.physics.constraint.ConstraintMath
import com.ixume.udar.physics.constraint.ConstraintSolver
import org.joml.Vector3d
import java.lang.Math.fma
import kotlin.math.max
import kotlin.math.sqrt

class PointConstraintSolver(val parent: ConstraintSolver) {
    private var constraintData = ConstraintData(0)
    private var numConstraints = 0

    private var dt = Udar.CONFIG.timeStep.toFloat()
    private var bias = Udar.CONFIG.positionConstraint.bias
    private var slop = Udar.CONFIG.positionConstraint.slop
    private var carryover = Udar.CONFIG.positionConstraint.carryover
    private var relaxation = Udar.CONFIG.positionConstraint.relaxation

    private val _tempV = Vector3d()

    fun setup(constraints: List<PointConstraint>) {
        dt = Udar.CONFIG.timeStep.toFloat()
        bias = Udar.CONFIG.positionConstraint.bias
        slop = Udar.CONFIG.positionConstraint.slop
        carryover = Udar.CONFIG.positionConstraint.carryover
        relaxation = Udar.CONFIG.positionConstraint.relaxation

        numConstraints = constraints.size

        if (constraintData.sizeFor(numConstraints) != constraintData.value.size) {
            val s = constraintData.sizeFor(numConstraints)
            constraintData = ConstraintData(s)
        }

        var i = 0
        while (i < numConstraints) {
            val constraint = constraints[i]

            val b1 = constraint.b1
            val b2 = constraint.b2

            b1.q.transform(constraint.r1x.toDouble(), constraint.r1y.toDouble(), constraint.r1z.toDouble(), _tempV)
            val lr1x = _tempV.x.toFloat()
            val lr1y = _tempV.y.toFloat()
            val lr1z = _tempV.z.toFloat()
            
            val r1x = lr1x + b1.pos.x.toFloat()
            val r1y = lr1y + b1.pos.y.toFloat()
            val r1z = lr1z + b1.pos.z.toFloat()

            b2.q.transform(constraint.r2x.toDouble(), constraint.r2y.toDouble(), constraint.r2z.toDouble(), _tempV)
            val lr2x = _tempV.x.toFloat()
            val lr2y = _tempV.y.toFloat()
            val lr2z = _tempV.z.toFloat()

            val r2x = lr2x + b2.pos.x.toFloat()
            val r2y = lr2y + b2.pos.y.toFloat()
            val r2z = lr2z + b2.pos.z.toFloat()

            val d = sqrt((r1x - r2x) * (r1x - r2x) + (r1y - r2y) * (r1y - r2y) + (r1z - r2z) * (r1z - r2z))
            val bias = bias * max(0f, d - slop) / dt

            var nx = r1x - r2x
            var ny = r1y - r2y
            var nz = r1z - r2z
            val l = sqrt(nx * nx + ny * ny + nz * nz)
            if (l < 1e-6) {
                nx = 0f
                ny = 0f
                nz = 0f
            } else {
                nx /= l
                ny /= l
                nz /= l
            }

            ConstraintMath.addData(
                b1 = b1,
                b2 = b2,

                r1x = lr1x,
                r1y = lr1y,
                r1z = lr1z,

                r2x = lr2x,
                r2y = lr2y,
                r2z = lr2z,

                nx = nx,
                ny = ny,
                nz = nz,

                bias = bias,
                lambda = 0f,
            ) { j10, j11, j12, j13, j14, j15, j23, j24, j25, ej13, ej14, ej15, ej23, ej24, ej25, lambda, iden, bias ->
                constraintData.set(
                    cursor = i * CONSTRAINT_DATA_SIZE,

                    body1Idx = b1.idx,
                    body2Idx = b2.idx,

                    im1 = b1.inverseMass.toFloat(),
                    im2 = b2.inverseMass.toFloat(),

                    j10 = j10,
                    j11 = j11,
                    j12 = j12,
                    j13 = j13,
                    j14 = j14,
                    j15 = j15,

                    j23 = j23,
                    j24 = j24,
                    j25 = j25,

                    ej13 = ej13,
                    ej14 = ej14,
                    ej15 = ej15,

                    ej23 = ej23,
                    ej24 = ej24,
                    ej25 = ej25,

                    lambda = lambda,
                    iden = iden,
                    bias = bias,
                )
            }

            i++
        }
    }

    private inline fun solve(
        data: ConstraintData,
        num: Int,

        lambdaTransform: (l: Float, lIdx: Int) -> Float,
    ) {
        val bodyData = parent.flatBodyData
        data.forEach(
            numConstraints = num,
        ) { b1Idx, b2Idx, im1, im2, rawIdx ->
            val lambdaIdx = rawIdx + LAMBDA_OFFSET
            val t = data[lambdaIdx]

            val v10 = bodyData[b1Idx * 6 + 0]
            val v11 = bodyData[b1Idx * 6 + 1]
            val v12 = bodyData[b1Idx * 6 + 2]
            val v13 = bodyData[b1Idx * 6 + 3]
            val v14 = bodyData[b1Idx * 6 + 4]
            val v15 = bodyData[b1Idx * 6 + 5]

            val v20 = bodyData[b2Idx * 6 + 0]
            val v21 = bodyData[b2Idx * 6 + 1]
            val v22 = bodyData[b2Idx * 6 + 2]
            val v23 = bodyData[b2Idx * 6 + 3]
            val v24 = bodyData[b2Idx * 6 + 4]
            val v25 = bodyData[b2Idx * 6 + 5]

            val bias = data[rawIdx + BIAS_OFFSET]
            val iden = data[rawIdx + IDEN_OFFSET]

            val j10 = data[rawIdx + 4]
            val j11 = data[rawIdx + 5]
            val j12 = data[rawIdx + 6]
            val j13 = data[rawIdx + 7]
            val j14 = data[rawIdx + 8]
            val j15 = data[rawIdx + 9]

            val j20 = -j10
            val j21 = -j11
            val j22 = -j12
            val j23 = data[rawIdx + 10]
            val j24 = data[rawIdx + 11]
            val j25 = data[rawIdx + 12]

            val ej10 = j10 * im1
            val ej11 = j11 * im1
            val ej12 = j12 * im1
            val ej13 = data[rawIdx + 13]
            val ej14 = data[rawIdx + 14]
            val ej15 = data[rawIdx + 15]

            val ej20 = j20 * im2
            val ej21 = j21 * im2
            val ej22 = j22 * im2
            val ej23 = data[rawIdx + 16]
            val ej24 = data[rawIdx + 17]
            val ej25 = data[rawIdx + 18]

            val l = lambdaTransform(
                t - iden * fma(
                    j10, v10,
                    fma(
                        j11, v11,
                        fma(
                            j12, v12,
                            fma(
                                j13, v13,
                                fma(
                                    j14, v14,
                                    fma(
                                        j15, v15,
                                        fma(
                                            j20, v20,
                                            fma(
                                                j21, v21,
                                                fma(
                                                    j22, v22,
                                                    fma(
                                                        j23, v23,
                                                        fma(
                                                            j24, v24,
                                                            fma(
                                                                j25, v25,
                                                                bias
                                                            )
                                                        )
                                                    )
                                                )
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                ),
                lambdaIdx
            )

            data[lambdaIdx] = l
            
            bodyData[b1Idx * 6 + 0] += ej10 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 1] += ej11 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 2] += ej12 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 3] += ej13 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 4] += ej14 * (l - t) * relaxation
            bodyData[b1Idx * 6 + 5] += ej15 * (l - t) * relaxation

            bodyData[b2Idx * 6 + 0] += ej20 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 1] += ej21 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 2] += ej22 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 3] += ej23 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 4] += ej24 * (l - t) * relaxation
            bodyData[b2Idx * 6 + 5] += ej25 * (l - t) * relaxation
        }
    }

    fun solveNormals() {
        solve(constraintData, numConstraints) { l, _ -> l }
    }
}

@JvmInline
private value class ConstraintData(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * CONSTRAINT_DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        body1Idx: Int,
        body2Idx: Int,

        im1: Float,
        im2: Float,

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
    ) {
        value[cursor + 0] = Float.fromBits(body1Idx)
        value[cursor + 1] = Float.fromBits(body2Idx)
        value[cursor + 2] = im1
        value[cursor + 3] = im2
        value[cursor + 4] = j10
        value[cursor + 5] = j11
        value[cursor + 6] = j12
        value[cursor + 7] = j13
        value[cursor + 8] = j14
        value[cursor + 9] = j15
        value[cursor + 10] = j23
        value[cursor + 11] = j24
        value[cursor + 12] = j25
        value[cursor + 13] = ej13
        value[cursor + 14] = ej14
        value[cursor + 15] = ej15
        value[cursor + 16] = ej23
        value[cursor + 17] = ej24
        value[cursor + 18] = ej25
        value[cursor + 19] = lambda
        value[cursor + 20] = iden
        value[cursor + 21] = bias
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            b1Idx: Int,
            b2Idx: Int,

            im1: Float,
            im2: Float,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * CONSTRAINT_DATA_SIZE + 0].toRawBits(),
                this[i * CONSTRAINT_DATA_SIZE + 1].toRawBits(),
                this[i * CONSTRAINT_DATA_SIZE + 2],
                this[i * CONSTRAINT_DATA_SIZE + 3],
                i * CONSTRAINT_DATA_SIZE,
            )

            i++
        }
    }
}

/*
[]Constraint

Constraint = {
  body1Idx: Int
  body2Idx: Int
  im1: Float, im2: Float
  
  j10: Float
  j11: Float
  j12: Float
  j13: Float
  j14: Float
  j15: Float
  //omit j20..2 since those are just j10..3 negated
  j23: Float
  j24: Float
  j25: Float
  
  ej13: Float
  ej14: Float
  ej15: Float
  
  ej23: Float
  ej24: Float
  ej25: Float
  
  lambda: Float
  iden: Float
  bias: Float
}
 */
private const val CONSTRAINT_DATA_SIZE = 22
private const val LAMBDA_OFFSET = 19
private const val IDEN_OFFSET = 20
private const val BIAS_OFFSET = 21
