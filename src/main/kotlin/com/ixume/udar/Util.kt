package com.ixume.udar

import org.joml.Matrix3d
import org.joml.Vector3d
import kotlin.math.abs
import kotlin.math.sqrt

fun <T> T.applyIf(condition: Boolean, action: T.() -> Unit): T {
    if (condition) action()
    return this
}


/**
 * Mutates 'matrix'
 * https://en.wikipedia.org/wiki/Jacobi_eigenvalue_algorithm#Algorithm
 */
fun jacobiEigenDecomposition(S: Matrix3d, maxItrs: Int = 100): Pair<Vector3d, Matrix3d> {

    val epsilon = 1e-20
    var k: Int
    var l: Int
    var m: Int
    var s = 0.0
    var c = 0.0
    var t: Double
    var p: Double
    var y: Double
    var d: Double
    var r: Double
    val eigenValues = Vector3d(S.getRowColumn(0, 0), S.getRowColumn(1, 1), S.getRowColumn(2, 2))
    val eigenVectors = Matrix3d()

    var state = 3
    val ind = IntArray(3)
    val changed = BooleanArray(3) { true }

    fun maxInd(k: Int): Int {
        var m = k + 1
        var i = k + 2
        while (i <= 2) {
            if (abs(S.getRowColumn(k, i)) > abs(S.getRowColumn(k, m))) {
                m = i
            }
            ++i
        }

        return m
    }

    fun update(k: Int, t: Double) {
        y = eigenValues[k]
        eigenValues[k] = y + t
        if (changed[k] && abs(y - eigenValues[k]) < epsilon) {
            changed[k] = false
            --state
        } else if (!changed[k] && abs(y - eigenValues[k]) > epsilon) {
            changed[k] = true
            ++state
        }
    }

    fun rotate(k: Int, l: Int, i: Int, j: Int) {
        val nSkl = c * S.getRowColumn(k, l) - s * S.getRowColumn(i, j)
        val nSij = s * S.getRowColumn(k, l) + c * S.getRowColumn(i, j)
        S.setRowColumn(k, l, nSkl)
        S.setRowColumn(i, j, nSij)
    }

    for (k in 0..2) {
        ind[k] = maxInd(k)
    }

    var itr = 0
    while (state != 0 && ++itr < maxItrs) {
        m = 0
        k = 1
        if (abs(S.getRowColumn(k, ind[k])) > abs(S.getRowColumn(m, ind[m]))) {
            m = k
        }

        k = m
        l = ind[m]
        p = S.getRowColumn(k, l)

        if (p == 0.0) {
            return eigenValues to eigenVectors
        }

        y = (eigenValues[l] - eigenValues[k]) / 2.0
        d = abs(y) + sqrt(p * p + y * y)
        r = sqrt(p * p + d * d)
        c = d / r
        s = p / r
        t = p * p / d

        if (y < 0) {
            s = -s
            t = -t
        }

        S.setRowColumn(k, l, 0.0)
        update(k, -t)
        update(l, t)

        for (i in 0..(k - 1)) {
            rotate(i, k, i, l)
        }

        for (i in (k + 1)..(l - 1)) {
            rotate(k, i, i, l)
        }

        for (i in (l + 1)..(2)) {
            rotate(k, i, l, i)
        }

        for (i in 0..2) {
            val nEik = c * eigenVectors.getRowColumn(i, k) - s * eigenVectors.getRowColumn(i, l)
            val nEil = s * eigenVectors.getRowColumn(i, k) + c * eigenVectors.getRowColumn(i, l)
            eigenVectors.setRowColumn(i, k, nEik)
            eigenVectors.setRowColumn(i, l, nEil)
        }

        for (i in 0..2) {
            ind[i] = maxInd(i)
        }
    }

    return eigenValues to eigenVectors
}

operator fun Vector3d.set(idx: Int, value: Double) {
    setComponent(idx, value)
}