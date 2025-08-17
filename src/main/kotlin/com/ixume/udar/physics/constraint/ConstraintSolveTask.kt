package com.ixume.udar.physics.constraint

import com.ixume.udar.graph.GraphUtil
import java.util.concurrent.Callable

class ConstraintSolveTask(
    val ranges: IntArray,
    val proc: Int,
    val color: Int,
    val processors: Int,
    val constraintSolver: LocalConstraintSolver,
    val graphUtil: GraphUtil,
    val normal: Boolean,
) : Callable<Unit> {
    override fun call() {
        val rangeColorOffset = color * (processors + 1)
        val start = ranges[rangeColorOffset + proc]
        var end = proc
        while (end < processors && ranges[rangeColorOffset + end + 1] != 0) {
            end++
        }

        end = ranges[rangeColorOffset + end]

//                        println("start: $start, idx: ${rangeColorOffset + proc} end: $end, idx: ${rangeColorOffset + end}, ranges: ${ranges.joinToString { it.toString() }}")

        if (end == 0) {
            return
        }

        constraintSolver.solve(
            graphUtil,
            color,
            start = start,
            end = end,
            normal = normal,
        )
    }
}