package com.ixume.udar.physics.constraint

import com.ixume.udar.physics.Contact
import java.util.concurrent.Callable

class EnvConstraintSolveTask(
    val envConstraints: List<Contact>,
    val processors: Int,
    val proc: Int,
    val constraintSolver: LocalConstraintSolver,
    val normal: Boolean,
) : Callable<Unit> {
    override fun call() {
        val start = (envConstraints.size / processors) * proc
        val end = if (proc == processors - 1) {
            envConstraints.size
        } else {
            (envConstraints.size / processors) * (proc + 1)
        }

        constraintSolver.solveEnv(
            envConstraints,
            start,
            end,
            normal = normal,
        )
    }
}