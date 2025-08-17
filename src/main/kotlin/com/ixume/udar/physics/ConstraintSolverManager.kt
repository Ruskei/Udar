package com.ixume.udar.physics

import com.ixume.udar.Udar
import com.ixume.udar.graph.GraphUtil
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking

class ConstraintSolverManager(
    val processors: Int,
    val scope: CoroutineScope,
) {
    private val constraintSolvers = Array(processors) { LocalConstraintSolver() }

    private var time = 0
    private var runningSolveTime = 0.0
    private val dataInterval = 10000

    //colors can be intra-parallelized
    //store the range for each color for each thread
    fun solve(graphUtil: GraphUtil, envConstraints: List<Contact>) {
        val ranges = buildRanges(graphUtil)
        val colors = graphUtil.maxColor + 1

        //every iteration, go through every color, compute its constraints in parallel

        var normalItrs = 1
        while (normalItrs <= Udar.CONFIG.collision.normalIterations) {
            iteration(colors, ranges, graphUtil, envConstraints, true)

            normalItrs++
        }

        var frictionItrs = 1
        while (frictionItrs <= Udar.CONFIG.collision.frictionIterations) {
            iteration(colors, ranges, graphUtil, envConstraints, false)

            frictionItrs++
        }
    }

    private fun iteration(
        colors: Int,
        ranges: IntArray,
        graphUtil: GraphUtil,
        envConstraints: List<Contact>,
        normal: Boolean
    ) {
        var color = 0
        while (color < colors) {
            val rangeColorOffset = color * (processors + 1)

            var sum = 0
            var i = 0

            while (i < graphUtil.necessaryLongs) {
                val l = graphUtil.colorSet[color * graphUtil.necessaryLongs + i]
                sum += l.countOneBits()

                i++
            }

            val jobStart = System.nanoTime()
            val job = scope.launch {
                (0..<processors).forEach { proc ->
                    launch {
                        val start = ranges[rangeColorOffset + proc]
                        var end = proc
                        while (end < processors && ranges[rangeColorOffset + end + 1] != 0) {
                            end++
                        }

                        end = ranges[rangeColorOffset + end]

//                        println("start: $start, idx: ${rangeColorOffset + proc} end: $end, idx: ${rangeColorOffset + end}, ranges: ${ranges.joinToString { it.toString() }}")

                        if (end == 0) {
                            return@launch
                        }

                        constraintSolvers[proc].solve(
                            graphUtil,
                            color,
                            start = start,
                            end = end,
                            normal = normal,
                        )
                    }
                }
            }

            runBlocking {
                job.join()

                val jobDur = System.nanoTime() - jobStart

                runningSolveTime += (jobDur.toDouble() / sum.toDouble()) / dataInterval.toDouble()
                time++

                if (time % dataInterval == 0) {
                    println("$runningSolveTime ns/constraint")
                    runningSolveTime = 0.0
                }


//                if (sum > 10) {
//                    val max = times.maxBy { it.first }
//                    val min = times.minBy { it.first }
//                    println("sum: $sum DIFF: ${max.first - min.first} first_time: ${max.first} first_amount: ${max.second} second_time: ${min.first} second_amount: ${min.second} average sub: ${(max.first + min.second).toDouble() / (max.second + min.second).toDouble()} ns/constraint total average: ${jobDur.toDouble() / sum.toDouble()} ns/constraint")
//                }
            }

            color++
        }

        val job = scope.launch {
            (0..<processors).forEach { i ->
                launch {
                    val start = (envConstraints.size / processors) * i
                    val end = if (i == processors - 1) {
                        envConstraints.size
                    } else {
                        (envConstraints.size / processors) * (i + 1)
                    }

                    constraintSolvers[i].solveEnv(
                        envConstraints,
                        start,
                        end,
                        normal = normal,
                    )
                }
            }
        }

        runBlocking { job.join() }
    }

    private fun buildRanges(graphUtil: GraphUtil): IntArray {
        val maxColor = graphUtil.maxColor
        val colorSet = graphUtil.colorSet
        val necessaryLongs = graphUtil.necessaryLongs
        var color = 0

        val ranges = IntArray((maxColor + 1) * (processors + 1)) //inclusive -> exclusive with merged boundaries

        while (color <= maxColor) {
            var j = 0
            var rangeIdx = 0

            var contactsForThisColor = 0
            var v = 0
            while (v < necessaryLongs) {
                contactsForThisColor += colorSet[color * necessaryLongs + v].countOneBits()

                v++
            }

            val contactsPerProcessor = contactsForThisColor / processors

            var countedContacts = 0
            while (j < necessaryLongs) {
                val l = colorSet[color * necessaryLongs + j]

                var k = l.countTrailingZeroBits()
                while (k < 64) {
                    countedContacts++
                    //j * 64 + k == other id
                    if (rangeIdx < processors && countedContacts >= contactsPerProcessor * (rangeIdx + 1)) {
                        rangeIdx++
                        ranges[color * (processors + 1) + rangeIdx] = j * 64 + k
                    }

                    k += ((l shr (k + 1)).countTrailingZeroBits() + 1)
                }

                j++
            }

            ranges[color * (processors + 1) + rangeIdx] = necessaryLongs * 64

            color++
        }

//        checkInvariants(ranges, graphUtil)

        return ranges
    }

    fun checkInvariants(ranges: IntArray, graphUtil: GraphUtil) {
        val maxColor = graphUtil.maxColor
        val colorSet = graphUtil.colorSet
        val necessaryLongs = graphUtil.necessaryLongs

        var color = 0

        while (color <= maxColor) {
            val rangeColorOffset = color * (processors + 1)
            var rangeSum = 0
            var rangeIdx = 0

            while (rangeIdx < processors) {
                val start = ranges[rangeColorOffset + rangeIdx]
                val end = ranges[rangeColorOffset + rangeIdx + 1]
                if (end == 0) {
                    break
                }

                var i = start shr 6
                var inRange = true
                var first = 0.inv()

                while (inRange && i < necessaryLongs) {
                    val l = colorSet[color * necessaryLongs + i]
                    var k = (l shr (start and first)).countTrailingZeroBits() + (start and 0b111111 and first)
                    first = 0
                    while (k < 64) {
                        if (i * 64 + k < end) {
                            rangeSum++
                        } else {
                            inRange = false
                            break
                        }

                        k += ((l shr (k + 1)).countTrailingZeroBits() + 1)
                    }

                    i++
                }

                rangeIdx++
            }

            var e = 0
            while (e < processors && ranges[rangeColorOffset + e + 1] != 0) {
                e++
            }

            var rawSum = 0
            var i = 0
            while (i < necessaryLongs) {
                val l = colorSet[color * necessaryLongs + i]
                var k = l.countTrailingZeroBits()
                while (k < 64) {
                    if (i * 64 + k in ranges[rangeColorOffset]..<ranges[e]) {
                        rawSum++
                    }

                    k += ((l shr (k + 1)).countTrailingZeroBits() + 1)
                }

                i++
            }

            check(rawSum == rangeSum) {
                val relevantRange = StringBuilder("[")

                var x = 0
                while (x <= processors) {
                    relevantRange.append(ranges[rangeColorOffset + x])
                    relevantRange.append(" -> ")

                    x++
                }

                relevantRange.append("]")

                val relevantBits = StringBuilder("{")

                var z = 0
                while (z < necessaryLongs) {
                    relevantBits.append(" " + colorSet[color * necessaryLongs + z])

                    z++
                }

                relevantBits.append(" }")

                "rawSum: $rawSum rangeSum: $rangeSum rangeStart: ${ranges[rangeColorOffset]} rangeEnd: ${ranges[e]} full: $relevantRange bits: $relevantBits"
            }

            color++
        }
    }
}