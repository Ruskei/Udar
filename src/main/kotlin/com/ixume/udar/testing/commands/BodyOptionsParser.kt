package com.ixume.udar.testing.commands

import org.joml.Vector3d

data class BodyOptions(
    val v0: Vector3d,
    val dims: Vector3d,
    val l: Vector3d,
    val rot0: Vector3d,
    val density: Double,
    val hasGravity: Boolean,
    val trueOmega: Boolean,
) {
    companion object {
        fun fromArgs(
            args: Array<out String>
        ): BodyOptions {
            var v0 = Vector3d(0.0)
            var dims = Vector3d(0.99)
            var l = Vector3d(0.0)
            var rot0 = Vector3d(0.0)

            var density = 1.0
            var hasGravity = true
            var trueOmega = false

            var i = 0
            while (i < args.size) {
                val arg = args[i]

                if (arg == "--velocity" || arg == "-v") {
                    val buf = mutableListOf<Double>()
                    i++
                    while (i < args.size) {
                        val arg2 = args[i]
                        val d = arg2.toDoubleOrNull()
                        if (d == null) {
                            i--;
                            break
                        }

                        buf += d

                        if (buf.size == 3) {
                            v0 = Vector3d(buf[0], buf[1], buf[2])
                            break
                        }

                        i++
                    }
                }

                if (arg == "--dims" || arg == "-d") {
                    val buf = mutableListOf<Double>()
                    i++
                    while (i < args.size) {
                        val arg2 = args[i]
                        val d = arg2.toDoubleOrNull()
                        if (d == null) {
                            i--;
                            break
                        }

                        buf += d

                        if (buf.size == 3) {
                            dims = Vector3d(buf[0], buf[1], buf[2])
                            break
                        }

                        i++
                    }
                }

                if (arg == "--omega" || arg == "-o") {
                    val buf = mutableListOf<Double>()
                    i++
                    while (i < args.size) {
                        val arg2 = args[i]
                        val d = arg2.toDoubleOrNull()
                        if (d == null) {
                            i--;
                            break
                        }

                        buf += d

                        if (buf.size == 3) {
                            l = Vector3d(buf[0], buf[1], buf[2])
                            break
                        }

                        i++
                    }
                }

                if (arg == "--rot" || arg == "-r") {
                    val buf = mutableListOf<Double>()
                    i++
                    while (i < args.size) {
                        val arg2 = args[i]
                        val d = arg2.toDoubleOrNull()
                        if (d == null) {
                            i--;
                            break
                        }

                        buf += d

                        if (buf.size == 3) {
                            rot0 = Vector3d(buf[0], buf[1], buf[2])
                            break
                        }

                        i++
                    }
                }

                if (arg == "--density") {
                    i++
                    val arg2 = args[i]
                    val d = arg2.toDoubleOrNull()
                    if (d == null) {
                        i--
                    } else {
                        density = d
                    }
                }

                if (arg == "--gravity" || arg == "-g") {
                    hasGravity = true
                }

                if (arg == "--no-gravity" || arg == "-ng") {
                    hasGravity = false
                }

                if (arg == "--true-omega" || arg == "-to") {
                    trueOmega = true
                }

                i++
            }

            return BodyOptions(
                v0 = v0,
                dims = dims,
                l = l,
                rot0 = rot0,
                density = density,
                hasGravity = hasGravity,
                trueOmega = trueOmega,
            )
        }

        fun default(): BodyOptions {
            return BodyOptions(
                v0 = Vector3d(0.0),
                dims = Vector3d(0.99),
                l = Vector3d(0.0),
                rot0 = Vector3d(0.0),
                density = 1.0,
                hasGravity = true,
                trueOmega = false,
            )
        }
    }
}