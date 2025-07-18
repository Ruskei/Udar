package com.ixume.udar.testing

import com.google.gson.*
import com.google.gson.stream.JsonReader
import com.google.gson.stream.JsonWriter
import com.ixume.udar.Udar
import org.joml.Vector3d
import java.io.File
import java.io.FileReader

object TestingConfigLoader {
    private val gson: Gson = GsonBuilder()
        .registerTypeAdapter(Config::class.java, Config)
        .setPrettyPrinting()
        .create()

    fun load() {
        val dataFolder = Udar.INSTANCE.dataFolder
        dataFolder.mkdirs()

        val configFile = File(dataFolder, "testing.json")
        if (!configFile.exists()) {
            Udar.CONFIG = Config()
            configFile.writeText(gson.toJson(Udar.CONFIG))

            Udar.LOGGER.info("Created config!")

            return
        }

        try {
            val cfg = gson.fromJson(FileReader(configFile), Config::class.java)
            if (cfg == null) {
                Udar.CONFIG = Config()
                configFile.writeText(gson.toJson(Udar.CONFIG))

                Udar.LOGGER.info("Created config!")

                return
            }

            Udar.LOGGER.info("Successfully loaded config!")
            Udar.CONFIG = cfg
        } catch (e: Exception) {
            Udar.LOGGER.warning("Failed to load configuration:")
            e.printStackTrace()
            Udar.LOGGER.info("Defaulting to default config.")

            Udar.CONFIG = Config()

            return
        }
    }
}

class Config(
    val enabled: Boolean = false,
    val timeStep: Double = DEFAULT_TIMESTEP,
    val gravity: Vector3d = Vector3d(0.0, -5.0, 0.0),
    val collision: CollisionConfig = CollisionConfig(),
    val satConfig: SATConfig = SATConfig(),
    val debug: DebugConfig = DebugConfig(),
) {
    companion object : TypeAdapter<Config>() {
        private var DEFAULT_GRAVITY = Vector3d(0.0, -5.0, 0.0)
        private var DEFAULT_TIMESTEP = 0.005

        override fun write(out: JsonWriter, value: Config?) {
            if (value == null) {
                out.nullValue()
                return
            }

            out.beginObject()

            out.name("enabled").value(value.enabled)
            out.name("gravity"); Vector3dTypeAdapter.write(out, value.gravity)
            out.name("collision"); CollisionConfig.write(out, value.collision)
            out.name("debug"); DebugConfig.write(out, value.debug)

            out.endObject()
        }

        override fun read(`in`: JsonReader): Config? {
            `in`.beginObject()

            var enabled = false
            var timestep = DEFAULT_TIMESTEP
            var gravity = DEFAULT_GRAVITY
            var collision = CollisionConfig()
            var debug = DebugConfig()
            var sat = SATConfig()

            while (`in`.hasNext()) {
                val name = `in`.nextName()
                when (name) {
                    "enabled" -> {
                        enabled = `in`.nextBoolean()
                    }
                    "timestep" -> timestep = `in`.nextDouble()
                    "gravity" -> {
                        gravity = Vector3dTypeAdapter.read(`in`)
                    }
                    "collision" -> {
                        collision = CollisionConfig.read(`in`)
                    }
                    "debug" -> {
                        debug = DebugConfig.read(`in`)
                    }
                    "sat" -> {
                        sat = SATConfig.read(`in`)
                    }
                    else -> `in`.skipValue()
                }
            }

            `in`.endObject()

            return Config(enabled, timestep, gravity, collision, sat, debug)
        }
    }

    class CollisionConfig(
        val bias: Double = DEFAULT_BIAS,
        val passiveSlop: Double = DEFAULT_PASSIVE_SLOP,
        val activeSlop: Double = DEFAULT_ACTIVE_SLOP,
        val friction: Double = DEFAULT_FRICTION,
        val lambdaCarryover: Double = DEFAULT_CARRYOVER,
    ) {
        companion object : TypeAdapter<CollisionConfig>() {
            private const val DEFAULT_BIAS = 0.15
            private const val DEFAULT_PASSIVE_SLOP = 0.0001
            private const val DEFAULT_ACTIVE_SLOP = 0.001
            private const val DEFAULT_FRICTION = 0.3
            private const val DEFAULT_CARRYOVER = 0.3

            override fun write(
                out: JsonWriter,
                value: CollisionConfig?
            ) {
                if (value == null) {
                    out.nullValue()
                    return
                }

                out.beginObject()

                out.name("bias").value(value.bias)
                out.name("passiveSlop").value(value.passiveSlop)
                out.name("activeSlop").value(value.activeSlop)
                out.name("friction").value(value.friction)
                out.name("lambdaCarryover").value(value.lambdaCarryover)

                out.endObject()
            }

            override fun read(`in`: JsonReader): CollisionConfig {
                `in`.beginObject()

                var bias = DEFAULT_BIAS
                var passiveSlop = DEFAULT_PASSIVE_SLOP
                var activeSlop = DEFAULT_ACTIVE_SLOP
                var friction = DEFAULT_FRICTION
                var lambdaCarryover = DEFAULT_CARRYOVER

                while (`in`.hasNext()) {
                    val name = `in`.nextName()
                    when (name) {
                        "bias" -> { bias = `in`.nextDouble() }
                        "passiveSlop" -> { passiveSlop = `in`.nextDouble() }
                        "activeSlop" -> { activeSlop = `in`.nextDouble() }
                        "friction" -> { friction = `in`.nextDouble() }
                        "lambdaCarryover" -> { lambdaCarryover = `in`.nextDouble() }
                        else -> `in`.skipValue()
                    }
                }

                `in`.endObject()

                return CollisionConfig(bias, passiveSlop, activeSlop, friction, lambdaCarryover)
            }
        }
    }

    class SATConfig(
        val fudge: Double = DEFAULT_FUDGE
    ) {
        companion object : TypeAdapter<SATConfig>() {
            private const val DEFAULT_FUDGE = 4.0
            override fun write(
                out: JsonWriter,
                value: SATConfig?
            ) {
                if (value == null) {
                    out.nullValue()
                    return
                }

                out.beginObject()

                out.name("fudge").value(value.fudge)

                out.endObject()
            }

            override fun read(`in`: JsonReader): SATConfig {
                `in`.beginObject()

                var fudge = DEFAULT_FUDGE

                while (`in`.hasNext()) {
                    val name = `in`.nextName()
                    when (name) {
                        "fudge" -> fudge = `in`.nextDouble()
                        else -> `in`.skipValue()
                    }
                }

                `in`.endObject()

                return SATConfig(fudge)
            }
        }
    }

    class DebugConfig(
        val frequency: Int = 2,
        val mesh: Int = 0,
    ) {
        companion object : TypeAdapter<DebugConfig>() {
            override fun write(
                out: JsonWriter,
                value: DebugConfig?
            ) {
                if (value == null) {
                    out.nullValue()
                    return
                }

                out.beginObject()

                out.name("frequency").value(value.frequency)
                out.name("mesh").value(value.mesh)

                out.endObject()
            }

            override fun read(`in`: JsonReader): DebugConfig {
                `in`.beginObject()

                var frequency = 2
                var mesh = 0

                while (`in`.hasNext()) {
                    val name = `in`.nextName()
                    when (name) {
                        "frequency" -> {
                            frequency = `in`.nextInt()
                        }
                        "mesh" -> {
                            mesh = `in`.nextInt()
                        }
                        else -> {
                            `in`.skipValue()
                        }
                    }
                }

                `in`.endObject()

                return DebugConfig(frequency, mesh)
            }
        }
    }
}

object Vector3dTypeAdapter : TypeAdapter<Vector3d>() {
    override fun write(out: JsonWriter, value: Vector3d?) {
        if (value == null) {
            out.nullValue()
            return
        }

        out.beginArray()

        out.value(value.x)
        out.value(value.y)
        out.value(value.z)

        out.endArray()
    }

    override fun read(`in`: JsonReader): Vector3d {
        `in`.beginArray()

        val x = `in`.nextDouble()
        val y = `in`.nextDouble()
        val z = `in`.nextDouble()

        `in`.endArray()

        return Vector3d(x, y, z)
    }
}
