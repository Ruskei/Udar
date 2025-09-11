package com.ixume.udar.testing

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.InstanceCreator
import com.google.gson.TypeAdapter
import com.google.gson.stream.JsonReader
import com.google.gson.stream.JsonToken
import com.google.gson.stream.JsonWriter
import com.ixume.udar.Udar
import org.bukkit.Bukkit
import org.bukkit.entity.Player
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3d
import java.io.File
import java.io.FileReader
import java.lang.reflect.Type

private val gson: Gson = GsonBuilder()
    .registerTypeAdapter(Vector3d::class.java, Vector3dTypeAdapter)
    .registerTypeAdapter(Config::class.java, Config)
    .registerTypeAdapter(Config.SDFConfig::class.java, Config.SDFConfig)
    .registerTypeAdapter(Config.SATConfig::class.java, Config.SATConfig)
    .registerTypeAdapter(Config.DebugConfig::class.java, Config.DebugConfig)
    .registerTypeAdapter(Config.DebugConfig.Tests::class.java, TestsAdapter)
    .setPrettyPrinting()
    .create()

object ConfigLoader {
    fun load() {
        val dataFolder = Udar.INSTANCE.dataFolder
        dataFolder.mkdirs()

        val configFile = File(dataFolder, "testing.json")
        if (!configFile.exists()) {
            Udar.CONFIG = Config()
            configFile.writeText(gson.toJson(Config()))

            Udar.LOGGER.info("Created config!")

            return
        }

        val reader = FileReader(configFile)

        try {
            val cfg = gson.fromJson(reader, Config::class.java)
            if (cfg == null) {
                Udar.CONFIG = Config()
                configFile.writeText(gson.toJson(Udar.CONFIG))

                Udar.LOGGER.info("Created config!")

                return
            }

            Udar.LOGGER.info("Successfully loaded config!")
            println(cfg)
            Udar.CONFIG = cfg
        } catch (e: Exception) {
            Udar.LOGGER.warning("Failed to load configuration:")
            e.printStackTrace()
            Udar.LOGGER.info("Defaulting to default config.")

            Udar.CONFIG = Config()

            return
        } finally {
            reader.close()
        }
    }
}

data class Config(
    val enabled: Boolean = true,
    val timeStep: Double = 0.005,
    val gravity: Vector3d = Vector3d(0.0, -5.0, 0.0),
    val collision: CollisionConfig = CollisionConfig(),
    val sat: SATConfig = SATConfig(),
    val sdf: SDFConfig = SDFConfig(),
    val debug: DebugConfig = DebugConfig(),
    val sleepLinearVelocity: Double = 1e-3,
    val sleepAngularVelocity: Double = 1e-3,
    val birthTime: Int = 20,
    val significant: Double = 1e-8,
) {
    companion object : InstanceCreator<Config> {
        override fun createInstance(type: Type?): Config? {
            return Config()
        }
    }

    data class CollisionConfig(
        val bias: Double = 0.15,
        val passiveSlop: Double = 0.0001,
        val activeSlop: Double = 0.001,
        val friction: Double = 0.3,
        val lambdaCarryover: Float = 0.8f,
        val normalIterations: Int = 4,
        val frictionIterations: Int = 4,
    ) {
        companion object : InstanceCreator<CollisionConfig> {
            override fun createInstance(type: Type?): CollisionConfig? {
                return CollisionConfig()
            }
        }
    }

    data class SATConfig(
        val fudge: Double = 4.0,
    ) {
        companion object : InstanceCreator<SATConfig> {
            override fun createInstance(type: Type?): SATConfig? {
                return SATConfig()
            }
        }
    }

    data class SDFConfig(
        val endFast: Boolean = false,
        val maxSteps: Int = 20,
        val maxNormalSteps: Int = 5,
        val stepSize: Double = 0.02,
        val fineStepSize: Double = 0.001,
        val epsilon: Double = 1e-7,
        val priority: Int = -1,
        val errorEpsilon: Double = 1e-14,
    ) {
        companion object : InstanceCreator<SDFConfig> {
            override fun createInstance(type: Type?): SDFConfig? {
                return SDFConfig()
            }
        }
    }

    data class DebugConfig(
        val frequency: Int = 3,
        val mesh: Int = 0,
        val normals: Int = 0,
        val collisionTimes: Int = 0,
        val data: Int = 0,

        val SDFContact: Int = 0,
        val SDFParticleCount: Int = 5,
        val SDFStartSize: Float = 0.1f,
        val SDFNodeSize: Float = 0.05f,
        val SDFDetection: Boolean = false,
        val SDFMy: Boolean = false,
        val SDFOther: Boolean = false,
        val SDFNode: Int = -1,
        val tests: Tests = Tests(mapOf()),

        val timings: Boolean = false,
    ) {
        companion object : InstanceCreator<DebugConfig> {
            override fun createInstance(type: Type?): DebugConfig? {
                return DebugConfig()
            }
        }

        data class Tests(
            val tests: Map<String, Test>,
        ) {
            data class Test(val timedCommands: Map<Int, List<String>>) {
                private var task: BukkitTask? = null

                fun run(player: Player) {
                    task?.cancel()
                    var t = 0

                    if (timedCommands.isEmpty()) return

                    task = Bukkit.getScheduler().runTaskTimer(Udar.INSTANCE, Runnable {
                        if (t > timedCommands.maxOf { it.key }) {
                            task!!.cancel()
                            return@Runnable
                        }

                        timedCommands[t]?.forEach { Bukkit.dispatchCommand(player, it) }

                        ++t
                    }, 1, 1)
                }

                fun kill() {
                    task?.cancel()
                    task = null
                }
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

object TestsAdapter : TypeAdapter<Config.DebugConfig.Tests>() {
    override fun write(out: JsonWriter, value: Config.DebugConfig.Tests?) {
        if (value == null) {
            out.nullValue()
            return
        }

        out.beginObject()

        for ((name, test) in value.tests) {
            out.name(name)
            out.beginObject()
            out.name("commands")
            out.beginObject()

            for ((time, commands) in test.timedCommands) {
                out.name(time.toString())
                if (commands.size == 1) {
                    out.value(commands[0])
                } else {
                    out.beginArray()
                    for (command in commands) {
                        out.value(command)
                    }
                    out.endArray()
                }
            }

            out.endObject()
            out.endObject()
        }

        out.endObject()
    }

    override fun read(`in`: JsonReader): Config.DebugConfig.Tests {
        val testsMap = mutableMapOf<String, Config.DebugConfig.Tests.Test>()

        `in`.beginObject()

        while (`in`.hasNext()) {
            val name = `in`.nextName()
            `in`.beginObject()

            val timedCommandsMap = mutableMapOf<Int, List<String>>()

            while (`in`.hasNext()) {
                val fieldName = `in`.nextName()
                if (fieldName == "commands") {
                    `in`.beginObject()

                    while (`in`.hasNext()) {
                        val timeStr = `in`.nextName()
                        val time = timeStr.toInt()

                        if (`in`.peek() == JsonToken.BEGIN_ARRAY) {
                            val commandsList = mutableListOf<String>()
                            `in`.beginArray()
                            while (`in`.hasNext()) {
                                commandsList.add(`in`.nextString())
                            }
                            `in`.endArray()
                            timedCommandsMap[time] = commandsList
                        } else {
                            val command = `in`.nextString()
                            timedCommandsMap[time] = listOf(command)
                        }
                    }

                    `in`.endObject()
                } else {
                    `in`.skipValue()
                }
            }

            `in`.endObject()
            testsMap[name] = Config.DebugConfig.Tests.Test(timedCommandsMap)
        }

        `in`.endObject()

        return Config.DebugConfig.Tests(testsMap)
    }
}
