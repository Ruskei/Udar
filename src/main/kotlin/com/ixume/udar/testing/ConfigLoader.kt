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
            configFile.writeText(gson.toJson(Udar.CONFIG))

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
    val timeStep: Double = 0.01,
    val gravity: Vector3d = Vector3d(0.0, -10.0, 0.0),
    val collision: CollisionConfig = CollisionConfig(),
    val sphericalJoint: SphericalJointConfig = SphericalJointConfig(),
    val debug: DebugConfig = DebugConfig(),
    val sleepLinearVelocity: Double = 1e-3,
    val sleepAngularVelocity: Double = 1e-3,
    val sleepTime: Int = 40,
    val birthTime: Int = 40,

    val worldDiffingProcessors: Int = 5,
    val meshingProcessors: Int = 5,
    val narrowPhaseProcessors: Int = 5,
    val envPhaseProcessors: Int = 7,
) {
    companion object : InstanceCreator<Config> {
        override fun createInstance(type: Type?): Config? {
            return Config()
        }
    }

    data class CollisionConfig(
        val bias: Double = 0.2,
        val passiveSlop: Double = 1e-3,
        val activeSlop: Double = 1e-3,
        val friction: Double = 0.35,
        val lambdaCarryover: Float = 0.99f,
        val normalIterations: Int = 6,
        val frictionIterations: Int = 3,
        val sameContactThreshold: Double = 0.5,
    ) {
        companion object : InstanceCreator<CollisionConfig> {
            override fun createInstance(type: Type?): CollisionConfig? {
                return CollisionConfig()
            }
        }
    }
    
    data class SphericalJointConfig(
        val bias: Double = 0.2,
        val slop: Double = 1e-3,
        val iterations: Int = 6,
        val lambdaCarryover: Float = 0.99f,
    ) {
        companion object : InstanceCreator<SphericalJointConfig> {
            override fun createInstance(type: Type?): SphericalJointConfig? {
                return SphericalJointConfig()
            }
        }
    }

    data class DebugConfig(
        val frequency: Int = 2,
        val mesh: Int = 0,
        val normals: Int = 0,
        val bbs: Boolean = false,
        val collisionTimes: Int = 0,
        val data: Int = 0,

        val tests: Tests = Tests.DEFAULT_TESTS,

        val timings: Boolean = false,
        val timingsSimReportInterval: Int = 500,
        val timingsWorldReportInterval: Int = 40,
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

            companion object {
                val DEFAULT_TESTS = Tests(
                    mapOf(
                        "stress-grid" to Test(
                            mapOf(
                                10 to listOf("udar stress-grid"),
                                20 to listOf("udar stress-grid"),
                                30 to listOf("udar stress-grid"),
                                40 to listOf("udar stress-grid"),
                                50 to listOf("udar stress-grid"),
                                60 to listOf("udar stress-grid"),
                                70 to listOf("udar stress-grid"),
                                80 to listOf("udar stress-grid"),
                                90 to listOf("udar stress-grid"),
                                100 to listOf("udar stress-grid"),
                            )
                        )
                    )
                )
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
