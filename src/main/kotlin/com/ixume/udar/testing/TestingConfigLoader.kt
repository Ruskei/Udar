package com.ixume.udar.testing

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.InstanceCreator
import com.google.gson.TypeAdapter
import com.google.gson.stream.JsonReader
import com.google.gson.stream.JsonWriter
import com.ixume.udar.Udar
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
    .setPrettyPrinting()
    .create()

object TestingConfigLoader {
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
    val enabled: Boolean = true,
    val timeStep: Double = 0.005,
    val gravity: Vector3d = Vector3d(0.0, -5.0, 0.0),
    val collision: CollisionConfig = CollisionConfig(),
    val sat: SATConfig = SATConfig(),
    val sdf: SDFConfig = SDFConfig(),
    val debug: DebugConfig = DebugConfig(),
) {
    companion object : com.google.gson.InstanceCreator<Config> {
        override fun createInstance(type: Type?): Config? {
            return Config()
        }
    }

    class CollisionConfig(
        val bias: Double = 0.15,
        val passiveSlop: Double = 0.0001,
        val activeSlop: Double = 0.001,
        val friction: Double = 0.3,
        val lambdaCarryover: Double = 0.3,
    ) {
        companion object : InstanceCreator<CollisionConfig> {
            override fun createInstance(type: Type?): CollisionConfig? { return CollisionConfig() }
        }
    }

    class SATConfig(
        val fudge: Double = 4.0,
    ) {
        companion object : InstanceCreator<SATConfig> {
            override fun createInstance(type: Type?): SATConfig? {
                return SATConfig()
            }
        }
    }

    class SDFConfig(
        val endFast: Boolean = false,
        val maxSteps: Int = 50,
        val maxNormalSteps: Int = 5,
        val stepSize: Double = 0.01,
        val epsilon: Double = 1e-7,
        val priority: Int = 1,
        val errorEpsilon: Double = 1e-14,
    ) {
        companion object : InstanceCreator<SDFConfig> {
            override fun createInstance(type: Type?): SDFConfig? {
                return SDFConfig()
            }
        }
    }

    class DebugConfig(
        val frequency: Int = 2,
        val mesh: Int = 0,
        val normals: Int = 0,
        val collisionTimes: Int = 0,

        val SDFContact: Int = 0,
        val SDFParticleCount: Int = 5,
        val SDFStartSize: Float = 0.1f,
        val SDFNodeSize: Float = 0.05f,
        val SDFDetection: Boolean = false,
        val SDFMy: Boolean = false,
        val SDFOther: Boolean = false,
        val SDFNode: Int = -1,
    ) {
        companion object : InstanceCreator<DebugConfig> {
            override fun createInstance(type: Type?): DebugConfig? {
                return DebugConfig()
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
