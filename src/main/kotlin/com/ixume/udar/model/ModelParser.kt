package com.ixume.udar.model

import com.google.gson.*
import com.ixume.udar.Udar
import com.ixume.udar.model.element.InvalidJsonException
import com.ixume.udar.model.element.WrongFileTypeException
import com.ixume.udar.model.element.array
import com.ixume.udar.model.element.instance.ModelElement
import com.ixume.udar.model.element.obj
import com.ixume.udar.model.element.str
import com.ixume.udar.model.element.type.CubeElementType
import java.io.File
import java.io.FileNotFoundException
import java.io.FileReader

object BBParser {
    fun load() {
        val dataFolder = Udar.INSTANCE.dataFolder
        dataFolder.mkdirs()

        val modelsFolder = File(dataFolder, "models")
        modelsFolder.mkdirs()

        if (!modelsFolder.exists() || !modelsFolder.isDirectory) {
            Udar.MODELS = emptyList()
            return
        }

        val models = mutableListOf<JavaModel>()

        for (file in modelsFolder.listFiles()) {
            parse(file).fold(
                { models += it },
                { it.printStackTrace() }
            )
        }

        Udar.MODELS = models
    }

    private fun parse(file: File): Result<JavaModel> {
        if (!file.exists()) return Result.failure(FileNotFoundException())
        if (file.extension != "json") return Result.failure(WrongFileTypeException())

        val reader = FileReader(file)

        try {
            val jsonRoot =
                JsonParser.parseReader(reader).obj() ?: return Result.failure(InvalidJsonException("Missing root!"))
            val name = file.nameWithoutExtension.replace(" ", "")
            val elemArr =
                jsonRoot.array("elements") ?: return Result.failure(InvalidJsonException("Missing 'elements' !"))

            val elems = mutableListOf<ModelElement>()

            for (e in elemArr) {
                val o = e.obj() ?: return Result.failure(InvalidJsonException("Elements is not an object array!"))
                CubeElementType.parse(o).fold(
                    { elems += it },
                    { return Result.failure(it) }
                )
            }

            return Result.success(JavaModel(name, elems))
        } catch (jsonParseException: JsonParseException) {
            return Result.failure(jsonParseException)
        } finally {
            reader.close()
        }
    }
}