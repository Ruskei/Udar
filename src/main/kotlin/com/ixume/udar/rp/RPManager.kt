package com.ixume.udar.rp

import com.google.gson.GsonBuilder
import com.google.gson.JsonArray
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import com.ixume.udar.Udar
import com.ixume.udar.model.JavaModel
import com.ixume.udar.model.ModelParser
import com.ixume.udar.model.element.asStringOrNull
import org.bukkit.Material
import org.bukkit.inventory.ItemStack
import java.io.File
import java.io.FileReader

object RPManager {
    private const val RP_NAME = "Udar"
    private const val RP_IMAGE = "pack.png"
    private const val RP_META = "pack.mcmeta"
    private val ITEM_TYPE = Material.TROPICAL_FISH
    private val ITEM = ITEM_TYPE.key.value()
    const val NAMESPACE = "p"

    val modelMap: MutableMap<String, JavaModel> = mutableMapOf()

    fun item(id: String): ItemStack? {
        val md = modelMap[id] ?: return null
        return createItem(md.index)
    }

    private fun createItem(index: Int): ItemStack {
        val stack = ItemStack(ITEM_TYPE)
        stack.editMeta { meta ->
            meta.setCustomModelData(index)
        }

        return stack
    }

    fun createResources() {
        val dataFolder = Udar.INSTANCE.dataFolder
        dataFolder.mkdirs()

        val tempDir = File(dataFolder, ".temp/")
        tempDir.deleteRecursively()

        val outputDir = File(dataFolder.path, "output")
        if (outputDir.exists()) {
            if (outputDir.isDirectory) {
                outputDir.deleteRecursively()
            } else {
                outputDir.delete()
            }
        }

        val modelFolder = File(dataFolder.path, "models")

        if (modelFolder.exists()) {
            val models: MutableList<File> = mutableListOf()
            for (file in modelFolder.listFiles()!!) {
                if (file.isDirectory) models += file
            }

            genModels(models)
        }

        val packImage = File(dataFolder.path, "output/$RP_NAME/pack.png")
        packImage.parentFile.mkdirs()
        packImage.writeBytes(Udar.INSTANCE.getResource(RP_IMAGE)!!.readAllBytes())

        val packMeta = File(dataFolder.path, "output/$RP_NAME/pack.mcmeta")
        packMeta.writeBytes(Udar.INSTANCE.getResource(RP_META)!!.readAllBytes())

    }

    private fun genModels(files: List<File>) {
        val dataFolder = Udar.INSTANCE.dataFolder

        val itemFolder = File(dataFolder.path + "/output/$RP_NAME/assets/minecraft/items")
        itemFolder.mkdirs()

        val modelItemFolder = File(dataFolder.path + "/output/$RP_NAME/assets/minecraft/models/item")
        modelItemFolder.mkdirs()

        val modelFolder = File(dataFolder.path + "/output/$RP_NAME/assets/minecraft/models/item/$NAMESPACE")
        modelFolder.mkdirs()

        val texturesFolder = File(dataFolder.path + "/output/$RP_NAME/assets/minecraft/textures/item/$NAMESPACE")
        texturesFolder.mkdirs()

        val gson = GsonBuilder().setPrettyPrinting().create()

        val texturesObj = JsonObject()

        val itemObj = JsonObject()

        val mObj = JsonObject()

        mObj.addProperty("type", "range_dispatch")
        mObj.addProperty("property", "custom_model_data")

        val fallback = JsonObject()
        fallback.addProperty("type", "model")
        fallback.addProperty("model", "item/$ITEM")

        mObj.add("fallback", fallback)

        val entries = JsonArray()

        mObj.add("entries", entries)

        itemObj.add("model", mObj)

        val tints = JsonArray()

        val modelDataTint = JsonObject()
        modelDataTint.addProperty("type", "custom_model_data")
        modelDataTint.addProperty("index", 0)
        modelDataTint.addProperty("default", 16777215)

        tints.add(modelDataTint)

        val modelItemsObj = JsonObject()
        modelItemsObj.addProperty("parent", "item/generated")
        texturesObj.addProperty("layer0", "item/$ITEM")
        modelItemsObj.add("textures", texturesObj)

        val overridesArr = JsonArray()

        var count = 100

        for (file in files) {
            val model = File(file.path + "/" + file.nameWithoutExtension + ".json")
            if (!model.exists()) continue

            val newModel = File(modelFolder.path + "/" + model.name)
            model.copyTo(newModel)

            val rootObject = JsonParser.parseReader(FileReader(newModel)).asJsonObject
            val texturesObject = if (rootObject.has("textures")) rootObject.getAsJsonObject("textures") else null
            if (texturesObject != null) {
                val newTexturesObj = JsonObject()

                for ((key, value) in texturesObject.entrySet()) {
                    val asStr = value.asStringOrNull() ?: continue
                    newTexturesObj.addProperty(key, "item/$NAMESPACE/$asStr")

                    val tex = File(file.path + "/" + asStr + ".png")
                    if (tex.exists()) {
                        val newFileLoc = File(texturesFolder.path + "/" + asStr + ".png")
                        if (!newFileLoc.exists()) {
                            tex.copyTo(newFileLoc)
                        }
                    }
                }

                rootObject.remove("textures")
                rootObject.add("textures", newTexturesObj)
            }

            newModel.writeText(gson.toJson(rootObject))

            val override = JsonObject()
            val predicate = JsonObject()
            val index = count++

            val stack = ItemStack(ITEM_TYPE)
            stack.editMeta { meta ->
                meta.setCustomModelData(index)
            }

            val iObj = JsonObject()
            iObj.addProperty("threshold", index)

            val miObj = JsonObject()

            miObj.addProperty("type", "model")
            miObj.addProperty("model", "item/$NAMESPACE/${file.nameWithoutExtension}")
            miObj.add("tints", tints)

            iObj.add("model", miObj)

            entries.add(iObj)

            val elems = ModelParser.parse(model).fold({ it }, {
                it.printStackTrace()
                null
            }) ?: continue

            modelMap += file.nameWithoutExtension to JavaModel(
                id = model.nameWithoutExtension,
                elems = elems,
                index = index
            )

            predicate.addProperty("custom_model_data", index)
            override.add("predicate", predicate)
            override.addProperty("model", "item/$NAMESPACE/${file.nameWithoutExtension}")
            overridesArr.add(override)
        }

        modelItemsObj.add("overrides", overridesArr)

        val modelItemTarget = File(modelItemFolder.path, "$ITEM.json")
        modelItemTarget.writeText(gson.toJson(modelItemsObj))

        val itemTarget = File(itemFolder.path, "$ITEM.json")
        itemTarget.writeText(gson.toJson(itemObj))
    }
}
