package com.ixume.udar.model.element

import com.google.gson.JsonArray
import com.google.gson.JsonElement
import com.google.gson.JsonObject
import com.google.gson.JsonPrimitive
import org.joml.Vector3d

class WrongFileTypeException : Exception()

class InvalidJsonException(msg: String) : Exception(msg)

fun JsonElement.obj(): JsonObject? {
    return if (isJsonObject) asJsonObject else null
}

fun JsonObject.primitive(path: String): JsonPrimitive? {
    val e = this[path] ?: return null
    return if (e.isJsonPrimitive) e.asJsonPrimitive else null
}

fun JsonObject.str(path: String): String? {
    val e = primitive(path) ?: return null
    return if (e.isString) e.asString else null
}

fun JsonElement.asStringOrNull(): String? {
    return if (isJsonPrimitive && asJsonPrimitive.isString) asString else null
}

fun JsonObject.array(path: String): JsonArray? {
    return this[path] as? JsonArray
}

fun JsonElement.asPrimitive(): JsonPrimitive? {
    return if (isJsonPrimitive) asJsonPrimitive else null
}

fun JsonElement.asNumber(): Number? {
    val p = asPrimitive() ?: return null
    return if (p.isNumber) p.asNumber else null
}

fun JsonObject.vec3d(path: String): Vector3d? {
    val toArr = array(path) ?: return null
    if (toArr.size() != 3) return null
    return Vector3d(
        (toArr.get(0).asNumber()
            ?: return null).toDouble(),
        (toArr.get(1).asNumber()
            ?: return null).toDouble(),
        (toArr.get(2).asNumber()
            ?: return null).toDouble(),
    )
}

enum class Axis {
    X, Y, Z
}

fun String.axis(): Axis? {
    return when(this) {
        "x" -> Axis.X
        "y" -> Axis.Y
        "z" -> Axis.Z
        else -> null
    }
}
