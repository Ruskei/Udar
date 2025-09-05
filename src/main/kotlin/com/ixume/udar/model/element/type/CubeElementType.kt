package com.ixume.udar.model.element.type

import com.google.gson.JsonObject
import com.ixume.udar.model.element.*
import com.ixume.udar.model.element.instance.CubeElement

object CubeElementType {
    fun parse(o: JsonObject): Result<CubeElement> {
        val from = o.vec3d("from") ?: return Result.failure(InvalidJsonException("element has malformed 'from' !"))
        val to = o.vec3d("to") ?: return Result.failure(InvalidJsonException("element has malformed 'to' !"))

        val rotation = if (o.has("rotation")) {
            val rotO =
                o["rotation"]!!.obj() ?: return Result.failure(InvalidJsonException("'rotation' has to be an object!"))
            val angle = (rotO["angle"]?.asNumber()
                         ?: return Result.failure(InvalidJsonException("'rotation' has improper 'angle'"))).toDouble()
            val axis = rotO.str("axis")?.axis()
                       ?: return Result.failure(InvalidJsonException("'rotation' has improper 'axis'"))
            val origin =
                rotO.vec3d("origin") ?: return Result.failure(InvalidJsonException("'rotation' has improper 'origin'"))
            CubeElement.Rotation(
                angle = angle,
                axis = axis,
                pivot = origin,
            )
        } else null

        return Result.success(CubeElement(from, to, rotation))
    }
}