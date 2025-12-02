package com.ixume.udar.body.active.tag

class Tag(
    val name: String,
    var collide: Boolean,
) {
    override fun equals(other: Any?): Boolean {
        return other != null && other is Tag && other.name == name
    }

    override fun hashCode(): Int {
        return name.hashCode()
    }
}