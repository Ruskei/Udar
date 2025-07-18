package com.ixume.udar

fun <T> T.applyIf(condition: Boolean, action: T.() -> Unit): T {
    if (condition) action()
    return this
}
