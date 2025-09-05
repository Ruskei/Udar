package com.ixume.udar.body.active

interface Composite : ActiveBody {
    val parts: List<ActiveBody>
}