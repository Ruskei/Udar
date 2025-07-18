package com.ixume.udar.collisiondetection.contactgeneration

import com.ixume.udar.physics.CollisionResult

interface ContactGenerator {
    fun collidesEnvironment(): List<CollisionResult>
}