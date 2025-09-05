package com.ixume.udar.collisiondetection.local

import com.ixume.udar.physics.contact.A2AContactArray
import com.ixume.udar.physics.contact.A2SContactArray

class LocalCompositeUtil {
    val buf = A2AContactArray()
    val envBuf = A2SContactArray()
}