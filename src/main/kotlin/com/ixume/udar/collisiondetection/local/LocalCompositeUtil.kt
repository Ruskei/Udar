package com.ixume.udar.collisiondetection.local

import com.ixume.udar.physics.contact.A2AContactArray
import com.ixume.udar.physics.contact.A2SContactArray
import com.ixume.udar.physics.contact.A2SManifoldArray

class LocalCompositeUtil {
    val buf = A2AContactArray()
    val envContactBuf = A2SManifoldArray(8)
}