package com.ixume.udar.collisiondetection.local

import com.ixume.udar.physics.contact.A2AManifoldArray
import com.ixume.udar.physics.contact.A2SManifoldArray

class LocalCompositeUtil {
    val buf = A2AManifoldArray(8)
    val envContactBuf = A2SManifoldArray(8)
}