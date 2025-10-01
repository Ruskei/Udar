package com.ixume.udar.collisiondetection.local

import com.ixume.udar.physics.contact.a2a.manifold.A2AManifoldArray
import com.ixume.udar.physics.contact.a2s.manifold.A2SManifoldArray

class LocalCompositeUtil {
    val buf = A2AManifoldArray(4)
    val envContactBuf = A2SManifoldArray(8)
}