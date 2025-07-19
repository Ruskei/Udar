package com.ixume.udar.body

import com.ixume.udar.physics.IContact

interface CollidableBody : Body, Collidable {
    val contacts: MutableList<IContact>
    val previousContacts: List<IContact>
}