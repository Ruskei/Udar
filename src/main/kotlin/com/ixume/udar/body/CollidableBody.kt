package com.ixume.udar.body

import com.ixume.udar.physics.Contact

interface CollidableBody : Body, Collidable {
    val contacts: MutableList<Contact>
    var contactIDs: LongArray
    val previousContacts: List<Contact>
}