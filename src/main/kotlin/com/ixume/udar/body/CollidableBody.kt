package com.ixume.udar.body

import com.ixume.udar.physics.contact.Contact

interface CollidableBody : Body, Collidable {
    val contacts: MutableList<Contact>
    val previousContacts: List<Contact>
}