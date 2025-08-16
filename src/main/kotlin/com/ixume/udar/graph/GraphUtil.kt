package com.ixume.udar.graph

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.Contact

class GraphUtil(
    val physicsWorld: PhysicsWorld
) {
    lateinit var idContactMap: Array<Contact>

    lateinit var constraintBitGraph: LongArray //maps from contact.id (index) to a flattened bit set
    private var necessaryLongs = 1
    var coloredContacts = IntArray(0) //maps from contact.id (index) to color
    var contactsSize = 0
    private var colorArr = LongArray(1)

    private var maxColor = 0

    fun process() {
        contactsSize = physicsWorld.contacts.size
        necessaryLongs = (contactsSize + 63) / 64
        constructIDMap()
        populateConstraintEdges()
        bitwiseGreedy()
        val m = coloredContacts.maxOrNull()
        if (m != null && m > maxColor) {
            println("MAX: $m")
            maxColor = m
        }
    }

    private fun constructIDMap() {
        val ls = physicsWorld.contacts.toList()
        val sorted = ls.sortedBy { it.id }
//        if (sorted.isNotEmpty()) {
//            check(sorted.last().id == contactsSize - 1)
//            var j = 0
//            while (j < sorted.size) {
//                check(sorted[j].id == j)
//
//                j++
//            }
//        }

        idContactMap = Array(contactsSize) { sorted[it] }
    }

    private fun populateConstraintEdges() {
        val cs = physicsWorld.contacts.toList()

        constraintBitGraph = LongArray(contactsSize * necessaryLongs)
        var i = 0

        val s = contactsSize

        while (i < s) {
            val contact = cs[i]
            val firstIncident = (contact.first as ActiveBody).contactIDs
            val firstReceiver = physicsWorld.secondaryContactMap[contact.first]
            val secondIncident = (contact.second as ActiveBody).contactIDs
            val secondReceiver = physicsWorld.secondaryContactMap[contact.second]

            var j = 0
            while (j < necessaryLongs) {
                var v = constraintBitGraph[i * necessaryLongs + j]
                    .or(firstIncident.getOrElse(j) { 0 })
                    .or(secondIncident.getOrElse(j) { 0 })

                if (firstReceiver != null) {
                    v = v or (firstReceiver.getOrElse(j) { 0 })
                }

                if (secondReceiver != null) {
                    v = v or (secondReceiver.getOrElse(j) { 0 })
                }

                constraintBitGraph[i * necessaryLongs + j] = v

                j++
            }

            i++
        }
    }

    private fun bitwiseGreedy() {
        coloredContacts = IntArray(contactsSize) { -1 }

        var i = contactsSize - 1 // current node, so this is currentID
        while (i >= 0) {
            var freeColor = -1

            var y = 0
            while (y < colorArr.size) {
                colorArr[y] = 0L
                y++
            }

            var j = 0
            while (j < necessaryLongs) {
                val l = constraintBitGraph[i * necessaryLongs + j]
                //an arbitrary node j, its connections are represented by the bits of constraintBitGraph[(i * necessaryLongs)..<((i + 1)( * necessaryLongs)
                //if a bit is set at one of those positions, then i is connected to j
                //so here we should iterate through all the bits and for all set bits J with indices


                // 0b00011001000
                // since insertion is like this:
                // words[wordIndex] |= (1L << bitIndex)
                // higher bit indices are in more significant
                var k = l.countTrailingZeroBits()
                if (k >= 64) {
                    //no set bits
                    j++
                    continue
                }

                while (k < 64) {
                    //at a set bit! k is index in current long, since we're indexing total ID here, we need to add (j * 64)
                    //not all colorings are yet added, so coloredContacts could return -1!
                    val color = coloredContacts[j * 64 + k]
                    if (color != -1) {
                        if (color > colorArr.size * 64) {
                            colorArr = colorArr.copyOf((color + 63) / 64)
                        }

                        colorArr[color shr 6] = (colorArr[color shr 6] or (1L shl color))
                    }

                    k += ((l shr (k + 1)).countTrailingZeroBits() + 1)
                }

                j++
            }

            //find first available color; this means finding first zero bit in colorArr
            var b = 0
            while (b < colorArr.size) {
                val ent = colorArr[b]
                if (ent == 0L) {
                    freeColor = 0
                    //can just set at first bit
                    break
                }

                val trailing = ent.inv().countTrailingZeroBits() //if this == 64, then no free bits here; else, we've found our bit
                if (trailing == 64) {
                    b++
                    continue
                }

                freeColor = trailing
                break
            }

            check(freeColor != -1)

            coloredContacts[i] = freeColor

            i--
        }
    }
}