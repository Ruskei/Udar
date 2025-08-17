package com.ixume.udar.graph

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.Contact
import kotlin.math.max

class GraphUtil(
    val physicsWorld: PhysicsWorld
) {
    lateinit var idConstraintMap: Array<Contact>

    lateinit var constraintBitGraph: LongArray //maps from contact.id (index) to a flattened bit set
    var necessaryLongs = 1
    var coloredContacts = IntArray(0) //maps from contact.id (index) to color

    //for use in solving, create a bitset for each color and iterate through the bits;
    //this means iteration might be slower, but it's probably not a big deal, since alternative is allocating creating lists, while this only requires bitsets
    //this can be represented as a flattened LongArray, if we have N colors then the bitset for color n can be accessed with [n * necessaryLongs]..[n * necessaryLongs + necessaryLongs]
    //resize if more colors needed
    var colorSet = LongArray(1)

    var maxColor = 0

    var contactsSize = 0
    private var colorArr = LongArray(1)

    fun process() {
        maxColor = 0
        contactsSize = physicsWorld.contacts.size
        necessaryLongs = (contactsSize + 63) / 64
        constructIDMap()
        populateConstraintEdges()
        bitwiseGreedy()
//        checkInvariants()
    }

    private fun constructIDMap() {
        val ls = physicsWorld.contacts.toList() as List<Contact>
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

        idConstraintMap = Array(contactsSize) { sorted[it] }
    }

    private fun populateConstraintEdges() {
        val cs = physicsWorld.contacts.toList()


        val bitGraph = LongArray(contactsSize * necessaryLongs)

        var i = 0

        val s = contactsSize

        while (i < s) {
            val contact = cs[i]
            val firstIncident = (contact.first as ActiveBody).contactIDs
            val secondIncident = (contact.second as ActiveBody).contactIDs

            var j = 0
            while (j < necessaryLongs) {
                val addon = firstIncident.getOrElse(j) { 0 } or secondIncident.getOrElse(j) { 0 }
                if (addon == 0L) {
                    j++
                    continue
                }

                //go through all the set bits and make the edges undirected
                var k = addon.countTrailingZeroBits()
                while (k < 64) {
                    //j * 64 + k == other id
                    bitGraph[(j * 64 + k) * necessaryLongs + (i shr 6)] = bitGraph[(j * 64 + k) * necessaryLongs + (i shr 6)] or (1L shl i)

                    k += ((addon shr (k + 1)).countTrailingZeroBits() + 1)
                }

                val v = bitGraph[i * necessaryLongs + j] or addon

                bitGraph[i * necessaryLongs + j] = v
                j++
            }

            bitGraph[i * necessaryLongs + (i shr 6)] = (bitGraph[i * necessaryLongs + (i shr 6)] and ((1L shl i).inv()))

//            check(((bitGraph[i * necessaryLongs + (i shr 6)] shr i) and 1) == 0L)

            i++
        }

        constraintBitGraph = bitGraph
    }

    private fun bitwiseGreedy() {
        coloredContacts = IntArray(contactsSize) { -1 }

        var n = 0
        while (n < colorSet.size) {
            colorSet[n] = 0L

            n++
        }

        var i = contactsSize - 1 // current node, so `i` is currentID
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
                    val otherID = j * 64 + k
                    //at a set bit! k is index in current long, since we're indexing total ID here, we need to add (j * 64)
                    //not all colorings are yet added, so coloredContacts could return -1!
                    val color = coloredContacts[otherID]
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

            if (freeColor == -1) {
                //ran out of colors, make more space!
                freeColor = colorArr.size * 64
                colorArr = colorArr.copyOf(colorArr.size + 1)
            }

            check(freeColor != -1)

            coloredContacts[i] = freeColor
            maxColor = max(freeColor, maxColor)

            if (colorSet.size < ((freeColor + 1) * necessaryLongs + (i shr 6))) {
                colorSet = colorSet.copyOf(colorSet.size * 2)
            }

            colorSet[freeColor * necessaryLongs + (i shr 6)] = (colorSet[freeColor * necessaryLongs + (i shr 6)] or (1L shl i))

            i--
        }
    }

    private fun checkInvariants() {
        var i = 0
        while (i < contactsSize) {
            val myColor = coloredContacts[i]
            check(myColor != -1)
            check((colorSet[myColor * necessaryLongs + (i shr 6)] shr i) and 1 == 1L)

            var j = 0
            while (j < necessaryLongs) {
                val l = constraintBitGraph[i * necessaryLongs + j]
                var k = l.countTrailingZeroBits()
                if (k >= 64) {
                    j++
                    continue
                }

                while (k < 64) {
                    val otherID = j * 64 + k
                    check((constraintBitGraph[otherID * necessaryLongs + (i shr 6)] shr i) and 1 == 1L) //check that the matching node has ME set as well (edges should not be directed)
                    val color = coloredContacts[otherID]
                    check(myColor != color) { "Failed! i: $i j64k: ${j * 64 + k} myColor: $myColor color: $color l: $l" }
                    k += ((l shr (k + 1)).countTrailingZeroBits() + 1)
                }

                j++
            }

            i++
        }
    }
}