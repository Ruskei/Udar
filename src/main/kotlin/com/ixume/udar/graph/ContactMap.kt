package com.ixume.udar.graph

import com.ixume.udar.PhysicsWorld
import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.physics.Contact
import com.ixume.udar.physics.constraint.*
import org.joml.Vector3f
import java.nio.FloatBuffer
import kotlin.math.abs

class ContactMap(
    val physicsWorld: PhysicsWorld
) {
    lateinit var idConstraintMap: Array<Contact>
    lateinit var flatConstraintData: FloatArray

    lateinit var envIDConstraintMap: Array<Contact>
    lateinit var envConstraintData: FloatArray

    fun process() {
        constructFlatConstraintData()
        constructFlatEnvConstraintData()
    }

    private val _c_vec3 = Vector3f()
    private val _c_j0 = Vector3f()
    private val _c_j1 = Vector3f()
    private val _c_j1Temp = Vector3f()
    private val _c_j2 = Vector3f()
    private val _c_j3 = Vector3f()
    private val _c_j3Temp = Vector3f()
    private val _c_im = Vector3f()

    private fun constructFlatConstraintData() {
        val ls = physicsWorld.contacts.toList() as List<Contact>

        idConstraintMap = Array(ls.size) { ls[it] }

        val numc = idConstraintMap.size
        flatConstraintData = FloatArray(numc * A2A_CONTACT_DATA_FLOATS)
        val n = FloatBuffer.wrap(flatConstraintData)
        var i = 0
        while (i < numc) {
            val contact = idConstraintMap[i]

            n.putVector3f(_c_vec3.set(contact.result.norm))

            n.putVector3f(
                _c_j1
                    .set(contact.first.pos)
                    .sub(_c_vec3.set(contact.result.point))
                    .cross(_c_vec3.set(contact.result.norm))
            ) //j1

            n.putVector3f(
                _c_j3
                    .set(contact.result.point)
                    .sub(_c_vec3.set(contact.second.pos))
                    .cross(_c_vec3.set(contact.result.norm))
            ) //j3

            val timestep = 0.005f
            val slop = 0.0001f

            n.put(0.1f / timestep * (abs(contact.result.depth.toFloat()) - slop).coerceAtLeast(0f)) // bias

            n.put(
                _c_j0.set(
                    contact.result.norm.x * contact.result.norm.x,
                    contact.result.norm.y * contact.result.norm.y,
                    contact.result.norm.z * contact.result.norm.z
                ).dot(
                    contact.first.inverseMass.toFloat(),
                    contact.first.inverseMass.toFloat(),
                    contact.first.inverseMass.toFloat(),
                ) +
                        _c_j1Temp.set(_c_j1).mul(contact.first.inverseInertia).dot(_c_j1) +
                        _c_j2.set(
                            contact.result.norm.x * contact.result.norm.x,
                            contact.result.norm.y * contact.result.norm.y,
                            contact.result.norm.z * contact.result.norm.z
                        ).dot(
                            contact.second.inverseMass.toFloat(),
                            contact.second.inverseMass.toFloat(),
                            contact.second.inverseMass.toFloat(),
                        ) +
                        _c_j3Temp.set(_c_j3).mul(contact.second.inverseInertia).dot(_c_j3)
            ) // den

            n.put(contact.lambdaSum.toFloat()) //lambda

            n.putVector3f(_c_im.set(contact.first.inverseMass).mul(_c_vec3.set(contact.result.norm).negate())) // DVA
            n.putVector3f(_c_j1Temp.set(_c_j1).mul(contact.first.inverseInertia)) // DOA
            n.putVector3f(_c_im.set(contact.second.inverseMass).mul(_c_vec3.set(contact.result.norm))) // DVB
            n.putVector3f(_c_j3Temp.set(_c_j3).mul(contact.second.inverseInertia)) // DOB

            n.put(Float.fromBits((contact.first as ActiveBody).id)) // my id
            n.put(Float.fromBits((contact.second as ActiveBody).id)) // other id


            i++
        }
    }

    private fun constructFlatEnvConstraintData() {
        val ls = physicsWorld.envContacts
        envIDConstraintMap = Array(ls.size) { ls[it] }

        val numc = envIDConstraintMap.size
        envConstraintData = FloatArray(numc * A2S_CONTACT_DATA_FLOATS)
        val n = FloatBuffer.wrap(envConstraintData)
        var i = 0
        while (i < numc) {
            val contact = envIDConstraintMap[i]

            n.putVector3f(_c_vec3.set(contact.result.norm)) // norm

            n.putVector3f(
                _c_j1
                    .set(contact.first.pos)
                    .sub(_c_vec3.set(contact.result.point))
                    .cross(_c_vec3.set(contact.result.norm))
            ) //j1

            val timestep = 0.005f
            val slop = 0.0001f

            n.put(0.1f / timestep * (abs(contact.result.depth.toFloat()) - slop).coerceAtLeast(0f)) // bias

            n.put(
                _c_j0.set(
                    contact.result.norm.x * contact.result.norm.x,
                    contact.result.norm.y * contact.result.norm.y,
                    contact.result.norm.z * contact.result.norm.z
                ).dot(
                    contact.first.inverseMass.toFloat(),
                    contact.first.inverseMass.toFloat(),
                    contact.first.inverseMass.toFloat(),
                ) +
                        _c_j1Temp.set(_c_j1).mul(contact.first.inverseInertia).dot(_c_j1) +
                        _c_j2.set(
                            contact.result.norm.x * contact.result.norm.x,
                            contact.result.norm.y * contact.result.norm.y,
                            contact.result.norm.z * contact.result.norm.z
                        ).dot(
                            contact.second.inverseMass.toFloat(),
                            contact.second.inverseMass.toFloat(),
                            contact.second.inverseMass.toFloat(),
                        ) +
                        _c_j3Temp.set(_c_j3).mul(contact.second.inverseInertia).dot(_c_j3)
            ) // den

            n.put(contact.lambdaSum.toFloat()) //lambda

            n.putVector3f(_c_im.set(contact.first.inverseMass).mul(_c_vec3.set(contact.result.norm).negate())) // DVA
            n.putVector3f(_c_j1Temp.set(_c_j1).mul(contact.first.inverseInertia)) // DOA

            n.put(Float.fromBits((contact.first as ActiveBody).id)) // my id

            i++
        }
    }
}