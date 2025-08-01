package com.ixume.udar.body.active

import com.ixume.udar.Udar
import org.joml.Quaterniond
import org.joml.Vector3d

class RigidbodyRotationIntegrator(
    val body: ActiveBody
) {
    private fun calcDerivatives(
        q: Quaterniond,
        o: Vector3d
    ): Pair<Quaterniond, Vector3d> {
        val dO = Vector3d(
            (body.localInertia.y - body.localInertia.z) / body.localInertia.x * o.y * o.z + body.torque.x / body.localInertia.x,
            (body.localInertia.z - body.localInertia.x) / body.localInertia.y * o.z * o.x + body.torque.y / body.localInertia.y,
            (body.localInertia.x - body.localInertia.y) / body.localInertia.z * o.x * o.y + body.torque.z / body.localInertia.z,
        )

        val dQ = Quaterniond(q).mul(Quaterniond(o.x, o.y, o.z, 0.0)).mul(0.5)

        return dQ to dO
    }

    fun process() {
        val h2 = Udar.CONFIG.timeStep / 2.0

        val (dK1Q, dK1O) = calcDerivatives(body.q, body.omega)

        val k2Q = Quaterniond(body.q).add(Quaterniond(dK1Q).scale(h2)).normalize()
        val k2O = Vector3d(body.omega).add(Vector3d(dK1O).mul(h2))
        val (dK2Q, dK2O) = calcDerivatives(k2Q, k2O)

        val k3Q = Quaterniond(body.q).add(Quaterniond(dK2Q).scale(h2)).normalize()
        val k3O = Vector3d(body.omega).add(Vector3d(dK2O).mul(h2))
        val (dK3Q, dK3O) = calcDerivatives(k3Q, k3O)

        val k4Q = Quaterniond(body.q).add(Quaterniond(dK3Q).scale(Udar.CONFIG.timeStep)).normalize()
        val k4O = Vector3d(body.omega).add(Vector3d(dK3O).mul(Udar.CONFIG.timeStep))
        val (dK4Q, dK4O) = calcDerivatives(k4Q, k4O)

        val fDO = Vector3d(dK1O)
            .add(Vector3d(dK2O).mul(2.0))
            .add(Vector3d(dK3O).mul(2.0))
            .add(dK4O)
            .mul(Udar.CONFIG.timeStep / 6.0)

        val fDQ = Quaterniond(dK1Q)
            .add(Quaterniond(dK2Q).scale(2.0))
            .add(Quaterniond(dK3Q).scale(2.0))
            .add(dK4Q)
            .mul(Udar.CONFIG.timeStep / 6.0)

        body.omega.add(fDO)

        body.q.add(fDQ)
        body.q.normalize()

        body.torque.set(0.0)
    }
}