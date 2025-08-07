package com.ixume.udar.body.active

import com.ixume.udar.Udar
import org.joml.Quaterniond
import org.joml.Vector3d

class RigidbodyRotationIntegrator(
    val body: ActiveBody
) {
    private fun calcDerivatives(
        q: Quaterniond,
        o: Vector3d,
        outDQ: Quaterniond,
        outDO: Vector3d,
    ) {
        outDQ.set(q).mul(o.x, o.y, o.z, 0.0).mul(0.5)

        outDO.set(
            (body.localInertia.y - body.localInertia.z) / body.localInertia.x * o.y * o.z + body.torque.x / body.localInertia.x,
            (body.localInertia.z - body.localInertia.x) / body.localInertia.y * o.z * o.x + body.torque.y / body.localInertia.y,
            (body.localInertia.x - body.localInertia.y) / body.localInertia.z * o.x * o.y + body.torque.z / body.localInertia.z,
        )
    }

    private val _dK1Q = Quaterniond()
    private val _dK1O = Vector3d()

    private val _dK2Q = Quaterniond()
    private val _dK2O = Vector3d()

    private val _dK3Q = Quaterniond()
    private val _dK3O = Vector3d()

    private val _dK4Q = Quaterniond()
    private val _dK4O = Vector3d()

    fun process() {
        val h = Udar.CONFIG.timeStep
        val h2 = h / 2.0

        calcDerivatives(body.q, body.omega, _dK1Q, _dK1O)

        _dK2Q.set(body.q).add(_dK1Q.x * h2, _dK1Q.y * h2, _dK1Q.z * h2, _dK1Q.w * h2).normalize()
        _dK2O.set(body.omega).add(_dK1O.x * h2, _dK1O.y * h2, _dK1O.z * h2)
        calcDerivatives(_dK2Q, _dK2O, _dK2Q, _dK2O)

        _dK3Q.set(body.q).add(_dK2Q.x * h2, _dK2Q.y * h2, _dK2Q.z * h2, _dK2Q.w * h2).normalize()
        _dK3O.set(body.omega).add(_dK2O.x * h2, _dK2O.y * h2, _dK2O.z * h2)
        calcDerivatives(_dK3Q, _dK3O, _dK3Q, _dK3O)

        _dK4Q.set(body.q).add(_dK2Q.x * h, _dK2Q.y * h, _dK2Q.z * h, _dK2Q.w * h).normalize()
        _dK4O.set(body.omega).add(_dK2O.x * h, _dK2O.y * h, _dK2O.z * h)
        calcDerivatives(_dK4Q, _dK4O, _dK4Q, _dK4O)

        val fDO = _dK1O
            .add(_dK2O.mul(2.0))
            .add(_dK3O.mul(2.0))
            .add(_dK4O)
            .mul(h / 6.0)

        val fDQ = _dK1Q
            .add(_dK2Q.scale(2.0))
            .add(_dK3Q.scale(2.0))
            .add(_dK4Q)
            .mul(h / 6.0)

        body.omega.add(fDO)

        body.q.add(fDQ)
        body.q.normalize()

        body.torque.set(0.0)
    }
}