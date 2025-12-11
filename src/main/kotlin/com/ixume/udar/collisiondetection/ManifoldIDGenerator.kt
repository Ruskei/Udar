package com.ixume.udar.collisiondetection

import com.ixume.udar.body.active.ActiveBody
import com.ixume.udar.collisiondetection.local.LocalEnvContactUtil
import com.ixume.udar.collisiondetection.local.LocalMathUtil
import com.ixume.udar.collisiondetection.mesh.mesh2.LocalMesher

/*
types of manifolds:
a2s - face
a2s - [ee, fe]

a2s
    0-32: (value ^ (value >>> 32)) (body)
    33-34: manifold type
    35-40: mesh

a2a - face
a2a - edge

a2a
    let's do 2 bits for manifold type just in case...
    both need to store 2 ints, let's just give 16 bits for that
    so 46 bits for 2 bodies: (a ^ b) & ((1L << 46) - 1);
    
    1-2: manifold type
    
    face
        49-56: a & 0b11111111
        57-64: b & 0b11111111
    edge
        49-56: a & 0b11111111
        57-64: b & 0b11111111

uuid is 128 bits... so use body id instead (it's a long but we should never have that many bodies at once
we know some of these things are always separate, we have 3 types of manifolds. let's use 2 bits for that, then have another 2 bits for misc
so we have *24* bits left for each manifold type

a2s
    face
        axis (2 bits)
        level (64 bits)
            get this into a proportion of the mesh's min and max, discretize, mask
        ID:
        41-42: axis
        43-64: (level - min) / (MESH_SIZE / (2^24 - 1))
    edge
        ee
            env edges are defined by their axis, a, b, i (2 doubles, 1 int, axis)
                16 bits for 2 doubles and 1 int
                realistically the int can be really short (2^4)
                so now 12 bits for 2 doubles, 2^6 = 64
                a single body is probably unlikely to be hitting more than like 4 blocks i hope
                so we can have 1/16 precision guaranteed for a single edge
                scale up so that an int represents 16, round to int, then take only firs 6 bits
                this means we can handle 64 states, which is 4 blocks
                
                
            edges on bodies are just an index
            ID:
            41-42: axis
            43-48: body edge idx
            49-56: mesh edge idx
            57-64: mesh edge pt idx
        fe
            axis, a, b, i, really don't need much more, so:
                we have 2 bits for axis, then 22 bits for 2 ints
            41-42: axis
            43-53: mesh edge idx
            54-64: mesh edge pt idx

a2a
    face
        2 ints
    edge
        2 ints
*/

object ManifoldIDGenerator {
    fun constructA2SEdgeManifoldID(
        activeBody: ActiveBody,
        bodyEdgeIdx: Int,
        meshEdgeIdx: Int,
        meshEdgePointIdx: Int,
        edgeAxis: LocalMesher.AxisD,
        mesh: LocalMesher.Mesh2,
        envContactUtil: LocalEnvContactUtil,
    ): Long {
        val r = constructA2SStart(activeBody, A2SManifoldType.EDGE_EDGE, mesh)
            .or((edgeAxis.ordinal.toLong() and 0b11) shl 40)
            .or((bodyEdgeIdx.toLong() and 0b111111) shl 42)
            .or((meshEdgeIdx.toLong() and 0xFF) shl 48)
            .or((meshEdgePointIdx.toLong() and 0xFF) shl 56)

        return r
    }

    fun constructA2SEdgeManifoldID(
        activeBody: ActiveBody,
        edgeAxis: LocalMesher.AxisD,
        meshEdgePointIdx: Int,
        meshEdgeIdx: Int,
        mesh: LocalMesher.Mesh2,
        envContactUtil: LocalEnvContactUtil,
    ): Long {
        val r = constructA2SStart(activeBody, A2SManifoldType.FACE_EDGE, mesh)
            .or((edgeAxis.ordinal.toLong() and 0b11) shl 40)
            .or((meshEdgeIdx.toLong() and 0b11111111111) shl 42)
            .or((meshEdgePointIdx.toLong() and 0b11111111111) shl 53)

        return r
    }

    fun constructA2SFaceManifoldID(
        activeBody: ActiveBody,
        faceAxis: LocalMesher.AxisD,
        faceLevel: Double,
        mesh: LocalMesher.Mesh2,
        math: LocalMathUtil,
        envContactUtil: LocalEnvContactUtil,
    ): Long {
        val r = constructA2SStart(activeBody, A2SManifoldType.FACE_FACE, mesh)
            .or((faceAxis.ordinal.toLong() and 0b11) shl 40)
            .or(((faceLevel - mesh.minLevel(faceAxis)) / (32.0 / 0xffffff.toDouble())).toLong() shl 42)

        return r
    }

    fun constructA2AFaceManifoldID(
        first: ActiveBody,
        second: ActiveBody,
        firstIdx: Int,
        secondIdx: Int,
    ): Long {
        val r = constructA2AStart(first, second, A2AManifoldType.FACE)
            .or((firstIdx.toLong() and ((1L shl 8) - 1)) shl 48)
            .or((secondIdx.toLong() and ((1L shl 8) - 1)) shl 56)

        return r
    }

    fun constructA2AEdgeManifoldID(
        first: ActiveBody,
        second: ActiveBody,
        firstIdx: Int,
        secondIdx: Int,
    ): Long {
        val r = constructA2AStart(first, second, A2AManifoldType.EDGE)
            .or((firstIdx.toLong() and ((1L shl 8) - 1)) shl 48)
            .or((secondIdx.toLong() and ((1L shl 8) - 1)) shl 56)

        return r
    }

    private fun constructA2SStart(
        activeBody: ActiveBody,
        manifoldType: A2SManifoldType,
        mesh: LocalMesher.Mesh2,
    ): Long {
        return (activeBody.uuid.hashCode().toLong() and 0xFFFFFFFF)
            .or((manifoldType.ordinal and 0b11).toLong() shl 32)
            .or((mesh.hashCode().toLong() and 0b111111) shl 34)
    }

    private fun constructA2AStart(
        first: ActiveBody,
        second: ActiveBody,
        manifoldType: A2AManifoldType,
    ): Long {
        return (manifoldType.ordinal.toLong() and 0b11)
            .or(((first.uuid.mostSignificantBits xor second.uuid.leastSignificantBits) and ((1L shl 46) - 1)) shl 2)
    }

    enum class A2SManifoldType {
        FACE_FACE, EDGE_EDGE, FACE_EDGE
    }

    enum class A2AManifoldType {
        FACE, EDGE
    }
}