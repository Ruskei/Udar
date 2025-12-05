package com.ixume.udar.physics.constraint

import kotlin.math.max

@JvmInline
value class ConstraintData3p0r(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        constraintIdx: Int,
        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        im1: Float,
        im2: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e34x: Float,
        e34y: Float,
        e34z: Float,

        k11: Float,
        k12: Float,
        k13: Float,

        k22: Float,
        k23: Float,

        k33: Float,

        b1: Float,
        b2: Float,
        b3: Float,

        l1: Float,
        l2: Float,
        l3: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)

        this[cursor + 3] = r1x
        this[cursor + 4] = r1y
        this[cursor + 5] = r1z

        this[cursor + 6] = r2x
        this[cursor + 7] = r2y
        this[cursor + 8] = r2z

        this[cursor + 9] = im1
        this[cursor + 10] = im2

        this[cursor + 11] = e12x
        this[cursor + 12] = e12y
        this[cursor + 13] = e12z

        this[cursor + 14] = e22x
        this[cursor + 15] = e22y
        this[cursor + 16] = e22z

        this[cursor + 17] = e32x
        this[cursor + 18] = e32y
        this[cursor + 19] = e32z

        this[cursor + 20] = e14x
        this[cursor + 21] = e14y
        this[cursor + 22] = e14z

        this[cursor + 23] = e24x
        this[cursor + 24] = e24y
        this[cursor + 25] = e24z

        this[cursor + 26] = e34x
        this[cursor + 27] = e34y
        this[cursor + 28] = e34z

        this[cursor + 29] = k11
        this[cursor + 30] = k12
        this[cursor + 31] = k13

        this[cursor + 32] = k22
        this[cursor + 33] = k23

        this[cursor + 34] = k33

        this[cursor + 35] = b1
        this[cursor + 36] = b2
        this[cursor + 37] = b3

        this[cursor + 38] = l1
        this[cursor + 39] = l2
        this[cursor + 40] = l3
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            constraintIdx: Int,

            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * DATA_SIZE + 0].toRawBits(),
                this[i * DATA_SIZE + 1].toRawBits(),
                this[i * DATA_SIZE + 2].toRawBits(),
                i * DATA_SIZE,
            )

            i++
        }
    }

    companion object {
        const val DATA_SIZE = 41
        const val J_OFFSET = 3
        const val IM_OFFSET = 9
        const val E_OFFSET = 11
        const val K_OFFSET = 29
        const val B_OFFSET = 35
        const val L_OFFSET = 38
    }
}

@JvmInline
value class ConstraintData3p1r(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        constraintIdx: Int,
        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        j42x: Float,
        j42y: Float,
        j42z: Float,

        im1: Float,
        im2: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

        e42x: Float,
        e42y: Float,
        e42z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e34x: Float,
        e34y: Float,
        e34z: Float,

        e44x: Float,
        e44y: Float,
        e44z: Float,

        k11: Float,
        k12: Float,
        k13: Float,
        k14: Float,

        k22: Float,
        k23: Float,
        k24: Float,

        k33: Float,
        k34: Float,

        k44: Float,

        b1: Float,
        b2: Float,
        b3: Float,
        b4: Float,

        l1: Float,
        l2: Float,
        l3: Float,
        l4: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)

        this[cursor + 3] = r1x
        this[cursor + 4] = r1y
        this[cursor + 5] = r1z

        this[cursor + 6] = r2x
        this[cursor + 7] = r2y
        this[cursor + 8] = r2z

        this[cursor + 9] = j42x
        this[cursor + 10] = j42y
        this[cursor + 11] = j42z

        this[cursor + 12] = im1
        this[cursor + 13] = im2

        this[cursor + 14] = e12x
        this[cursor + 15] = e12y
        this[cursor + 16] = e12z

        this[cursor + 17] = e22x
        this[cursor + 18] = e22y
        this[cursor + 19] = e22z

        this[cursor + 20] = e32x
        this[cursor + 21] = e32y
        this[cursor + 22] = e32z

        this[cursor + 23] = e42x
        this[cursor + 24] = e42y
        this[cursor + 25] = e42z

        this[cursor + 26] = e14x
        this[cursor + 27] = e14y
        this[cursor + 28] = e14z

        this[cursor + 29] = e24x
        this[cursor + 30] = e24y
        this[cursor + 31] = e24z

        this[cursor + 32] = e34x
        this[cursor + 33] = e34y
        this[cursor + 34] = e34z

        this[cursor + 35] = e44x
        this[cursor + 36] = e44y
        this[cursor + 37] = e44z

        this[cursor + 38] = k11
        this[cursor + 39] = k12
        this[cursor + 40] = k13
        this[cursor + 41] = k14

        this[cursor + 42] = k22
        this[cursor + 43] = k23
        this[cursor + 44] = k24

        this[cursor + 45] = k33
        this[cursor + 46] = k34

        this[cursor + 47] = k44

        this[cursor + 48] = b1
        this[cursor + 49] = b2
        this[cursor + 50] = b3
        this[cursor + 51] = b4

        this[cursor + 52] = l1
        this[cursor + 53] = l2
        this[cursor + 54] = l3
        this[cursor + 55] = l4
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            constraintIdx: Int,

            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * DATA_SIZE + 0].toRawBits(),
                this[i * DATA_SIZE + 1].toRawBits(),
                this[i * DATA_SIZE + 2].toRawBits(),
                i * DATA_SIZE,
            )

            i++
        }
    }

    companion object {
        const val DATA_SIZE = 56
        const val J_OFFSET = 3
        const val IM_OFFSET = 12
        const val E_OFFSET = 14
        const val K_OFFSET = 38
        const val B_OFFSET = 48
        const val L_OFFSET = 52
    }
}

@JvmInline
value class ConstraintData3p2r(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        constraintIdx: Int,
        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        j42x: Float,
        j42y: Float,
        j42z: Float,

        j52x: Float,
        j52y: Float,
        j52z: Float,

        im1: Float,
        im2: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

        e42x: Float,
        e42y: Float,
        e42z: Float,

        e52x: Float,
        e52y: Float,
        e52z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e34x: Float,
        e34y: Float,
        e34z: Float,

        e44x: Float,
        e44y: Float,
        e44z: Float,

        e54x: Float,
        e54y: Float,
        e54z: Float,

        k11: Float,
        k12: Float,
        k13: Float,
        k14: Float,
        k15: Float,

        k22: Float,
        k23: Float,
        k24: Float,
        k25: Float,

        k33: Float,
        k34: Float,
        k35: Float,

        k44: Float,
        k45: Float,

        k55: Float,

        b1: Float,
        b2: Float,
        b3: Float,
        b4: Float,
        b5: Float,

        l1: Float,
        l2: Float,
        l3: Float,
        l4: Float,
        l5: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)

        this[cursor + 3] = r1x
        this[cursor + 4] = r1y
        this[cursor + 5] = r1z

        this[cursor + 6] = r2x
        this[cursor + 7] = r2y
        this[cursor + 8] = r2z

        this[cursor + 9] = j42x
        this[cursor + 10] = j42y
        this[cursor + 11] = j42z

        this[cursor + 12] = j52x
        this[cursor + 13] = j52y
        this[cursor + 14] = j52z

        this[cursor + 15] = im1
        this[cursor + 16] = im2

        this[cursor + 17] = e12x
        this[cursor + 18] = e12y
        this[cursor + 19] = e12z

        this[cursor + 20] = e22x
        this[cursor + 21] = e22y
        this[cursor + 22] = e22z

        this[cursor + 23] = e32x
        this[cursor + 24] = e32y
        this[cursor + 25] = e32z

        this[cursor + 26] = e42x
        this[cursor + 27] = e42y
        this[cursor + 28] = e42z

        this[cursor + 29] = e52x
        this[cursor + 30] = e52y
        this[cursor + 31] = e52z

        this[cursor + 32] = e14x
        this[cursor + 33] = e14y
        this[cursor + 34] = e14z

        this[cursor + 35] = e24x
        this[cursor + 36] = e24y
        this[cursor + 37] = e24z

        this[cursor + 38] = e34x
        this[cursor + 39] = e34y
        this[cursor + 40] = e34z

        this[cursor + 41] = e44x
        this[cursor + 42] = e44y
        this[cursor + 43] = e44z

        this[cursor + 44] = e54x
        this[cursor + 45] = e54y
        this[cursor + 46] = e54z

        this[cursor + 47] = k11
        this[cursor + 48] = k12
        this[cursor + 49] = k13
        this[cursor + 50] = k14
        this[cursor + 51] = k15

        this[cursor + 52] = k22
        this[cursor + 53] = k23
        this[cursor + 54] = k24
        this[cursor + 55] = k25

        this[cursor + 56] = k33
        this[cursor + 57] = k34
        this[cursor + 58] = k35

        this[cursor + 59] = k44
        this[cursor + 60] = k45

        this[cursor + 61] = k55

        this[cursor + 62] = b1
        this[cursor + 63] = b2
        this[cursor + 64] = b3
        this[cursor + 65] = b4
        this[cursor + 66] = b5

        this[cursor + 67] = l1
        this[cursor + 68] = l2
        this[cursor + 69] = l3
        this[cursor + 70] = l4
        this[cursor + 71] = l5
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            constraintIdx: Int,

            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * DATA_SIZE + 0].toRawBits(),
                this[i * DATA_SIZE + 1].toRawBits(),
                this[i * DATA_SIZE + 2].toRawBits(),
                i * DATA_SIZE,
            )

            i++
        }
    }

    companion object {
        const val DATA_SIZE = 72
        const val J_OFFSET = 3
        const val IM_OFFSET = 15
        const val E_OFFSET = 17
        const val K_OFFSET = 47
        const val B_OFFSET = 62
        const val L_OFFSET = 67
    }
}

@JvmInline
value class ConstraintData3p3r(val value: FloatArray) {
    constructor(size: Int) : this(FloatArray(size))

    inline operator fun get(idx: Int) = value[idx]
    inline operator fun set(idx: Int, v: Float) {
        value[idx] = v
    }

    fun sizeFor(constraints: Int): Int {
        return max(constraints * DATA_SIZE, value.size)
    }

    fun set(
        cursor: Int,

        constraintIdx: Int,
        body1Idx: Int,
        body2Idx: Int,

        r1x: Float,
        r1y: Float,
        r1z: Float,

        r2x: Float,
        r2y: Float,
        r2z: Float,

        j42x: Float,
        j42y: Float,
        j42z: Float,

        j52x: Float,
        j52y: Float,
        j52z: Float,

        j62x: Float,
        j62y: Float,
        j62z: Float,

        im1: Float,
        im2: Float,

        e12x: Float,
        e12y: Float,
        e12z: Float,

        e22x: Float,
        e22y: Float,
        e22z: Float,

        e32x: Float,
        e32y: Float,
        e32z: Float,

        e42x: Float,
        e42y: Float,
        e42z: Float,

        e52x: Float,
        e52y: Float,
        e52z: Float,

        e62x: Float,
        e62y: Float,
        e62z: Float,

        e14x: Float,
        e14y: Float,
        e14z: Float,

        e24x: Float,
        e24y: Float,
        e24z: Float,

        e34x: Float,
        e34y: Float,
        e34z: Float,

        e44x: Float,
        e44y: Float,
        e44z: Float,

        e54x: Float,
        e54y: Float,
        e54z: Float,

        e64x: Float,
        e64y: Float,
        e64z: Float,

        k11: Float,
        k12: Float,
        k13: Float,
        k14: Float,
        k15: Float,
        k16: Float,

        k22: Float,
        k23: Float,
        k24: Float,
        k25: Float,
        k26: Float,

        k33: Float,
        k34: Float,
        k35: Float,
        k36: Float,

        k44: Float,
        k45: Float,
        k46: Float,

        k55: Float,
        k56: Float,

        k66: Float,

        b1: Float,
        b2: Float,
        b3: Float,
        b4: Float,
        b5: Float,
        b6: Float,

        l1: Float,
        l2: Float,
        l3: Float,
        l4: Float,
        l5: Float,
        l6: Float,
    ) {
        this[cursor + 0] = Float.fromBits(constraintIdx)
        this[cursor + 1] = Float.fromBits(body1Idx)
        this[cursor + 2] = Float.fromBits(body2Idx)

        this[cursor + 3] = r1x
        this[cursor + 4] = r1y
        this[cursor + 5] = r1z

        this[cursor + 6] = r2x
        this[cursor + 7] = r2y
        this[cursor + 8] = r2z

        this[cursor + 9] = j42x
        this[cursor + 10] = j42y
        this[cursor + 11] = j42z

        this[cursor + 12] = j52x
        this[cursor + 13] = j52y
        this[cursor + 14] = j52z

        this[cursor + 15] = j62x
        this[cursor + 16] = j62y
        this[cursor + 17] = j62z

        this[cursor + 18] = im1
        this[cursor + 19] = im2

        this[cursor + 20] = e12x
        this[cursor + 21] = e12y
        this[cursor + 22] = e12z

        this[cursor + 23] = e22x
        this[cursor + 24] = e22y
        this[cursor + 25] = e22z

        this[cursor + 26] = e32x
        this[cursor + 27] = e32y
        this[cursor + 28] = e32z

        this[cursor + 29] = e42x
        this[cursor + 30] = e42y
        this[cursor + 31] = e42z

        this[cursor + 32] = e52x
        this[cursor + 33] = e52y
        this[cursor + 34] = e52z

        this[cursor + 35] = e62x
        this[cursor + 36] = e62y
        this[cursor + 37] = e62z

        this[cursor + 38] = e14x
        this[cursor + 39] = e14y
        this[cursor + 40] = e14z

        this[cursor + 41] = e24x
        this[cursor + 42] = e24y
        this[cursor + 43] = e24z

        this[cursor + 44] = e34x
        this[cursor + 45] = e34y
        this[cursor + 46] = e34z

        this[cursor + 47] = e44x
        this[cursor + 48] = e44y
        this[cursor + 49] = e44z

        this[cursor + 50] = e54x
        this[cursor + 51] = e54y
        this[cursor + 52] = e54z

        this[cursor + 53] = e64x
        this[cursor + 54] = e64y
        this[cursor + 55] = e64z

        this[cursor + 56] = k11
        this[cursor + 57] = k12
        this[cursor + 58] = k13
        this[cursor + 59] = k14
        this[cursor + 60] = k15
        this[cursor + 61] = k16

        this[cursor + 62] = k22
        this[cursor + 63] = k23
        this[cursor + 64] = k24
        this[cursor + 65] = k25
        this[cursor + 66] = k26

        this[cursor + 67] = k33
        this[cursor + 68] = k34
        this[cursor + 69] = k35
        this[cursor + 70] = k36

        this[cursor + 71] = k44
        this[cursor + 72] = k45
        this[cursor + 73] = k46

        this[cursor + 74] = k55
        this[cursor + 75] = k56

        this[cursor + 76] = k66

        this[cursor + 77] = b1
        this[cursor + 78] = b2
        this[cursor + 79] = b3
        this[cursor + 80] = b4
        this[cursor + 81] = b5
        this[cursor + 82] = b6

        this[cursor + 83] = l1
        this[cursor + 84] = l2
        this[cursor + 85] = l3
        this[cursor + 86] = l4
        this[cursor + 87] = l5
        this[cursor + 88] = l6
    }

    inline fun forEach(
        numConstraints: Int,
        block: (
            constraintIdx: Int,

            b1Idx: Int,
            b2Idx: Int,

            rawIdx: Int,
        ) -> Unit,
    ) {
        var i = 0
        while (i < numConstraints) {
            block(
                this[i * DATA_SIZE + 0].toRawBits(),
                this[i * DATA_SIZE + 1].toRawBits(),
                this[i * DATA_SIZE + 2].toRawBits(),
                i * DATA_SIZE,
            )

            i++
        }
    }

    companion object {
        const val DATA_SIZE = 89
        const val J_OFFSET = 3
        const val IM_OFFSET = 18
        const val E_OFFSET = 20
        const val K_OFFSET = 56
        const val B_OFFSET = 77
        const val L_OFFSET = 83
    }
}