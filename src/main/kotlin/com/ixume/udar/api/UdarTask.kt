package com.ixume.udar.api

sealed interface UdarTask {
    val isCancelled: Boolean
    fun run()

    interface Runnable : UdarTask {
        companion object {
            fun of(runnable: java.lang.Runnable): Runnable {
                return object : Runnable {
                    override val isCancelled: Boolean = false
                    override fun run() = runnable.run()
                }
            }
        }
    }

    open class Delayed(
        var delay: Long,
        val runnable: Runnable,
    ) : UdarTask by runnable

    open class Timer(
        val period: Long,
        var delay: Long,
        val runnable: Runnable,
    ) : UdarTask by runnable {
        private var time = 0L

        /**
         * Whether an action should be performed
         */
        fun tick() = (time++ % period) == 0L
    }
}

operator fun UdarTask.invoke() = run()