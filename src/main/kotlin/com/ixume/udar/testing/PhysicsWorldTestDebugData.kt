package com.ixume.udar.testing

class PhysicsWorldTestDebugData(
    var totalPairs: Int = 0,
    var totalPairCollisionChecks: Int = 0,
    var pairCollisions: Int = 0,

    var totalCompositePairs: Int = 0,
    var totalCompositeCollisionChecks: Int = 0,
    var compositeCollisions: Int = 0,

    var totalEnvironmentCollisionChecks: Int = 0,
    var environmentCollisions: Int = 0,

    var missedEarlies: Int = 0,
) {
    fun reset() {
        totalPairs = 0
        totalPairCollisionChecks = 0
        pairCollisions = 0

        totalCompositePairs = 0
        totalCompositeCollisionChecks = 0
        compositeCollisions = 0

        totalEnvironmentCollisionChecks = 0
        environmentCollisions = 0

        missedEarlies = 0
    }

    fun print() {
        println("*** DEBUG DATA ***")
        println(" - totalPairs: $totalPairs")
        println(" - totalPairCollisionChecks: $totalPairCollisionChecks")
        println(" - pairCollisions: $pairCollisions")
        println(" - ${pairCollisions.toDouble() / totalPairCollisionChecks.toDouble() * 100.0}% of pairs collided")
        println(" - ${compositeCollisions.toDouble() / totalCompositeCollisionChecks.toDouble() * 100.0}% of composite pairs collided")
        println(" - totalEnvironmentCollisionChecks: $totalEnvironmentCollisionChecks")
        println(" - environmentCollisions: $environmentCollisions")
        println(" - ${environmentCollisions.toDouble() / totalEnvironmentCollisionChecks.toDouble() * 100.0}% of pairs collided")
        println(" - missedEarlies: $missedEarlies ( ${(missedEarlies.toDouble() / (totalPairCollisionChecks.toDouble() - pairCollisions.toDouble())) * 100.0}% of waste )")
        println("******************")
    }
}