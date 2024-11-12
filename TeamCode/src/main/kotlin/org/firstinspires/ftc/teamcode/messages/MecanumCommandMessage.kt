package org.firstinspires.ftc.teamcode.messages

data class MecanumCommandMessage(
    var voltage: Double,
    var leftFrontPower: Double,
    var leftBackPower: Double,
    var rightBackPower: Double,
    var rightFrontPower: Double
) {
    var timestamp: Long = System.nanoTime()
}