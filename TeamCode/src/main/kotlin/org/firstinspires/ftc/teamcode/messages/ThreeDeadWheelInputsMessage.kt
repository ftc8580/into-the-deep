package org.firstinspires.ftc.teamcode.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair


data class ThreeDeadWheelInputsMessage(
    var par0: PositionVelocityPair,
    var par1: PositionVelocityPair,
    var perp: PositionVelocityPair
) {
    var timestamp: Long = System.nanoTime()
}