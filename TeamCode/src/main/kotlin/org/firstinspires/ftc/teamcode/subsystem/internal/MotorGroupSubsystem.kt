package org.firstinspires.ftc.teamcode.subsystem.internal

import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.util.MotorGroup

abstract class MotorGroupSubsystem {
    fun MotorGroup.safelyGoToPosition(
        targetPosition: Int,
        motorSpeed: Double
    ) {
        this.targetPosition = targetPosition
        this.power = motorSpeed
    }

    fun getBoundedPosition(position: Int, min: Int, max: Int): Int =
        Range.clip(position, min, max)

    fun getBoundedPower(power: Double, min: Double = -1.0, max: Double = 1.0): Double =
        Range.clip(power, min, max)
}