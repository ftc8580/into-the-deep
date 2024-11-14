package org.firstinspires.ftc.teamcode.subsystem

import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class ActiveIntakeSubsystem(private val hardware: HardwareManager) {
    fun rotateHome() {
        hardware.intakeRotateServo?.position = WRIST_MIN
    }

    fun rotateToBasket() {
        hardware.intakeRotateServo?.position = WRIST_DELIVER
    }

    fun rotateIncrementDown() {
        val currentPosition = hardware.intakeRotateServo?.position
        if (currentPosition != null && currentPosition < WRIST_MAX) {
            hardware.intakeRotateServo?.position = currentPosition + WRIST_STEP
        }
    }

    fun rotateIncrementUp() {
        val currentPosition = hardware.intakeRotateServo?.position
        if (currentPosition != null && currentPosition > WRIST_MIN) {
            hardware.intakeRotateServo?.position = currentPosition - WRIST_STEP
        }
    }

    fun runIntake() {
        hardware.intakeWheelServoRear?.power = -INTAKE_MAX_POWER
        hardware.intakeWheelServoFront?.power = INTAKE_MAX_POWER
    }

    fun runEject() {
        hardware.intakeWheelServoRear?.power = INTAKE_MAX_POWER
        hardware.intakeWheelServoFront?.power = -INTAKE_MAX_POWER
    }

    fun stopIntake() {
        hardware.intakeWheelServoRear?.power = 0.0
        hardware.intakeWheelServoFront?.power = 0.0
    }

    companion object {
        private const val WRIST_MIN = 0.0
        const val WRIST_DELIVER = 0.66
        private const val WRIST_MAX = 1.0
        private const val WRIST_STEP = 0.01
        private const val INTAKE_MAX_POWER = 1.0
    }
}