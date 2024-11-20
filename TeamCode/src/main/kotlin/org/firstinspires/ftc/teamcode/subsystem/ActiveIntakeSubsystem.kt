package org.firstinspires.ftc.teamcode.subsystem

import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class ActiveIntakeSubsystem(hardware: HardwareManager) {
    private val wristServo = hardware.intakeRotateServo
    private val intakeRearServo = hardware.intakeWheelServoRear
    private val intakeFrontServo = hardware.intakeWheelServoFront
    
    fun set(target: WristRotationPosition) {
        wristServo?.position = target.position
    }

    fun rotateIncrementDown() {
        val currentPosition = wristServo?.position
        if (currentPosition != null && currentPosition < WristRotationPosition.MAX.position) {
            wristServo?.position = currentPosition + WRIST_STEP
        }
    }

    fun rotateIncrementUp() {
        val currentPosition = wristServo?.position
        if (currentPosition != null && currentPosition > WristRotationPosition.MIN.position) {
            wristServo?.position = currentPosition - WRIST_STEP
        }
    }

    fun runIntake() {
        intakeRearServo?.power = INTAKE_MAX_POWER
        intakeFrontServo?.power = -INTAKE_MAX_POWER
    }

    fun runEject() {
        intakeRearServo?.power = -INTAKE_MAX_POWER
        intakeFrontServo?.power = INTAKE_MAX_POWER
    }

    fun stopIntake() {
        intakeRearServo?.power = 0.0
        intakeFrontServo?.power = 0.0
    }

    val currentPosition: Double?
        get() = wristServo?.position

    companion object {
        private const val WRIST_STEP = 0.01
        private const val INTAKE_MAX_POWER = 1.0
    }
}

enum class WristRotationPosition(val position: Double) {
    MIN(0.0),
    PICKUP(0.0),
    DELIVER(0.66),
    MAX(1.0)
}