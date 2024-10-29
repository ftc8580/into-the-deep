package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class ActiveIntakeSubsystem(
    private val hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null
) : SubsystemBase() {
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
        hardware.intakeWheelServoRear?.power = -1.0
        hardware.intakeWheelServoFront?.power = 1.0
    }

    fun runEject() {
        hardware.intakeWheelServoRear?.power = 1.0
        hardware.intakeWheelServoFront?.power = -1.0
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
    }
}