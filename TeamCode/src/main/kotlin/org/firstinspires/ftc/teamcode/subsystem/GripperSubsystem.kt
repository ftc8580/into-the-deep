package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class GripperSubsystem(
    private val hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null
) : SubsystemBase() {
    fun incrementUp() {
        val currentPosition = hardware.gripperServo?.position

        currentPosition?.let {
            val newPosition = currentPosition + 0.01
            hardware.gripperServo?.position = newPosition
        }
    }

    fun incrementDown() {
        val currentPosition = hardware.gripperServo?.position

        currentPosition?.let {
            val newPosition = currentPosition - 0.01
            hardware.gripperServo?.position = newPosition
        }
    }

    fun setPickupHeight() {
        hardware.gripperServo?.position = 0.005
    }

    fun setHighChamberHeight() {
        hardware.gripperServo?.position = 1.0
    }

    fun setLowChamberHeight() {
        hardware.gripperServo?.position = 0.3
    }
}