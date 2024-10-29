package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class GripperSubsystem(
    private val hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null

) : SubsystemBase() {

    fun gripperTop(): Boolean {
        val currentGripperPosition = hardware.gripperServo?.position
        var gripperTopPosition = false
        if (currentGripperPosition !=null) {
            gripperTopPosition = (currentGripperPosition >= 0.998)
        }
        return gripperTopPosition
    }

    fun gripperBottom(): Boolean {
        val currentGripperPosition = hardware.gripperServo?.position
        var gripperTopPosition = false
        if (currentGripperPosition !=null) {
            gripperTopPosition = (currentGripperPosition <= 0.007)
        }
        return gripperTopPosition
    }
    fun getGripperPosition(): Double? {
        val currentGripperPosition = hardware.gripperServo?.position
        return currentGripperPosition
    }

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

    companion object {
        private const val GRIPPER_HIGH_CHAMBER_HEIGHT = 1.0
        private const val GRIPPER_LOW_CHAMBER_HEIGHT = 0.3
        private const val GRIPPER_PICKUP_HEIGHT = 0.005
    }
}