package org.firstinspires.ftc.teamcode.subsystem

import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class GripperSubsystem(private val hardware: HardwareManager) {
    fun set(height: GripperHeight) {
        hardware.gripperServo?.position = height.position
    }
}

enum class GripperHeight(val position: Double) {
    HOME(0.005),
    LOW(0.3),
    HIGH(1.0)
}