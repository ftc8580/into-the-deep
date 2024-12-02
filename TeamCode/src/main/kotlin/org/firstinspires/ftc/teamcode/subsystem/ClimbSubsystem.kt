package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.util.isTimedOut

class ClimbSubsystem(hardware: HardwareManager) {
    private val climbServo = hardware.climbServo
    private var position = ClimbPosition.RETRACTED
    private val elapsed = ElapsedTime()

    fun set(power: Double) {
        climbServo?.power = power
    }

    fun extend() {
        if (climbServo == null) return

        elapsed.reset()
        while (position != ClimbPosition.EXTENDED) {
            if (elapsed.isTimedOut(EXTENSION_MS)) {
                climbServo.power = 0.0
                position = ClimbPosition.EXTENDED
            } else {
                climbServo.power = 1.0
            }
        }
    }

    fun retract() {
        if (climbServo == null) return

        elapsed.reset()
        while (position != ClimbPosition.RETRACTED) {
            if (elapsed.isTimedOut(RETRACTION_MS)) {
                climbServo.power = 0.0
                position = ClimbPosition.RETRACTED
            } else {
                climbServo.power = -1.0
            }
        }
    }

    companion object {
        // TODO: Use real values
        private const val EXTENSION_MS = 1000.0
        private const val RETRACTION_MS = 2000.0
    }
}

enum class ClimbPosition {
    EXTENDED,
    RETRACTED
}