package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ClimbSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperExtensionSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class Ascend(
    private val climbSubsystem: ClimbSubsystem,
    private val extensionSubsystem: ViperExtensionSubsystem,
    private val rotationSubsystem: ArmRotationSubsystem,
    private val target: AscensionTarget
) : Action {
    private var currentState = AscensionState.IDLE
    private val elapsedTime = ElapsedTime()

    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            AscensionState.IDLE -> {
                currentState = AscensionState.STARTED
                return true
            }
            AscensionState.STARTED -> {
                currentState = when (target) {
                    AscensionTarget.LEVEL_2 -> AscensionState.L2_CLIMBING
                    AscensionTarget.LEVEL_3 -> AscensionState.L3_CLIMBING
                }

                // Hold extension in position for L3
                extensionSubsystem.extendToPosition(ArmExtensionPosition.PRE_ASCENT)

                elapsedTime.reset()
                return true
            }
            AscensionState.L2_CLIMBING -> {
                val isTimedOut = elapsedTime.isTimedOut(L2_CLIMB_TIMEOUT)
                val isRotated = (rotationSubsystem.currentPosition ?: 0) >= L2_ROTATE_POSITION

                if (isTimedOut) {
                    climbSubsystem.set(0.0)
                } else {
                    climbSubsystem.set(-1.0) // Negative pulls the robot up
                }

                if (isRotated) {
                    rotationSubsystem.setRotationMotorGroupPower(0.0)
                    rotationSubsystem.rotateToPosition(ArmRotationPosition.PRE_L3)
                } else {
                    rotationSubsystem.setRotationMotorGroupPower(-1.0)
                }

                if (isTimedOut && isRotated) {
                    currentState = AscensionState.FINISHED
                }

                return true
            }
            AscensionState.L3_CLIMBING -> {
                if ((extensionSubsystem.currentPosition ?: 0) >= 200) {
                    extensionSubsystem.extendToPosition(ArmExtensionPosition.ASCEND)
                    rotationSubsystem.rotateToPosition(ArmRotationPosition.PRE_L3)
                } else {
                    extensionSubsystem.extendToPosition(ArmExtensionPosition.ASCEND)
                    rotationSubsystem.rotateToPosition(ArmRotationPosition.HOME)
                }

                currentState = AscensionState.FINISHED
                return true
            }
            AscensionState.FINISHED -> {
                currentState = AscensionState.IDLE
                return false
            }
        }
    }

    private enum class AscensionState {
        IDLE,
        STARTED,
        L2_CLIMBING,
        L3_CLIMBING,
        FINISHED
    }

    companion object {
        // TODO: Validate all of these values
        private const val L2_CLIMB_TIMEOUT = 5000.0
        private const val L2_ROTATE_POSITION = 3300
    }
}

enum class AscensionTarget {
    LEVEL_2,
    LEVEL_3
}