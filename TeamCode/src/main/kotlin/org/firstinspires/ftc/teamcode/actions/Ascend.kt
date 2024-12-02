package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
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
                currentState = AscensionState.L2_CLIMBING
                elapsedTime.reset()
                return true
            }
            AscensionState.L2_CLIMBING -> {
                if (elapsedTime.isTimedOut(L2_CLIMB_TIMEOUT)) {
                    climbSubsystem.set(0.0)
                    currentState = AscensionState.L2_ROTATING
                    return true
                } else {
                    climbSubsystem.set(-1.0) // TODO: Is this rotating in the right direction?
                    return true
                }
            }
            AscensionState.L2_ROTATING -> {
                if ((rotationSubsystem.currentPosition ?: 0) < L2_ROTATE_POSITION) {
                    rotationSubsystem.setRotationMotorGroupPower(1.0)
                } else {
                    rotationSubsystem.setRotationMotorGroupPower(0.0)
                }

                currentState = if (target == AscensionTarget.LEVEL_2) {
                    AscensionState.FINISHED
                } else {
                    AscensionState.L3_CLIMBING
                }

                return true
            }
            AscensionState.L3_CLIMBING -> {
                TODO()
            }
            AscensionState.L3_ROTATING -> {
                TODO()
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
        L2_ROTATING,
        L3_CLIMBING,
        L3_ROTATING,
        FINISHED
    }

    companion object {
        // TODO: Validate all of these values
        private const val L2_CLIMB_TIMEOUT = 2000.0
        private const val L2_ROTATE_POSITION = 2000
    }
}

enum class AscensionTarget {
    LEVEL_2,
    LEVEL_3
}