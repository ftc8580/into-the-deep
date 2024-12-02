package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.actions.internal.ActionState
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ViperExtensionSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut
import kotlin.properties.Delegates

class ExtensionPosition(
    private val armExtensionSubsystem: ViperExtensionSubsystem,
    private val target: ArmExtensionPosition,
    private val delayMs: Double = 0.0
) : Action {
    private var currentState = ActionState.IDLE
    private var extensionDirection = ExtensionDirection.OUT
    private val elapsedTime = ElapsedTime()

    private var power by Delegates.notNull<Double>()
    private var initialPosition by Delegates.notNull<Int>()

    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            ActionState.IDLE -> {
                if (!elapsedTime.isTimedOut(delayMs)) return true

                currentState = ActionState.STARTED
                initialPosition = armExtensionSubsystem.currentPosition!!

                if (target == ArmExtensionPosition.HOME && armExtensionSubsystem.isExtensionHome) {
                    currentState = ActionState.FINISHED
                    return false
                }

                if (target.position < initialPosition || target == ArmExtensionPosition.HOME) {
                    extensionDirection = ExtensionDirection.IN
                }

                power = if (extensionDirection == ExtensionDirection.OUT) {
                    MAX_POWER
                } else {
                    -MAX_POWER
                }

                armExtensionSubsystem.setExtensionMotorGroupPower(power)

                currentState = ActionState.STARTED

                return true
            }

            ActionState.STARTED -> {
                val currentPosition = armExtensionSubsystem.currentPosition

                if (currentPosition != null) {
                    if (target == ArmExtensionPosition.HOME) {
                        if (armExtensionSubsystem.isExtensionHome) {
                            armExtensionSubsystem.setExtensionMotorGroupPower(0.0)
                            currentState = ActionState.FINISHED
                        }
                    } else {
                        if (extensionDirection == ExtensionDirection.OUT && currentPosition >= target.position) {
                            armExtensionSubsystem.setExtensionMotorGroupPower(0.0)
                            currentState = ActionState.FINISHED
                        } else if (extensionDirection == ExtensionDirection.IN && (currentPosition <= target.position || armExtensionSubsystem.isExtensionHome)) {
                            armExtensionSubsystem.setExtensionMotorGroupPower(0.0)
                            currentState = ActionState.FINISHED
                        }
                    }
                }

                return true
            }

            ActionState.FINISHED -> {
                currentState = ActionState.IDLE
                return false
            }
        }
    }

    companion object {
        private const val MAX_POWER = 1.0

        enum class ExtensionDirection {
            OUT,
            IN
        }
    }
}