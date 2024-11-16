package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.actions.internal.ActionState
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem
import kotlin.properties.Delegates

class RotationPosition(
    private val armRotationSubsystem: ArmRotationSubsystem,
    private val target: ArmRotationPosition
) : Action {
    private var currentState = ActionState.IDLE
    private var rotationDirection = RotationDirection.UP

    private var power by Delegates.notNull<Double>()
    private var initialPosition by Delegates.notNull<Int>()

    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            ActionState.IDLE -> {
                currentState = ActionState.STARTED
                initialPosition = armRotationSubsystem.currentPosition!!

                if (target.position < initialPosition) {
                    rotationDirection = RotationDirection.DOWN
                }

                power = if (rotationDirection == RotationDirection.UP) {
                    -MAX_POWER
                } else {
                    MAX_POWER
                }

                armRotationSubsystem.setRotationMotorGroupPower(power)

                currentState = ActionState.STARTED

                return true
            }

            ActionState.STARTED -> {
                val currentPosition = armRotationSubsystem.currentPosition

                if (currentPosition != null) {
                    if (rotationDirection == RotationDirection.UP && currentPosition >= target.position) {
                        armRotationSubsystem.setRotationMotorGroupPower(0.0)
                        currentState = ActionState.FINISHED
                    } else if (rotationDirection == RotationDirection.DOWN && currentPosition <= target.position) {
                        armRotationSubsystem.setRotationMotorGroupPower(0.0)
                        currentState = ActionState.FINISHED
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
        private const val MAX_POWER = 0.7

        enum class RotationDirection {
            UP,
            DOWN
        }
    }
}