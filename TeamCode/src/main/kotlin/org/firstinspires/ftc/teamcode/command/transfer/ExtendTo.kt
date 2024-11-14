package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class ExtendTo(private val armRotationSubsystem: ArmRotationSubsystem, private val target: Int) : CommandBase() {
    init {
        addRequirements(armRotationSubsystem)
    }

    private var currentState = ExtensionState.IDLE
    private var extensionDirection = ExtensionDirection.OUT

    override fun initialize() {
        currentState = ExtensionState.STARTED
    }

    override fun execute() {
        when (currentState) {
            ExtensionState.STARTED -> {
                if (target < armRotationSubsystem.getExtensionMotorGroupPosition()) {
                    extensionDirection = ExtensionDirection.IN
                }

                val power = if (extensionDirection == ExtensionDirection.OUT) {
                    1.0
                } else {
                    -1.0
                }

                armRotationSubsystem.setExtensionMotorGroupPower(power)
                currentState = ExtensionState.EXTENDING
            }
            ExtensionState.EXTENDING -> {
                if (extensionDirection == ExtensionDirection.OUT && armRotationSubsystem.getExtensionMotorGroupPosition() >= target) {
                    armRotationSubsystem.setExtensionMotorGroupPower(0.0)
                    currentState = ExtensionState.FINISHED
                } else if (extensionDirection == ExtensionDirection.IN && armRotationSubsystem.getExtensionMotorGroupPosition() <= target) {
                    armRotationSubsystem.setExtensionMotorGroupPower(0.0)
                    currentState = ExtensionState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == ExtensionState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            armRotationSubsystem.setExtensionMotorGroupPower(0.0)
        }

        currentState = ExtensionState.IDLE
    }

    companion object {
        enum class ExtensionState {
            IDLE,
            STARTED,
            EXTENDING,
            FINISHED
        }

        enum class ExtensionDirection {
            OUT,
            IN
        }
    }
}