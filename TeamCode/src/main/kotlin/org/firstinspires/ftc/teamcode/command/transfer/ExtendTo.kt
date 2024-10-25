package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class ExtendTo(private val viperArmSubsystem: ViperArmSubsystem, private val target: Int) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = ExtensionState.IDLE
    private var extensionDirection = ExtensionDirection.OUT

    override fun initialize() {
        currentState = ExtensionState.STARTED
    }

    override fun execute() {
        when (currentState) {
            ExtensionState.STARTED -> {
                if (target < viperArmSubsystem.getExtensionMotorGroupPosition()) {
                    extensionDirection = ExtensionDirection.IN
                }

                val power = if (extensionDirection == ExtensionDirection.OUT) {
                    1.0
                } else {
                    -1.0
                }

                viperArmSubsystem.setExtensionMotorGroupPower(power)
                currentState = ExtensionState.EXTENDING
            }
            ExtensionState.EXTENDING -> {
                if (extensionDirection == ExtensionDirection.OUT && viperArmSubsystem.getExtensionMotorGroupPosition() >= target) {
                    viperArmSubsystem.setExtensionMotorGroupPower(0.0)
                    currentState = ExtensionState.FINISHED
                } else if (extensionDirection == ExtensionDirection.IN && viperArmSubsystem.getExtensionMotorGroupPosition() <= target) {
                    viperArmSubsystem.setExtensionMotorGroupPower(0.0)
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
            viperArmSubsystem.setExtensionMotorGroupPower(0.0)
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