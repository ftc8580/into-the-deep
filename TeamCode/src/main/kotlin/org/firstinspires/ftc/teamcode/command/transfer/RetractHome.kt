package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class RetractHome(private val viperArmSubsystem: ViperArmSubsystem) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = RetractState.IDLE

    override fun initialize() {
        currentState = RetractState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RetractState.STARTED -> {
                viperArmSubsystem.setExtensionMotorGroupPower(-1.0)
                currentState = RetractState.RETRACTING
            }
            RetractState.RETRACTING -> {
                // TODO Adjust target rotation position
                if (viperArmSubsystem.isExtensionHome) {
                    viperArmSubsystem.setExtensionMotorGroupPower(0.0)
                    viperArmSubsystem.resetExtensionEncoder()
                    currentState = RetractState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == RetractState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            viperArmSubsystem.setExtensionMotorGroupPower(0.0)
        }

        currentState = RetractState.IDLE
    }

    companion object {
        enum class RetractState {
            IDLE,
            STARTED,
            RETRACTING,
            FINISHED
        }
    }
}