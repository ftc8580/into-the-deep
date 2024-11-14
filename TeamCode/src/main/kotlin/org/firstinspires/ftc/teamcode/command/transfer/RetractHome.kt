package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class RetractHome(private val armRotationSubsystem: ArmRotationSubsystem) : CommandBase() {
    init {
        addRequirements(armRotationSubsystem)
    }

    private var currentState = RetractState.IDLE

    override fun initialize() {
        println("RetractHome initialized")
        currentState = RetractState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RetractState.STARTED -> {
                if (armRotationSubsystem.isExtensionHome) {
                    println("RetractHome started - already at home, set to finished")
                    armRotationSubsystem.setExtensionMotorGroupPower(0.0)
                    armRotationSubsystem.resetExtensionEncoder()
                    currentState = RetractState.FINISHED
                } else {
                    println("Retract home started, retracting...")
                    currentState = RetractState.RETRACTING
                    armRotationSubsystem.setExtensionMotorGroupPower(-1.0)
                }
            }
            RetractState.RETRACTING -> {
                println("Still retracting...")
                if (armRotationSubsystem.isExtensionHome) {
                    println("RetractHome has reached home position, finishing...")
                    armRotationSubsystem.setExtensionMotorGroupPower(0.0)
                    armRotationSubsystem.resetExtensionEncoder()
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
            armRotationSubsystem.setExtensionMotorGroupPower(0.0)
        }

        println("Retract Home ending")
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