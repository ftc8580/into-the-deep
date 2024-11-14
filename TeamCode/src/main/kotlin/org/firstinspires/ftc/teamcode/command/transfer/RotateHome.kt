package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class RotateHome(private val armRotationSubsystem: ArmRotationSubsystem) : CommandBase() {
    init {
        addRequirements(armRotationSubsystem)
    }

    private var currentState = RotateState.IDLE

    override fun initialize() {
        println("RotateHome initialized")
        currentState = RotateState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RotateState.STARTED -> {
                if (armRotationSubsystem.isRotationHome) {
                    println("RotateHome started - already at home, set to finished")
                    armRotationSubsystem.setRotationMotorGroupPower(0.0)
                    armRotationSubsystem.resetRotationEncoder()
                    currentState = RotateState.FINISHED
                } else {
                    println("Rotate home started, rotating...")
                    currentState = RotateState.ROTATING
                    armRotationSubsystem.setRotationMotorGroupPower(1.0)
                }
            }
            RotateState.ROTATING -> {
                println("Still rotating...")
                if (armRotationSubsystem.isExtensionHome) {
                    println("RotateHome has reached home position, finishing...")
                    armRotationSubsystem.setRotationMotorGroupPower(0.0)
                    armRotationSubsystem.resetRotationEncoder()
                    currentState = RotateState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == RotateState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            armRotationSubsystem.setRotationMotorGroupPower(0.0)
        }

        println("Retract Home ending")
        currentState = RotateState.IDLE
    }

    companion object {
        enum class RotateState {
            IDLE,
            STARTED,
            ROTATING,
            FINISHED
        }
    }
}