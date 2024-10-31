package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class RotateHome(private val viperArmSubsystem: ViperArmSubsystem) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = RotateState.IDLE

    override fun initialize() {
        println("RotateHome initialized")
        currentState = RotateState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RotateState.STARTED -> {
                if (viperArmSubsystem.isRotationHome) {
                    println("RotateHome started - already at home, set to finished")
                    viperArmSubsystem.setRotationMotorGroupPower(0.0)
                    viperArmSubsystem.resetRotationEncoder()
                    currentState = RotateState.FINISHED
                } else {
                    println("Rotate home started, rotating...")
                    currentState = RotateState.ROTATING
                    viperArmSubsystem.setRotationMotorGroupPower(1.0)
                }
            }
            RotateState.ROTATING -> {
                println("Still rotating...")
                if (viperArmSubsystem.isExtensionHome) {
                    println("RotateHome has reached home position, finishing...")
                    viperArmSubsystem.setRotationMotorGroupPower(0.0)
                    viperArmSubsystem.resetRotationEncoder()
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
            viperArmSubsystem.setRotationMotorGroupPower(0.0)
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