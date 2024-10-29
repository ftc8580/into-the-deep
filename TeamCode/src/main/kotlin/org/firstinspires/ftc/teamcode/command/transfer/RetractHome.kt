package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class RetractHome(private val viperArmSubsystem: ViperArmSubsystem) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = RetractState.IDLE

    override fun initialize() {
        println("RetractHome initialized")
        currentState = RetractState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RetractState.STARTED -> {
                if (viperArmSubsystem.isExtensionHome) {
                    println("RetractHome started - already at home, set to finished")
                    viperArmSubsystem.setExtensionMotorGroupPower(0.0)
                    viperArmSubsystem.resetExtensionEncoder()
                    currentState = RetractState.FINISHED
                } else {
                    println("Retract home started, retracting...")
                    currentState = RetractState.RETRACTING
                    viperArmSubsystem.setExtensionMotorGroupPower(-1.0)
                }
            }
            RetractState.RETRACTING -> {
                println("Still retracting...")
                if (viperArmSubsystem.isExtensionHome) {
                    println("RetractHome has reached home position, finishing...")
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