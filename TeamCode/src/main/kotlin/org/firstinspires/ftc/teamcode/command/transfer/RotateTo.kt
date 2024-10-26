package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class RotateTo(private val viperArmSubsystem: ViperArmSubsystem, private val target: Int) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = RotateState.IDLE
    private var rotationDirection = RotationDirection.UP

    override fun initialize() {
        currentState = RotateState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RotateState.STARTED -> {
                if (-target > viperArmSubsystem.getRotationMotorGroupPosition()) {
                    rotationDirection = RotationDirection.DOWN
                }

                val power = if (rotationDirection == RotationDirection.UP) {
                    -0.6
                } else {
                    0.6
                }

                println("RotateState.STARTED")
                println("direction: $rotationDirection")
                println("power: $power")
                println("target: $target")

                viperArmSubsystem.setRotationMotorGroupPower(power)
                currentState = RotateState.ROTATING
            }
            RotateState.ROTATING -> {
                println("RotateState.ROTATING")
                println("position: ${viperArmSubsystem.getRotationMotorGroupPosition()}")

                if (rotationDirection == RotationDirection.UP && viperArmSubsystem.getRotationMotorGroupPosition() <= -target) {
                    println("rotation up reached target")
                    viperArmSubsystem.setRotationMotorGroupPower(0.0)
                    currentState = RotateState.FINISHED
                } else if (rotationDirection == RotationDirection.DOWN && viperArmSubsystem.getRotationMotorGroupPosition() >= -target) {
                    println("rotation down reached target")
                    viperArmSubsystem.setRotationMotorGroupPower(0.0)
                    currentState = RotateState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        println("RotateState.FINISHED")
        return currentState == RotateState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            viperArmSubsystem.setRotationMotorGroupPower(0.0)
        }

        currentState = RotateState.IDLE
    }

    companion object {
        enum class RotateState {
            IDLE,
            STARTED,
            ROTATING,
            FINISHED
        }

        enum class RotationDirection {
            UP,
            DOWN
        }
    }
}