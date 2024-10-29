package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem
import kotlin.math.abs
import kotlin.properties.Delegates

class RotateTo(private val viperArmSubsystem: ViperArmSubsystem, private val target: Int) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = RotateState.IDLE
    private var rotationDirection = RotationDirection.UP

    private var power = -MAX_POWER
    private var initialPosition by Delegates.notNull<Double>()
    private var positionDelta by Delegates.notNull<Double>()

    override fun initialize() {
        currentState = RotateState.STARTED
        initialPosition = viperArmSubsystem.getRotationMotorGroupPosition()
        positionDelta = abs(target - initialPosition)
    }

    override fun execute() {
        when (currentState) {
            RotateState.STARTED -> {
                if (-target > initialPosition) {
                    rotationDirection = RotationDirection.DOWN
                }

                power = if (rotationDirection == RotationDirection.DOWN) {
                    -MAX_POWER
                } else {
                    MAX_POWER
                }

                viperArmSubsystem.setRotationMotorGroupPower(power)
                currentState = RotateState.ROTATING
            }
            RotateState.ROTATING -> {
                val currentPosition = viperArmSubsystem.getRotationMotorGroupPosition()

                if (rotationDirection == RotationDirection.UP) {
                    val slowTarget = target - (positionDelta * 0.1)
                    if (currentPosition <= -slowTarget) {
                        viperArmSubsystem.setRotationMotorGroupPower(power * LOW_POWER_MULT)
                    }
                } else if (rotationDirection == RotationDirection.DOWN) {
                    val slowTarget = target + (positionDelta * 0.1)
                    if (currentPosition >= -slowTarget) {
                        viperArmSubsystem.setRotationMotorGroupPower(power * LOW_POWER_MULT)
                    }
                }
                if (rotationDirection == RotationDirection.UP && currentPosition <= -target) {
                    println("rotation up reached target")
                    currentState = RotateState.FINISHED
                } else if (rotationDirection == RotationDirection.DOWN && currentPosition >= -target) {
                    println("rotation down reached target")
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

        currentState = RotateState.IDLE
    }

    companion object {
        private const val MAX_POWER = 0.7
        private const val LOW_POWER_MULT = 0.5

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