package org.firstinspires.ftc.teamcode.command.specimen

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem

class HighChamberPosition(private val gripperSubsystem: GripperSubsystem) : CommandBase() {
    init {
        addRequirements(gripperSubsystem)
    }

    private var currentState = GripperHeightState.IDLE
    private var targetTimeMs = 0.0

    override fun initialize() {
        targetTimeMs = 1000.0
        currentState = GripperHeightState.STARTED
    }

    override fun execute() {
        //State Machine
        when (currentState) {
            GripperHeightState.STARTED -> {
                gripperSubsystem.setHighChamberHeight()
                currentState = GripperHeightState.RAISING
            }
            GripperHeightState.RAISING -> {
                if (gripperSubsystem.gripperTop()) {
                    currentState = GripperHeightState.FINISHED
                }
            }
            else -> {
                //Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == GripperHeightState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            gripperSubsystem.setPickupHeight()
        }
    }

    companion object {
        enum class GripperHeightState {
            IDLE,
            STARTED,
            RAISING,
            LOWERING,
            FINISHED
        }
    }
}
