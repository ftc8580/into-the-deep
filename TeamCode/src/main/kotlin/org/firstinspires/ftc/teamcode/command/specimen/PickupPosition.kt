package org.firstinspires.ftc.teamcode.command.specimen

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class PickupPosition(private val gripperSubsystem: GripperSubsystem) : CommandBase() {
    init {
        addRequirements(gripperSubsystem)
    }

    private var currentState = GripperHeightState.IDLE
    private var targetTimeMs = 0.0
    private val runtime = ElapsedTime()

    override fun initialize() {
        targetTimeMs = 1000.0
        currentState = GripperHeightState.STARTED
    }

    override fun execute() {
        //State Machine
        when (currentState) {
            GripperHeightState.STARTED -> {
                runtime.reset()
                gripperSubsystem.setPickupHeight()
                currentState = GripperHeightState.LOWERING
            }
            GripperHeightState.LOWERING -> {
                if (runtime.isTimedOut(targetTimeMs) || gripperSubsystem.gripperBottom()) {
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
