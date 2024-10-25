package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class WristToPickup(private val activeIntakeSubsystem: ActiveIntakeSubsystem) : CommandBase() {
    init {
        addRequirements(activeIntakeSubsystem)
    }

    private var currentState = WristState.IDLE
    private var targetTimeMs = 0.0
    private val runtime = ElapsedTime()

    override fun initialize() {
        targetTimeMs = 500.0
        currentState = WristState.STARTED
    }

    override fun execute() {
        when (currentState) {
            WristState.STARTED -> {
                runtime.reset()
                activeIntakeSubsystem.rotateHome()
                currentState = WristState.ROTATING
            }
            WristState.ROTATING -> {
                if (runtime.isTimedOut(targetTimeMs)) {
                    currentState = WristState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == WristState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        currentState = WristState.IDLE
    }

    companion object {
        enum class WristState {
            IDLE,
            STARTED,
            ROTATING,
            FINISHED
        }
    }
}