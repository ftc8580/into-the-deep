package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class IntakeSample(private val activeIntakeSubsystem: ActiveIntakeSubsystem) : CommandBase() {
    init {
        addRequirements(activeIntakeSubsystem)
    }

    private var currentState = IntakeState.IDLE
    private var targetTimeMs = 0.0
    private val runtime = ElapsedTime()

    override fun initialize() {
        runtime.reset()
        targetTimeMs = 1000.0

        activeIntakeSubsystem.runIntake()

        currentState = IntakeState.STARTED
    }

    override fun execute() {
        when (currentState) {
            IntakeState.STARTED -> {
                currentState = IntakeState.RUNNING
            }
            IntakeState.RUNNING -> {
                if (runtime.isTimedOut(targetTimeMs)) {
                    println("intake is timed out")
                    currentState = IntakeState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == IntakeState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        activeIntakeSubsystem.stopIntake()
        currentState = IntakeState.IDLE
    }

    companion object {
        enum class IntakeState {
            IDLE,
            STARTED,
            RUNNING,
            FINISHED
        }
    }
}