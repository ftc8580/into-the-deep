package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.actions.internal.ActionState
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

// TODO: Can we read voltage (or something) to know if we no longer have a sample in the intake?
class DeliverSample(
    private val intakeSubsystem: ActiveIntakeSubsystem,
    private val timeout: Double = 1000.0,
    private val power: Double? = null
) : Action {
    private var currentState = ActionState.IDLE
    private var elapsedTime = ElapsedTime()

    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            ActionState.IDLE -> {
                elapsedTime.reset()
                intakeSubsystem.runEject(power)
                currentState = ActionState.STARTED
                return true
            }

            ActionState.STARTED -> {
                if (elapsedTime.isTimedOut(timeout)) {
                    intakeSubsystem.stopIntake()
                    currentState = ActionState.FINISHED
                }
                return true
            }

            ActionState.FINISHED -> {
                currentState = ActionState.IDLE
                return false
            }
        }
    }
}