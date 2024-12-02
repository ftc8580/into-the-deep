package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.actions.internal.ActionState
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition
import org.firstinspires.ftc.teamcode.util.isTimedOut

class WristPosition(
    private val intakeSubsystem: ActiveIntakeSubsystem,
    private val target: WristRotationPosition,
    private val delayMs: Double = 0.0
) : Action {
    private var currentState = ActionState.IDLE
    private var elapsedTime = ElapsedTime()

    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            ActionState.IDLE -> {
                if (!elapsedTime.isTimedOut(delayMs)) return true

                intakeSubsystem.set(target)
                currentState = ActionState.STARTED
                return true
            }

            ActionState.STARTED -> {
                currentState = ActionState.FINISHED
                return true
            }

            ActionState.FINISHED -> {
                currentState = ActionState.IDLE
                return false
            }
        }
    }
}