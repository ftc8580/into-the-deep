package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.actions.internal.ActionState
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class GripperPosition(
    private val gripperSubsystem: GripperSubsystem, 
    private val height: GripperHeight, 
    private val timeout: Double = 1000.0
) : Action {
    private var currentState = ActionState.IDLE
    private var elapsedTime = ElapsedTime()
    
    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            ActionState.IDLE -> {
                elapsedTime.reset()
                gripperSubsystem.set(height)
                currentState = ActionState.STARTED
                return true
            }

            ActionState.STARTED -> {
                if (elapsedTime.isTimedOut(timeout)) {
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