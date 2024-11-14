package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class GripperPosition(
    private val gripperSubsystem: GripperSubsystem, 
    private val height: GripperHeight, 
    private val timeout: Double = 1000.0
) : Action {
    private var currentState = GripperState.IDLE
    private var elapsedTime = ElapsedTime()
    
    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            GripperState.IDLE -> {
                elapsedTime.reset()
                gripperSubsystem.set(height)
                currentState = GripperState.STARTED
                return true
            }

            GripperState.STARTED -> {
                if (elapsedTime.isTimedOut(timeout)) {
                    currentState = GripperState.FINISHED
                }
                return true
            }

            GripperState.FINISHED -> {
                currentState = GripperState.IDLE
                return false
            }
        }
    }
    
    companion object {
        enum class GripperState {
            IDLE,
            STARTED,
            FINISHED
        }
    }
}