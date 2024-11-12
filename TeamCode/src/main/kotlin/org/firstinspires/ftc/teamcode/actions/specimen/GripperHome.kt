package org.firstinspires.ftc.teamcode.actions.specimen

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.util.isTimedOut

class GripperHome(private val gripperSubsystem: GripperSubsystem) : Action {
    private var currentState = GripperState.IDLE
    private var targetTimeMs = 1000.0
    private var elapsedTime = ElapsedTime()

    override fun run(p: TelemetryPacket): Boolean {
        when (currentState) {
            GripperState.IDLE -> {
                elapsedTime.reset()
                gripperSubsystem.setPickupHeight()
                currentState = GripperState.STARTED
                return true
            }

            GripperState.STARTED -> {
                if (elapsedTime.isTimedOut(targetTimeMs)) {
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
}