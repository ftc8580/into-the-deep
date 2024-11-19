package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class ArmPosition(
    private val armExtensionPosition: ArmExtensionPosition,
    private val armRotationSubsystem: ArmRotationSubsystem,
    private val extensionPosition: ArmExtensionPosition,
    private val rotationPosition: ArmRotationPosition
) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }
}