package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.Subsystem

class ActionCommand(private val action: Action, private val requirements: Set<Subsystem> = setOf()) : Command {
    private var isFinished: Boolean = false

    override fun getRequirements(): Set<Subsystem> = requirements

    override fun execute() {
        val packet = TelemetryPacket()
        action.preview(packet.fieldOverlay())
        isFinished = !action.run(packet)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }
}