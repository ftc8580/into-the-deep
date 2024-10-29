package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem

class WristToPickup(private val activeIntakeSubsystem: ActiveIntakeSubsystem) : CommandBase() {
    init {
        addRequirements(activeIntakeSubsystem)
    }

    override fun initialize() {
        activeIntakeSubsystem.rotateHome()
    }
}