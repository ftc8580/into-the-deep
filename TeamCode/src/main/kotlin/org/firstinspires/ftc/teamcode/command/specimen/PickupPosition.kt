package org.firstinspires.ftc.teamcode.command.specimen

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem

class PickupPosition(private val gripperSubsystem: GripperSubsystem) : CommandBase() {
    init {
        addRequirements(gripperSubsystem)
    }

    override fun initialize() {
        gripperSubsystem.setPickupHeight()
    }
}