package org.firstinspires.ftc.teamcode.command.specimen

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem

class HighChamberPosition(private val gripperSubsystem: GripperSubsystem) : CommandBase() {
    init {
        addRequirements(gripperSubsystem)
    }
    
    override fun initialize() {
        gripperSubsystem.setHighChamberHeight()
    }
}