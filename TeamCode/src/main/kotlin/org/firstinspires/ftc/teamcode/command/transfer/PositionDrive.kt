package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class PositionDrive(viperArmSubsystem: ViperArmSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            RotateTo(viperArmSubsystem, ViperArmSubsystem.ROTATION_DRIVE_POSITION),
            ParallelCommandGroup(
                RetractHome(viperArmSubsystem),
                WristToPickup(activeIntakeSubsystem),
            )
        )
        addRequirements(viperArmSubsystem, activeIntakeSubsystem)
    }
}