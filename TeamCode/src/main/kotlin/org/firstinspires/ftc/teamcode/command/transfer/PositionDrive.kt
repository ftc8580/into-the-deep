package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class PositionDrive(viperArmSubsystem: ViperArmSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                WristToPickup(activeIntakeSubsystem),
                RetractHome(viperArmSubsystem)
            ),
            RotateTo(viperArmSubsystem, ViperArmSubsystem.ROTATION_DRIVE_POSITION)
        )
        addRequirements(viperArmSubsystem, activeIntakeSubsystem)
    }
}