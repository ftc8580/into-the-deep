package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class PositionDrive(armRotationSubsystem: ArmRotationSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                RetractHome(armRotationSubsystem),
                WristToPickup(activeIntakeSubsystem),
            ),
            RotateTo(armRotationSubsystem, ArmRotationSubsystem.ROTATION_DRIVE_POSITION)
        )
        addRequirements(armRotationSubsystem, activeIntakeSubsystem)
    }
}