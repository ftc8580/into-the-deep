package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class PositionHome(armRotationSubsystem: ArmRotationSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            RetractHome(armRotationSubsystem),
            ParallelCommandGroup(
                RotateHome(armRotationSubsystem),
                WristToPickup(activeIntakeSubsystem),
            )
        )
        addRequirements(armRotationSubsystem, activeIntakeSubsystem)
    }
}