package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class PositionPickup(armRotationSubsystem: ArmRotationSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            ParallelCommandGroup(
                WristToPickup(activeIntakeSubsystem),
                ExtendTo(armRotationSubsystem, ArmRotationSubsystem.EXTENSION_PICKUP_POSITION)
            ),
            RotateTo(armRotationSubsystem, ArmRotationSubsystem.ROTATION_PICKUP_POSITION)
        )
        addRequirements(armRotationSubsystem, activeIntakeSubsystem)
    }
}