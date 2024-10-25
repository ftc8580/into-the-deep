package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class PositionPickup(viperArmSubsystem: ViperArmSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            RotateTo(viperArmSubsystem, ViperArmSubsystem.ROTATION_PICKUP_POSITION),
            ExtendTo(viperArmSubsystem, ViperArmSubsystem.EXTENSION_PICKUP_POSITION)
//            ParallelCommandGroup(
//                WristToPickup(activeIntakeSubsystem),
//                ExtendTo(viperArmSubsystem, ViperArmSubsystem.EXTENSION_PICKUP_POSITION)
//            ),
//            RotateTo(viperArmSubsystem, ViperArmSubsystem.ROTATION_PICKUP_POSITION)
        )
        addRequirements(viperArmSubsystem, activeIntakeSubsystem)
    }
}