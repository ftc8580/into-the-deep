package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class PositionHome(viperArmSubsystem: ViperArmSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            RetractHome(viperArmSubsystem),
            ParallelCommandGroup(
                RotateHome(viperArmSubsystem),
                WristToPickup(activeIntakeSubsystem),
            )
        )
        addRequirements(viperArmSubsystem, activeIntakeSubsystem)
    }
}