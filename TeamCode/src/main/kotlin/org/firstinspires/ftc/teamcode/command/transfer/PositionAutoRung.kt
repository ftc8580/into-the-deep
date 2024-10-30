package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class PositionAutoRung(viperArmSubsystem: ViperArmSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            //WristToPickup(activeIntakeSubsystem),
            RetractHome(viperArmSubsystem),
            ParallelCommandGroup(
                RotateTo(viperArmSubsystem, ViperArmSubsystem.ROTATION_AUTORUNG_POSITION),
            )
        )
        addRequirements(viperArmSubsystem, activeIntakeSubsystem)
    }
}