package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class PositionDeliveryToUpperBasket(viperArmSubsystem: ViperArmSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            SequentialCommandGroup(
                RotateTo(viperArmSubsystem, ViperArmSubsystem.ROTATION_MAX_POSITION),
                ParallelCommandGroup(
                    ExtendTo(viperArmSubsystem, ViperArmSubsystem.EXTENSION_MAX_POSITION),
                    WristToDeliver(activeIntakeSubsystem),
                )
            )
        )
        addRequirements(viperArmSubsystem, activeIntakeSubsystem)
    }
}