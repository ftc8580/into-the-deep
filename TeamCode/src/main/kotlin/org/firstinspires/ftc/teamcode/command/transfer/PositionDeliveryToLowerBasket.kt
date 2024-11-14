package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class PositionDeliveryToLowerBasket(armRotationSubsystem: ArmRotationSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem): SequentialCommandGroup() {
    init {
        addCommands(
            SequentialCommandGroup(
                RotateTo(armRotationSubsystem, ArmRotationSubsystem.ROTATION_MAX_POSITION),
                ParallelCommandGroup(
                    ExtendTo(armRotationSubsystem, ArmRotationSubsystem.EXTENSION_LOWER_BASKET_POSITION),
                    WristToDeliver(activeIntakeSubsystem),
                )
            )
        )
        addRequirements(armRotationSubsystem, activeIntakeSubsystem)
    }
}