package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

class PositionAutoRung(armRotationSubsystem: ArmRotationSubsystem, activeIntakeSubsystem: ActiveIntakeSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            //WristToPickup(activeIntakeSubsystem),
            RetractHome(armRotationSubsystem),
            ParallelCommandGroup(
                RotateTo(armRotationSubsystem, ArmRotationSubsystem.ROTATION_AUTORUNG_POSITION),
            )
        )
        addRequirements(armRotationSubsystem, activeIntakeSubsystem)
    }
}