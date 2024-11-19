package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperExtensionSubsystem
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition

fun buildArmPositionAction(
    extensionSubsystem: ViperExtensionSubsystem,
    rotationSubsystem: ArmRotationSubsystem,
    intakeSubsystem: ActiveIntakeSubsystem,
    extensionTarget: ArmExtensionPosition,
    rotationTarget: ArmRotationPosition,
    wristTarget: WristRotationPosition
): Action {
    val currentRotationPosition = rotationSubsystem.currentPosition ?: 0

    val extensionDelay = if (currentRotationPosition > 500) 0.0 else 250.0
    val wristDelay = if (currentRotationPosition < 200) 250.0 else 0.0

    return ParallelAction(
        ExtensionPosition(extensionSubsystem, extensionTarget, extensionDelay),
        RotationPosition(rotationSubsystem, rotationTarget),
        WristPosition(intakeSubsystem, wristTarget, wristDelay)
    )
}