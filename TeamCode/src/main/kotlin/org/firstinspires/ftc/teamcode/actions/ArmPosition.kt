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
    val currentExtensionPosition = extensionSubsystem.currentPosition ?: 0
    val currentRotationPosition = rotationSubsystem.currentPosition ?: 0
    val currentWristPosition = intakeSubsystem.currentPosition ?: 0.0

    val extensionDelta = extensionTarget.position - currentExtensionPosition
    val rotationDelta = rotationTarget.position - currentRotationPosition
    val wristDelta = wristTarget.position - currentWristPosition

    // TODO: How to calculate appropriate delays for the various systems based on current position and target position
    return ParallelAction(
        ExtensionPosition(extensionSubsystem, extensionTarget),
        RotationPosition(rotationSubsystem, rotationTarget),
        WristPosition(intakeSubsystem, wristTarget)
    )
}

// Returns the height of the end of the arm above the floor in MM
private fun armAboveFloorHeight(): Double {
    TODO("Write the function")
}