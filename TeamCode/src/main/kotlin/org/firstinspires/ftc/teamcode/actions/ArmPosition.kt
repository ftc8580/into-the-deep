package org.firstinspires.ftc.teamcode.actions

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperExtensionSubsystem
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition

data class ArmSubsystems(
    val extensionSubsystem: ViperExtensionSubsystem,
    val rotationSubsystem: ArmRotationSubsystem,
    val intakeSubsystem: ActiveIntakeSubsystem
)

fun ArmSubsystems.buildArmPositionAction(
    extensionTarget: ArmExtensionPosition,
    rotationTarget: ArmRotationPosition,
    wristTarget: WristRotationPosition,
    extensionDelayMs: Double? = null,
): Action {
    val currentRotationPosition = rotationSubsystem.currentPosition ?: 0

    val extensionDelay = extensionDelayMs
        ?: if (currentRotationPosition > 500) {
            0.0
        } else {
            250.0
        }
    val wristDelay = if (currentRotationPosition < 200) 250.0 else 0.0

    return ParallelAction(
        ExtensionPosition(extensionSubsystem, extensionTarget, extensionDelay),
        RotationPosition(rotationSubsystem, rotationTarget),
        WristPosition(intakeSubsystem, wristTarget, wristDelay)
    )
}

fun ArmSubsystems.buildPreParkArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.HOME,
    ArmRotationPosition.DRIVE,
    WristRotationPosition.PICKUP,
    500.0
)

fun ArmSubsystems.buildParkArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.HOME,
    ArmRotationPosition.PARK,
    WristRotationPosition.PICKUP
)

fun ArmSubsystems.buildDriveArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.HOME,
    ArmRotationPosition.DRIVE,
    WristRotationPosition.PICKUP
)

fun ArmSubsystems.buildPrePickupArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.AUTON_PICKUP,
    ArmRotationPosition.DRIVE,
    WristRotationPosition.PICKUP
)

fun ArmSubsystems.buildPickupArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.AUTON_PICKUP,
    ArmRotationPosition.AUTON_PICKUP,
    WristRotationPosition.PICKUP
)

fun ArmSubsystems.buildLowDeliveryArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.LOW_BASKET,
    ArmRotationPosition.TOP,
    WristRotationPosition.DELIVER
)

fun ArmSubsystems.buildHighDeliveryArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.AUTON_MAX,
    ArmRotationPosition.AUTON_DELIVERY,
    WristRotationPosition.DELIVER
)

fun ArmSubsystems.buildHomeArmPositionAction(): Action = this.buildArmPositionAction(
    ArmExtensionPosition.HOME,
    ArmRotationPosition.HOME,
    WristRotationPosition.PICKUP
)