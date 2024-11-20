package org.firstinspires.ftc.teamcode.opmode.teleop

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.ExtensionPosition
import org.firstinspires.ftc.teamcode.actions.RotationPosition
import org.firstinspires.ftc.teamcode.actions.WristPosition
import org.firstinspires.ftc.teamcode.commands.ActionCommand
//import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToLowerBasket
//import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToUpperBasket
//import org.firstinspires.ftc.teamcode.command.transfer.PositionDrive
//import org.firstinspires.ftc.teamcode.command.transfer.PositionHome
//import org.firstinspires.ftc.teamcode.command.transfer.PositionPickup
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.subsystem.ArmExtensionPosition
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition
import org.firstinspires.ftc.teamcode.util.RevColor
import kotlin.math.pow

@Suppress("UNUSED")
@TeleOp(name="CDTeleop")
class CDTeleop : OpModeBase() {
    private var driveSpeedScale = DRIVE_SPEED_NORMAL
    private var revColorSensor: RevColor? = null

    private var extensionGroupState = MotorGroupState.STOPPED
    private var rotationGroupState = MotorGroupState.STOPPED

    override fun initialize() {
        initHardware()
        initializeDriverGamepad(driverGamepad)
        initializeCoDriverGamepad(accessoryGamepad)

        hardware.intakeColorSensor?.let {
            revColorSensor = RevColor(it)
        }
    }

    // Actions

//    private val armDrivePositionAction = ParallelAction(
//        ExtensionPosition(armExtensionSubsystem, ArmExtensionPosition.HOME),
//        RotationPosition(armRotationSubsystem, ArmRotationPosition.DRIVE),
//        WristPosition(activeIntakeSubsystem, WristRotationPosition.PICKUP, 500.0)
//    )

    @SuppressLint("UseValueOf")
    override fun run() {
        super.run()

        val packet = TelemetryPacket()

        mecanumDrive.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                    driverGamepad.leftY * driveSpeedScale,
                    -driverGamepad.leftX * driveSpeedScale
                ),
            -driverGamepad.rightX * driveSpeedScale
        ))

        mecanumDrive.updatePoseEstimate()

        if (armExtensionSubsystem.isExtensionHome) {
            armExtensionSubsystem.resetMotorEncoders()
        }

        revColorSensor?.let {
            hardware.intakeIndicatorLight?.position = if (it.isBlue) {
                0.645
            } else if (it.isRed) {
                0.278
            } else if (it.isYellow) {
                0.388
            } else {
                0.0
            }
        }

        // Driver controls
        val driverLeftTriggerValue = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val driverRightTriggerValue = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

        if (driverLeftTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.gripperServo?.let {
                if (it.position > 0.0) {
                    it.position -= 0.01
                }
            }
        }

        if (driverRightTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.gripperServo?.let {
                if (it.position < 1.0) {
                    it.position += 0.01
                }
            }
        }

        // Accessory controls
        val accessoryLeftTriggerValue = accessoryGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val accessoryRightTriggerValue = accessoryGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

        if (accessoryLeftTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.intakeWheelServoRear?.power = 1.0
            hardware.intakeWheelServoFront?.power = -1.0
        } else if (accessoryRightTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.intakeWheelServoRear?.power = -1.0
            hardware.intakeWheelServoFront?.power = 1.0
        } else {
            hardware.intakeWheelServoRear?.power = 0.0
            hardware.intakeWheelServoFront?.power = 0.0
        }

        if (accessoryGamepad.rightY > VARIABLE_INPUT_DEAD_ZONE || accessoryGamepad.rightY < -VARIABLE_INPUT_DEAD_ZONE) {
            extensionGroupState = MotorGroupState.ACTIVE
            armExtensionSubsystem.setExtensionMotorGroupPower((-accessoryGamepad.rightY).pow(3.0))
        } else if (extensionGroupState == MotorGroupState.ACTIVE) {
            extensionGroupState = MotorGroupState.STOPPED
            armExtensionSubsystem.setExtensionMotorGroupPower(0.0)
//            viperArmSubsystem.correctExtensionGroupFollower()
        }

        if (accessoryGamepad.leftY > VARIABLE_INPUT_DEAD_ZONE || accessoryGamepad.leftY < -VARIABLE_INPUT_DEAD_ZONE) {
            rotationGroupState = MotorGroupState.ACTIVE
            armRotationSubsystem.setRotationMotorGroupPower((-accessoryGamepad.leftY).pow(3.0) * 0.6)
        } else if (rotationGroupState == MotorGroupState.ACTIVE) {
            rotationGroupState = MotorGroupState.STOPPED
            armRotationSubsystem.setRotationMotorGroupPower(0.0)
            armRotationSubsystem.correctRotationGroupFollower()
        }

//        if (gamepad2.a) {
//            armDrivePositionAction.run(packet)
//        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        writeTelemetry()
    }

    private fun initializeDriverGamepad(gamepad: GamepadEx) {
        val speedFastButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)
        val speedSlowButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val normalDriveButton = gamepad.getGamepadButton(GamepadKeys.Button.B)
        val gripperPickupButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val gripperLowDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val gripperHighDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)

//        val retractForClimbButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//        val ratchetClimbButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        speedFastButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_FAST })
        speedSlowButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_SLOW })
        normalDriveButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_NORMAL})

        gripperPickupButton.whenPressed(Runnable { gripperSubsystem.set(GripperHeight.HOME) })
        gripperLowDeliveryButton.whenPressed(Runnable { gripperSubsystem.set(GripperHeight.LOW) })
        gripperHighDeliveryButton.whenPressed(Runnable { gripperSubsystem.set(GripperHeight.HIGH) })

        // TODO: Set correct numbers from telemetry
//        retractForClimbButton.whenPressed(Runnable { viperArmSubsystem.extendToPosition(0) })
//        ratchetClimbButton.whenPressed(Runnable {
//            viperArmSubsystem.rotateToPosition(0)
//            viperArmSubsystem.extendToPosition(0)
//            viperArmSubsystem.rotateToPosition(200)
//        })
    }

    private fun initializeCoDriverGamepad(gamepad: GamepadEx) {
        val wristPickupButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val wristDeliverButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val wristLeftButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val wristRightButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)

//        val armDrivePositionButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val armPickupPositionButton = gamepad.getGamepadButton(GamepadKeys.Button.B)
        val armLowPositionButton = gamepad.getGamepadButton(GamepadKeys.Button.X)
        val armHighPositionButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)
        val armHomeButton = gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)

        wristPickupButton.whenPressed(Runnable { activeIntakeSubsystem.set(WristRotationPosition.PICKUP) })
        wristDeliverButton.whenPressed(Runnable { activeIntakeSubsystem.set(WristRotationPosition.DELIVER) })

        wristLeftButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementDown() })
        wristRightButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementUp() })

        // TODO: Refactor this to use ArmPosition when ready
//        armDrivePositionButton.whenPressed(
//            ActionCommand(
//                ParallelAction(
//                    ExtensionPosition(armExtensionSubsystem, ArmExtensionPosition.HOME),
//                    RotationPosition(armRotationSubsystem, ArmRotationPosition.DRIVE),
//                    WristPosition(activeIntakeSubsystem, WristRotationPosition.PICKUP, 500.0)
//                )
//            )
//        )
        armPickupPositionButton.whenPressed(
            ActionCommand(
                ParallelAction(
                    ExtensionPosition(armExtensionSubsystem, ArmExtensionPosition.AUTON_PICKUP),
                    RotationPosition(armRotationSubsystem, ArmRotationPosition.AUTON_PICKUP, 200.0),
                    WristPosition(activeIntakeSubsystem, WristRotationPosition.PICKUP)
                )
            )
        )
        armLowPositionButton.whenPressed(
            ActionCommand(
                ParallelAction(
                    ExtensionPosition(armExtensionSubsystem, ArmExtensionPosition.LOW_BASKET),
                    RotationPosition(armRotationSubsystem, ArmRotationPosition.TOP),
                    WristPosition(activeIntakeSubsystem, WristRotationPosition.DELIVER, 200.0)
                )
            )
        )
        armHighPositionButton.whenPressed(
            ActionCommand(
                ParallelAction(
                    ExtensionPosition(armExtensionSubsystem, ArmExtensionPosition.MAX_UP),
                    RotationPosition(armRotationSubsystem, ArmRotationPosition.TOP),
                    WristPosition(activeIntakeSubsystem, WristRotationPosition.DELIVER, 200.0)
                )
            )
        )

        armHomeButton.whenPressed(
            ActionCommand(
                SequentialAction(
                    WristPosition(activeIntakeSubsystem, WristRotationPosition.PICKUP),
                    ParallelAction(
                        ExtensionPosition(armExtensionSubsystem, ArmExtensionPosition.HOME),
                        RotationPosition(armRotationSubsystem, ArmRotationPosition.HOME)
                    )
                )
            )
        )
    }

    private fun writeTelemetry() {
        telemetry.addLine()
        telemetry.addLine("speed mult: $driveSpeedScale")
        telemetry.addLine()

        // TODO: Comment out telemetry we only need for troubleshooting

        hardware.viperExtensionMotorGroup?.let {
            telemetry.addLine("viperExtensionPos: ${armExtensionSubsystem.extensionPositions}")
        } ?: telemetry.addLine("[WARNING] viperExtensionGroup not found")

        hardware.intakeRotateServo?.let {
            telemetry.addLine("intakeRotationPosition: ${it.position}")
        } ?: telemetry.addLine("[WARNING] wrist servo not found")

        hardware.extensionHomeSensor?.let {
            telemetry.addLine("extensionHomeSensor pressed?: ${it.isPressed}")
        } ?: telemetry.addLine("[WARNING] extension home sensor not found")

        hardware.rotationHomeSensor?.let {
            telemetry.addLine("rotationHomeSensor pressed?: ${it.isPressed}")
        } ?: telemetry.addLine("[WARNING] rotation home sensor not found")

        hardware.gripperServo?.let {
            telemetry.addLine("gripperServo position: ${it.position}")
        } ?: telemetry.addLine("[WARNING] gripper servo not found")

        hardware.armRotationEncoder?.let {
            telemetry.addLine("rotation position: ${it.getPositionAndVelocity().position}")
        } ?: telemetry.addLine("[WARNING] arm rotation encoder not found")

        telemetry.update()
    }

    companion object {
        private const val VARIABLE_INPUT_DEAD_ZONE = 0.1

        private const val DRIVE_SPEED_FAST = 0.9
        private const val DRIVE_SPEED_NORMAL = 0.75
        private const val DRIVE_SPEED_SLOW = 0.5

        enum class MotorGroupState {
            ACTIVE,
            STOPPED
        }
    }
}