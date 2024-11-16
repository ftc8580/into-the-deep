package org.firstinspires.ftc.teamcode.opmode.teleop

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToLowerBasket
//import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToUpperBasket
//import org.firstinspires.ftc.teamcode.command.transfer.PositionDrive
//import org.firstinspires.ftc.teamcode.command.transfer.PositionHome
//import org.firstinspires.ftc.teamcode.command.transfer.PositionPickup
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
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

    @SuppressLint("UseValueOf")
    override fun run() {
        super.run()

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

        // LED Light
        // hardware.intakeColorSensor?.argb()

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
        val wristForwardButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val wristReverseButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val wristLeftButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val wristRightButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)

        val viperDrivePositionButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val viperPickupPositionButton = gamepad.getGamepadButton(GamepadKeys.Button.B)
        val viperLowPositionButton = gamepad.getGamepadButton(GamepadKeys.Button.X)
        val viperHighPositionButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)

        val homeButton = gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)

        wristForwardButton.whenPressed(Runnable { activeIntakeSubsystem.rotateHome() })
        wristReverseButton.whenPressed(Runnable { activeIntakeSubsystem.rotateToBasket() })

        wristLeftButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementDown() })
        wristRightButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementUp() })

//        viperDrivePositionButton.whenPressed(PositionDrive(armRotationSubsystem, activeIntakeSubsystem))
//        viperPickupPositionButton.whenPressed(PositionPickup(armRotationSubsystem, activeIntakeSubsystem))
//        viperLowPositionButton.whenPressed(PositionDeliveryToLowerBasket(armRotationSubsystem, activeIntakeSubsystem))
//        viperHighPositionButton.whenPressed(PositionDeliveryToUpperBasket(armRotationSubsystem, activeIntakeSubsystem))
//
//        homeButton.whenPressed(PositionHome(armRotationSubsystem, activeIntakeSubsystem))
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