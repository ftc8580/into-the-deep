package org.firstinspires.ftc.teamcode.opmode.teleop

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import kotlin.math.pow

@Suppress("UNUSED")
@TeleOp(name="CDTeleop")
class CDTeleop : OpModeBase() {
    private var driveSpeedScale = DRIVE_SPEED_NORMAL

    override fun initialize() {
        initHardware()
        initializeDriverGamepad(driverGamepad)
        initializeCoDriverGamepad(accessoryGamepad)

        viperArmSubsystem.setMotorGroupsRawPower()
    }

    @SuppressLint("UseValueOf")
    override fun run() {
        super.run()

        mecanumDrive.setDrivePower(
            Pose2d(
                driverGamepad.leftY * driveSpeedScale,
                -driverGamepad.leftX * driveSpeedScale,
                -driverGamepad.rightX * driveSpeedScale
            )
        )

        mecanumDrive.updatePoseEstimate()

        if (viperArmSubsystem.isExtensionHome) {
            viperArmSubsystem.resetExtensionEncoder()
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
            // TODO: Fix multiplier later
            viperArmSubsystem.setExtensionMotorGroupPower((-accessoryGamepad.rightY).pow(3.0) * 0.75)
        } else {
            viperArmSubsystem.setExtensionMotorGroupPower(0.0)
            viperArmSubsystem.correctExtensionGroupFollower()
        }

        if (accessoryGamepad.leftY > VARIABLE_INPUT_DEAD_ZONE || accessoryGamepad.leftY < -VARIABLE_INPUT_DEAD_ZONE) {
            // TODO: Fix multiplier later
            viperArmSubsystem.setRotationMotorGroupPower((-accessoryGamepad.leftY).pow(3.0) * 0.5)
        } else {
            viperArmSubsystem.setRotationMotorGroupPower(0.0)
            viperArmSubsystem.correctRotationGroupFollower()
        }

        writeTelemetry()
    }

    private fun initializeDriverGamepad(gamepad: GamepadEx) {
        val speedFastButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)
        val speedSlowButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val normalDriveButton = gamepad.getGamepadButton(GamepadKeys.Button.B)
        val gripperUpButton = gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        val gripperDownButton = gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        gripperUpButton.whenPressed(Runnable { gripperSubsystem.incrementUp() })
        gripperDownButton.whenPressed(Runnable { gripperSubsystem.incrementDown() })
//        val gripperPickupButton = gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//        val gripperLowDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//        val gripperHighDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)

        val retractForClimbButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val ratchetClimbButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        speedFastButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_FAST })
        speedSlowButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_SLOW })
        normalDriveButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_NORMAL})

        // TODO: Set correct numbers from telemetry
        retractForClimbButton.whenPressed(Runnable { viperArmSubsystem.extendToPosition(0) })
        ratchetClimbButton.whenPressed(Runnable {
            viperArmSubsystem.rotateToPosition(0)
            viperArmSubsystem.extendToPosition(0)
            viperArmSubsystem.rotateToPosition(200)
        })
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

        wristForwardButton.whenPressed(Runnable { activeIntakeSubsystem.rotateHome() })
        wristReverseButton.whenPressed(Runnable { activeIntakeSubsystem.rotateToBasket() })
//        wristLeftButton.whenPressed(Runnable { activeIntakeSubsystem.rotateLeft() })
//        wristRightButton.whenPressed(Runnable { activeIntakeSubsystem.rotateRight() })

        wristLeftButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementDown() })
        wristRightButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementUp() })

        viperDrivePositionButton.whenPressed(Runnable { viperArmSubsystem.drivePosition() })
        viperPickupPositionButton.whenPressed(Runnable { viperArmSubsystem.pickupPosition() })
        viperLowPositionButton.whenPressed(Runnable { viperArmSubsystem.deliverLowerBasket() })
        viperHighPositionButton.whenPressed(Runnable { viperArmSubsystem.deliverTopBasket() })
    }

    private fun writeTelemetry() {
        telemetry.addLine()
        telemetry.addLine("speed mult: $driveSpeedScale")
        telemetry.addLine()

        // TODO: Comment out telemetry we only need for troubleshooting

        // testing - added poslist
        hardware.viperExtensionMotorGroup?.let {
            telemetry.addLine("viperExtensionPos: ${viperArmSubsystem.getExtensionMotorGroupPosition()}")
            telemetry.addLine("viperExtensionPosList: ${viperArmSubsystem.getExtensionMotorGroupPositionList()}")
        } ?: telemetry.addLine("[WARNING] viperExtensionGroup not found")

        //testing added leftspeed
        hardware.viperRotationMotorLeft?.let {
            telemetry.addLine("ViperRotationMotorLeftSpeed: ${it.get()}")
        } ?: telemetry.addLine("[WARNING] viperrotationLEFTspeed not found")

        //testing added rightspeed
        hardware.viperRotationMotorRight?.let {
            telemetry.addLine("ViperRotationMotorRightSpeed: ${it.get()}")
        } ?: telemetry.addLine("[WARNING] viperrotationRIGHTspeed not found")

        //testing added groupspeed and poslist
        hardware.viperRotationMotorGroup?.let {
            telemetry.addLine("viperRotationGroupSpeed: ${it.get()}")
            telemetry.addLine("viperRotationPos: ${viperArmSubsystem.getRotationMotorGroupPosition()}")
            telemetry.addLine("viperRotationPosList: ${viperArmSubsystem.getRotationMotorGroupPositionList()}")
        } ?: telemetry.addLine("[WARNING] viperRotationGroup not found")
        telemetry.addLine("inputPower: ${(-accessoryGamepad.leftY).pow(3.0) * 0.3}")

        hardware.intakeRotateServo?.let {
            telemetry.addLine("intakeRotationPosition: ${it.position ?: "null"}")
        } ?: telemetry.addLine("[WARNING] wrist servo not found")

        hardware.extensionHomeSensor?.let {
            telemetry.addLine("extensionHomeSensor pressed?: ${it.isPressed}")
        } ?: telemetry.addLine("[WARNING] extension home sensor not found")

        hardware.gripperServo?.let {
            telemetry.addLine("gripperServo position: ${it.position}")
        } ?: telemetry.addLine("[WARNING] gripper servo not found")

        telemetry.update()
    }

    companion object {
        private const val VARIABLE_INPUT_DEAD_ZONE = 0.05

        private const val DRIVE_SPEED_FAST = 0.9
        private const val DRIVE_SPEED_NORMAL = 0.75
        private const val DRIVE_SPEED_SLOW = 0.5
    }
}