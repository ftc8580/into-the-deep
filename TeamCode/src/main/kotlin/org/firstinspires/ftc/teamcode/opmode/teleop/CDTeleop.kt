package org.firstinspires.ftc.teamcode.opmode.teleop

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.actions.Ascend
import org.firstinspires.ftc.teamcode.actions.AscensionTarget
import org.firstinspires.ftc.teamcode.actions.buildDriveArmPositionAction
import org.firstinspires.ftc.teamcode.actions.buildHighDeliveryArmPositionAction
import org.firstinspires.ftc.teamcode.actions.buildLowDeliveryArmPositionAction
import org.firstinspires.ftc.teamcode.actions.buildPickupArmPositionAction
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight
import org.firstinspires.ftc.teamcode.subsystem.WristRotationPosition
import kotlin.math.pow

@Suppress("UNUSED")
@TeleOp(name="CDTeleop")
class CDTeleop : OpModeBase() {
    private var driveSpeedScale = DRIVE_SPEED_FAST

    private var extensionGroupState = MotorGroupState.STOPPED
    private var rotationGroupState = MotorGroupState.STOPPED

    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = mutableListOf()

    override fun initialize() {
        initHardware()
        initializeDriverGamepad(driverGamepad)
        initializeCoDriverGamepad(accessoryGamepad)
    }

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
            hardware.intakeWheelServoRear?.power = -0.6
            hardware.intakeWheelServoFront?.power = 0.6
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
        }

        if (accessoryGamepad.leftY > VARIABLE_INPUT_DEAD_ZONE || accessoryGamepad.leftY < -VARIABLE_INPUT_DEAD_ZONE) {
            rotationGroupState = MotorGroupState.ACTIVE
            armRotationSubsystem.setRotationMotorGroupPower((-accessoryGamepad.leftY).pow(3.0) * 0.6)
        } else if (rotationGroupState == MotorGroupState.ACTIVE) {
            rotationGroupState = MotorGroupState.STOPPED
            armRotationSubsystem.setRotationMotorGroupPower(0.0)
            armRotationSubsystem.correctRotationGroupFollower()
        }

        // Preset buttons

//        if (gamepad2.a) {
//            runningActions.add(armSubsystems.buildDriveArmPositionAction())
//        } else if (gamepad2.b) {
//            runningActions.add(armSubsystems.buildPickupArmPositionAction())
//        } else if (gamepad2.x) {
//            runningActions.add(armSubsystems.buildLowDeliveryArmPositionAction())
//        } else if (gamepad2.y) {
//            runningActions.add(armSubsystems.buildHighDeliveryArmPositionAction())
//        }

        // Require both bumpers to be pressed in order to start ascent
        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            runningActions.add(Ascend(climbSubsystem, armExtensionSubsystem, armRotationSubsystem, AscensionTarget.LEVEL_2))
        }

        // update running actions
        val newActions: MutableList<Action> = mutableListOf()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        dash.sendTelemetryPacket(packet)

        writeTelemetry()
    }

    private fun initializeDriverGamepad(gamepad: GamepadEx) {
        val speedFastButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)
        val speedSlowButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val normalDriveButton = gamepad.getGamepadButton(GamepadKeys.Button.B)
        val gripperPickupButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val gripperLowDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val gripperHighDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)

        speedFastButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_FAST })
        speedSlowButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_SLOW })
        normalDriveButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_NORMAL})

        gripperPickupButton.whenPressed(Runnable { gripperSubsystem.set(GripperHeight.HOME) })
        gripperLowDeliveryButton.whenPressed(Runnable { gripperSubsystem.set(GripperHeight.HOME) })
        gripperHighDeliveryButton.whenPressed(Runnable { gripperSubsystem.set(GripperHeight.HIGH) })
    }

    private fun initializeCoDriverGamepad(gamepad: GamepadEx) {
        val wristPickupButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val wristDeliverButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val wristLeftButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val wristRightButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)

        wristPickupButton.whenPressed(Runnable { activeIntakeSubsystem.set(WristRotationPosition.PICKUP) })
        wristDeliverButton.whenPressed(Runnable { activeIntakeSubsystem.set(WristRotationPosition.DELIVER) })

        wristLeftButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementDown() })
        wristRightButton.whileHeld(Runnable { activeIntakeSubsystem.rotateIncrementUp() })
    }

    private fun writeTelemetry() {
        telemetry.addLine()
        telemetry.addLine("speed mult: $driveSpeedScale")
        telemetry.addLine()

        hardware.viperExtensionMotorGroup?.let {
            telemetry.addLine("viperExtensionPos: ${armExtensionSubsystem.extensionPositions}")
        } ?: telemetry.addLine("[WARNING] viperExtensionGroup not found")

        hardware.extensionHomeSensor?.let {
            telemetry.addLine("extensionHomeSensor pressed?: ${it.isPressed}")
        } ?: telemetry.addLine("[WARNING] extension home sensor not found")

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