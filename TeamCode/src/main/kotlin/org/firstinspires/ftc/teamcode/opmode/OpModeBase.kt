package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperExtensionSubsystem

abstract class OpModeBase : CommandOpMode() {
    lateinit var hardware: HardwareManager
    lateinit var mecanumDrive: MecanumDrive
    lateinit var driverGamepad: GamepadEx
    lateinit var accessoryGamepad: GamepadEx
    lateinit var multiTelemetry: MultipleTelemetry

    // Subsystems
    lateinit var activeIntakeSubsystem: ActiveIntakeSubsystem
    lateinit var gripperSubsystem: GripperSubsystem
    lateinit var armRotationSubsystem: ArmRotationSubsystem
    lateinit var armExtensionSubsystem: ViperExtensionSubsystem

    fun initHardware() {
        hardware = HardwareManager(hardwareMap)
        // TODO: Start position?
        mecanumDrive = MecanumDrive(hardware, Pose2d(0.0, 0.0, 0.0))
        multiTelemetry = MultipleTelemetry(telemetry)

        activeIntakeSubsystem = ActiveIntakeSubsystem(hardware)
        gripperSubsystem = GripperSubsystem(hardware)
        armRotationSubsystem = ArmRotationSubsystem(hardware)
        armExtensionSubsystem = ViperExtensionSubsystem(hardware)

        driverGamepad = GamepadEx(gamepad1)
        accessoryGamepad = GamepadEx(gamepad2)
    }
}