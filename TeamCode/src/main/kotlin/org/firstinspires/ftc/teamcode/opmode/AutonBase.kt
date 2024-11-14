package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem

abstract class AutonBase : LinearOpMode() {
    protected lateinit var hardware: HardwareManager
    protected lateinit var drive: MecanumDrive

    protected lateinit var activeIntakeSubsystem: ActiveIntakeSubsystem
    protected lateinit var gripperSubsystem: GripperSubsystem
    protected lateinit var armRotationSubsystem: ArmRotationSubsystem

    protected fun initialize(initialPose: Pose2d) {
        hardware = HardwareManager(hardwareMap)
        drive = MecanumDrive(hardware, initialPose)

        activeIntakeSubsystem = ActiveIntakeSubsystem(hardware)
        gripperSubsystem = GripperSubsystem(hardware)
        armRotationSubsystem = ArmRotationSubsystem(hardware)
    }
}