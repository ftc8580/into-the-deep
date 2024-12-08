package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.actions.ArmSubsystems
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperExtensionSubsystem

abstract class AutonBase : LinearOpMode() {
    protected lateinit var hardware: HardwareManager
    protected lateinit var drive: MecanumDrive

    protected lateinit var activeIntakeSubsystem: ActiveIntakeSubsystem
    protected lateinit var gripperSubsystem: GripperSubsystem
    protected lateinit var armRotationSubsystem: ArmRotationSubsystem
    protected lateinit var armExtensionSubsystem: ViperExtensionSubsystem
    protected lateinit var armSubsystems: ArmSubsystems

    protected val fastSegmentVelocityConstraint = TranslationalVelConstraint(75.0)
    protected val midSegmentVelocityConstraint = TranslationalVelConstraint(60.0)
    protected val slowSegmentVelocityConstraint = TranslationalVelConstraint(50.0)
    protected val slowerSegmentVelocityConstraint = TranslationalVelConstraint(25.0)
    protected val slowestSegmentVelocityConstraint = TranslationalVelConstraint(23.50) //faster than this on tight curves is not accurate
    protected val slowSegmentAccelerationConstraint = ProfileAccelConstraint(-30.0, 45.0)
    protected val slowerSegmentAccelerationConstraint = ProfileAccelConstraint(-15.0, 22.0)

    protected fun initialize(initialPose: Pose2d) {
        hardware = HardwareManager(hardwareMap)
        drive = MecanumDrive(hardware, initialPose)

        activeIntakeSubsystem = ActiveIntakeSubsystem(hardware)
        gripperSubsystem = GripperSubsystem(hardware)
        armRotationSubsystem = ArmRotationSubsystem(hardware)
        armExtensionSubsystem = ViperExtensionSubsystem(hardware)

        armSubsystems = ArmSubsystems(
            armExtensionSubsystem,
            armRotationSubsystem,
            activeIntakeSubsystem
        )
    }
}