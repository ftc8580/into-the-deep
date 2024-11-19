package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.actions.RotationPosition
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight

@Suppress("Unused")
@Autonomous(name = "Narwhal Specimen", group = "Narwhal")
class NarwhalSpecimenAuton : AutonBase() {
    private val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val action = drive.actionBuilder(
            Pose2d(40.0, 63.5, Math.toRadians(270.0))
        )
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .splineToConstantHeading(Vector2d(5.0, 33.0), Math.toRadians(270.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.3)
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(28.0, 40.0, Math.toRadians(200.0)), Math.toRadians(0.0))
            .splineToLinearHeading(Pose2d(23.5, 7.5, Math.toRadians(180.0)), Math.toRadians(180.0))
            .afterTime(0.0, RotationPosition(armRotationSubsystem, ArmRotationPosition.PARK))
            .waitSeconds(2.0)
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}