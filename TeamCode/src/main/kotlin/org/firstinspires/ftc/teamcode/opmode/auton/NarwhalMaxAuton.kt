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
@Autonomous(name = "Narwhal Max", group = "Narwhal")
class NarwhalMaxAuton : AutonBase() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

        initialize(initialPose)

        val action = drive.actionBuilder(initialPose)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH)) // Raise gripper
            .waitSeconds(0.8) // Wait long enough for gripper to reach max position before arriving at submersible
            .splineToConstantHeading(Vector2d(5.0, 33.0), Math.toRadians(270.0)) // Drive to submersibile delivery position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME)) // Lower gripper
            .waitSeconds(0.3) // Wait long enough to hook specimen before driving away
            .setTangent(Math.toRadians(90.0)) // Start spline in 90deg direction
            .splineToConstantHeading(Vector2d(48.5, 33.0), Math.toRadians(270.0)) // Spline to first sample pickup position
            .waitSeconds(1.0)
            .lineToY(36.0)
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(52.0, 52.0, Math.toRadians(225.0)), Math.toRadians(45.0))
            .waitSeconds(1.0)
            .splineToLinearHeading(Pose2d(58.5, 33.0, Math.toRadians(270.0)), Math.toRadians(270.0))
            .waitSeconds(1.0)
            .lineToY(36.0)
            .setTangent(90.0)
            .splineToLinearHeading(Pose2d(52.0, 52.0, Math.toRadians(225.0)), Math.toRadians(45.0))
            .waitSeconds(1.0)
            .splineToLinearHeading(Pose2d(58.5, 33.0, Math.toRadians(325.0)), Math.toRadians(270.0))
            .waitSeconds(1.0)
            .lineToY(36.0)
            .setTangent(45.0)
            .splineToLinearHeading(Pose2d(52.0, 52.0, Math.toRadians(225.0)), Math.toRadians(45.0))
            .waitSeconds(1.0)
            .setTangent(225.0)
            .splineToLinearHeading(Pose2d(23.5, 12.0, Math.toRadians(180.0)), Math.toRadians(180.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}