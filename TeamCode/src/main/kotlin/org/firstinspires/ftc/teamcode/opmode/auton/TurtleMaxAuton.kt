package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight

@Suppress("Unused")
@Autonomous(name = "Turtle Max", group = "Turtle")
class TurtleMaxAuton : AutonBase() {
    private val initialPose = Pose2d(-8.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

//        val trajectory = drive.actionBuilder(
//            Pose2d(-8.0, 63.0, Math.toRadians(270.0))
//        )
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
//            .waitSeconds(1.2)
//            .lineToY(33.0)
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
//            .waitSeconds(0.3)
//            .lineToY(38.0)
//            .splineToConstantHeading(Vector2d(-37.0, 24.0), Math.toRadians(270.0))
//            .setTangent(Math.toRadians(270.0))
//            .splineToConstantHeading(Vector2d(-47.5, 14.0), Math.toRadians(90.0))
//            .strafeToConstantHeading(Vector2d(-47.5, 53.0))
//            .strafeToConstantHeading(Vector2d(-47.5, 14.0))
//            .strafeToConstantHeading(Vector2d(-57.5, 14.0))
//            .strafeToConstantHeading(Vector2d(-57.5, 53.0))
////            .strafeToConstantHeading(Vector2d(-57.5, 12.0))
////            .strafeToConstantHeading(Vector2d(-63.0, 12.0))
////            .strafeToConstantHeading(Vector2d(-63.0, 53.0))
////            .strafeToConstantHeading(Vector2d(-63.0, 48.0))
//            .strafeToLinearHeading(Vector2d(-40.0, 48.0), Math.toRadians(90.0))
//            .waitSeconds(1.0)
//            .strafeToConstantHeading(Vector2d(-40.0, 63.0))
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
//            .waitSeconds(0.5)
//            .setTangent(0.0)
//            .splineToSplineHeading(Pose2d(-6.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
//            .waitSeconds(0.3)
//            .strafeToConstantHeading(Vector2d(-6.0, 38.0))
//            .setTangent(Math.toRadians(90.0))
//            .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
//            .waitSeconds(1.0)
//            .strafeToConstantHeading(Vector2d(-40.0, 63.0))
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
//            .waitSeconds(0.5)
//            .setTangent(0.0)
//            .splineToSplineHeading(Pose2d(-4.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
//            .waitSeconds(0.3)
//            .strafeToConstantHeading(Vector2d(-4.0, 38.0))
//            .setTangent(Math.toRadians(90.0))
//            .splineToLinearHeading(Pose2d(-40.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
//            .build()

        val trajectory = drive.actionBuilder(
            Pose2d(-8.0, 63.0, Math.toRadians(270.0))
        )
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(1.0) // Wait for gripper to raise
            .lineToY(33.0) // Drive to first delivery position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.25) // Wait for gripper to deliver
            .setTangent(Math.toRadians(150.0)) // Set exit direction
            .splineToConstantHeading(Vector2d(-37.0, 24.0), Math.toRadians(270.0)) // Spline around submersible corner
            .setTangent(Math.toRadians(270.0)) // Continue moving in the correct direction
            .splineToConstantHeading(Vector2d(-47.5, 12.0), Math.toRadians(90.0)) // Continue spline to position behind first sample
            .splineToConstantHeading(Vector2d(-47.5, 53.0), Math.toRadians(90.0)) // Push first sample to observation zone
            .setTangent(Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-50.5, 53.0), Math.toRadians(270.0)) // Make a short loop to head back to pickup
            .splineToConstantHeading(Vector2d(-50.5, 12.0), Math.toRadians(270.0)) // Drive back to next pickup
            .setTangent(Math.toRadians(270.0))
            .splineToConstantHeading(Vector2d(-57.5, 16.0), Math.toRadians(90.0)) // Make a short loop to position behind second sample
            .splineToConstantHeading(Vector2d(-57.5, 53.0), Math.toRadians(90.0)) // Push second sample to the observation area
//        .setTangent(Math.toRadians(90.0))
//        .splineToConstantHeading(Vector2d(-59.5, 53.0), Math.toRadians(270.0)) // Make a short loop to head back to pickup
//        .splineToConstantHeading(Vector2d(-59.5, 12.0), Math.toRadians(270.0)) // Drive back to next pickup
//        .setTangent(Math.toRadians(270.0))
//        .splineToConstantHeading(Vector2d(-63.0, 16.0), Math.toRadians(90.0)) // Make a short loop to position behind third sample
//        .splineToConstantHeading(Vector2d(-63.0, 53.0), Math.toRadians(90.0)) // Push third sample to the observation area
            .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-40.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.4)
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(-6.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.25)
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-40.0, 56.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-40.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.4)
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(-4.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.25)
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-40.0, 48.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-40.0, 56.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-40.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.4)
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(-2.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.25)
            .setTangent(Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-56.0, 58.0), Math.toRadians(180.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(trajectory)
    }
}