package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight

//@Disabled
@Suppress("Unused")
@Autonomous(name = "Turtle Max FAST KM", group = "Turtle")
class TurtleMaxFASTAutonKM : AutonBase() {
    private val initialPose = Pose2d(-8.0, 63.0, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val trajectory = drive.actionBuilder(
            Pose2d(-8.0, 63.0, Math.toRadians(270.0))
        )
            //DELIVER PRELOADED
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            //.waitSeconds(1.1) // Wait for gripper to raise
            .lineToY(33.0) // Drive to first delivery position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1) // Wait for gripper to deliver

            //PUSH SAMPLES TO OBSERVATION ZONE
            .setTangent(Math.toRadians(120.0))
            ///.splineToConstantHeading(Vector2d(-8.0, 38.0), Math.toRadians(270.0))
            ///.setTangent(Math.toRadians(150.0)) // Set exit direction
            .splineToConstantHeading(Vector2d(-37.0, 28.0), Math.toRadians(270.0)) // Spline around submersible corner
            //.setTangent(Math.toRadians(270.0)) // Continue moving in the correct direction
            .splineToConstantHeading(Vector2d(-44.0, 12.0), Math.toRadians(110.0))
            //.splineToConstantHeading(Vector2d(-47.5, 12.0), Math.toRadians(90.0), slowSegmentVelocityConstraint) // Continue spline to position behind first sample
            //.splineToConstantHeading(Vector2d(-47.5, 57.0), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-52.0, 57.0), Math.toRadians(90.0), null, slowSegmentAccelerationConstraint) // Push first sample to observation zone
            .setTangent(Math.toRadians(90.0))
            //.splineToConstantHeading(Vector2d(-45.5, 30.0), Math.toRadians(270.0), slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Make a short loop to head back to pickup
            .splineToConstantHeading(Vector2d(-38.5, 14.0), Math.toRadians(270.0), null, slowSegmentAccelerationConstraint) // Drive back to next pickup
            .setTangent(Math.toRadians(270.0))
            .splineToConstantHeading(Vector2d(-57.5, 14.0), Math.toRadians(90.0), slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Make a short loop to position behind second sample
            .splineToConstantHeading(Vector2d(-57.5, 57.0), Math.toRadians(90.0), null, slowSegmentAccelerationConstraint) // Push second sample to the observation area
//        .setTangent(Math.toRadians(90.0))
//        .splineToConstantHeading(Vector2d(-59.5, 53.0), Math.toRadians(270.0)) // Make a short loop to head back to pickup
//        .splineToConstantHeading(Vector2d(-59.5, 12.0), Math.toRadians(270.0)) // Drive back to next pickup
//        .setTangent(Math.toRadians(270.0))
//        .splineToConstantHeading(Vector2d(-63.0, 16.0), Math.toRadians(90.0)) // Make a short loop to position behind third sample
//        .splineToConstantHeading(Vector2d(-63.0, 53.0), Math.toRadians(90.0)) // Push third sample to the observation area

            //PUSH 3RD SAMPLE
            .splineToConstantHeading(Vector2d(-46.5, 14.0), Math.toRadians(270.0))//, null, slowSegmentAccelerationConstraint) // Drive back to next pickup
            .setTangent(Math.toRadians(270.0))
            .splineToConstantHeading(Vector2d(-63.5, 14.0), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Make a short loop to position behind second sample
            .splineToConstantHeading(Vector2d(-63.5, 57.0), Math.toRadians(90.0))//, null, slowSegmentAccelerationConstraint) // Push second sample to the observation area

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 2
            .splineToLinearHeading(Pose2d(-42.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            //.waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-42.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 2 ON HIGH CHAMBER
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(-6.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 3
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-40.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            //.waitSeconds(1.0)
            ////.setTangent(Math.toRadians(90.0))
            ////.splineToLinearHeading(Pose2d(-40.0, 56.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-42.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 3 ON HIGH CHAMBER
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(-4.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 4
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-42.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            //.waitSeconds(1.0)
            ////.setTangent(Math.toRadians(90.0))
            ////.splineToLinearHeading(Pose2d(-40.0, 56.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-42.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 4 ON HIGH
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(-2.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 5
            .setTangent(Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-42.0, 58.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            //.waitSeconds(1.0)
            ////.setTangent(Math.toRadians(90.0))
            ////.splineToLinearHeading(Pose2d(-40.0, 56.0, Math.toRadians(90.0)), Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-42.0, 63.0), Math.toRadians(90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 5 ON HIGH
            .setTangent(-45.0)
            .splineToSplineHeading(Pose2d(0.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //PARK
            .setTangent(Math.toRadians(90.0))
            .splineToConstantHeading(Vector2d(-56.0, 58.0), Math.toRadians(180.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(trajectory)
    }
}