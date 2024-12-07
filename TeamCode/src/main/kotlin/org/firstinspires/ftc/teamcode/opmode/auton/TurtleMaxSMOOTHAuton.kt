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
@Autonomous(name = "Turtle Max SMOOTH", group = "Turtle")
class TurtleMaxSMOOTHAuton : AutonBase() {
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

//            //PUSH SAMPLES TO OBSERVATION ZONE OLD
//            .setTangent(Math.toRadians(120.0)) // Set exit direction
//            .splineToConstantHeading(Vector2d(-35.0, 28.0), Math.toRadians(270.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Spline around submersible corner
//            //.splineToConstantHeading(Vector2d(-47.0, 14.0), Math.toRadians(90.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
//            .splineToLinearHeading(Pose2d(-47.0, 14.0, Math.toRadians(270.0)), Math.toRadians(90.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
//            .splineToConstantHeading(Vector2d(-47.0, 57.0), Math.toRadians(90.0), slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Push first sample to observation zone
//            .setTangent(Math.toRadians(90.0))
//            .splineToConstantHeading(Vector2d(-43.0, 12.0), Math.toRadians(270.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Drive back to next pickup
//            .setTangent(Math.toRadians(270.0))
//            .splineToConstantHeading(Vector2d(-57.5, 14.0), Math.toRadians(90.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Make a short loop to position behind second sample
//            .splineToConstantHeading(Vector2d(-57.5, 57.0), Math.toRadians(90.0), slowSegmentVelocityConstraint, null) // Push second sample to the observation area
//
//            //PUSH 3RD SAMPLE
//            .splineToConstantHeading(Vector2d(-53.0, 12.0), Math.toRadians(270.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Drive back to next pickup
//            .setTangent(Math.toRadians(270.0))
//            .splineToConstantHeading(Vector2d(-63.5, 14.0), Math.toRadians(90.0), optimalSegmentVelocityConstraint, slowSegmentAccelerationConstraint) // Make a short loop to position behind second sample
//            .splineToConstantHeading(Vector2d(-60.5, 56.0), Math.toRadians(90.0), slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)//, null, slowSegmentAccelerationConstraint) // Push second sample to the observation area

            //PUSH SAMPLES TO OBSERVATION ZONE
            .setTangent(Math.toRadians(120.0)) // Set exit direction
            .splineToConstantHeading(Vector2d(-35.0, 28.0), Math.toRadians(270.0), null, null) // Spline around submersible corner
            .splineToConstantHeading(Vector2d(-41.0, 16.0), Math.toRadians(180.0), slowestSegmentVelocityConstraint, null) //waypoint for smooth curve
            .splineToConstantHeading(Vector2d(-47.0, 20.0), Math.toRadians(90.0), slowestSegmentVelocityConstraint, null)  //lineup to push first sample

            .splineToConstantHeading(Vector2d(-47.0, 48.0), Math.toRadians(90.0), fastSegmentVelocityConstraint, null) // Push first sample to observation zone
            .splineToConstantHeading(Vector2d(-45.0, 53.0), Math.toRadians(0.0), slowestSegmentVelocityConstraint, null) //waypoint for smooth curve
            .splineToConstantHeading(Vector2d(-43.0, 48.0), Math.toRadians(270.0), slowestSegmentVelocityConstraint, null) //waypoint for smooth curve

            .splineToConstantHeading(Vector2d(-43.0, 17.0), Math.toRadians(270.0), fastSegmentVelocityConstraint, null) // Drive back to next pickup
            .splineToConstantHeading(Vector2d(-50.0, 16.0), Math.toRadians(180.0), slowestSegmentVelocityConstraint, null) // waypoint Drive back to next pickup
            .splineToConstantHeading(Vector2d(-57.0, 20.0), Math.toRadians(90.0), slowestSegmentVelocityConstraint, null) // waypoint Drive back to next pickup

            .splineToConstantHeading(Vector2d(-57.5, 48.0), Math.toRadians(90.0), fastSegmentVelocityConstraint, null) // Push second sample to the observation area
            .splineToConstantHeading(Vector2d(-55.5, 53.0), Math.toRadians(0.0), slowestSegmentVelocityConstraint, null) // waypoint Push second sample to the observation area
//            .splineToConstantHeading(Vector2d(-53.5, 48.0), Math.toRadians(270.0), slowestSegmentVelocityConstraint, null) // waypoint Push second sample to the observation area
//
//            //PUSH 3RD SAMPLE
//            .splineToConstantHeading(Vector2d(-53.5, 17.0), Math.toRadians(270.0), fastSegmentVelocityConstraint, null) // Drive back to next pickup
//            .splineToConstantHeading(Vector2d(-58.0, 16.0), Math.toRadians(180.0), slowestSegmentVelocityConstraint, null) // waypoint Drive back to next pickup
//            .splineToConstantHeading(Vector2d(-63.5, 20.0), Math.toRadians(90.0), slowestSegmentVelocityConstraint, null) // waypoint Drive back to next pickup
//
//            .splineToConstantHeading(Vector2d(-63.5, 48.0), Math.toRadians(90.0), fastSegmentVelocityConstraint, null)// Push third sample to the observation area
//            .splineToConstantHeading(Vector2d(-60.5, 53.0), Math.toRadians(0.0), slowestSegmentVelocityConstraint, null) // waypoint Push third sample to the observation area


            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 2
            .splineToLinearHeading(Pose2d(-44.0, 57.0, Math.toRadians(90.0)), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            //.waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-44.0, 63.0), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 2 ON HIGH CHAMBER
            .setTangent(Math.toRadians(-30.0))
            .splineToSplineHeading(Pose2d(-6.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 3
            .splineToLinearHeading(Pose2d(-6.0, 33.01, Math.toRadians(-90.0)), Math.toRadians(120.0)) //adding this changes rotation direction
            .splineToLinearHeading(Pose2d(-44.0, 57.0, Math.toRadians(93.0)), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            //.waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-44.0, 63.0), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 3 ON HIGH CHAMBER
            .setTangent(Math.toRadians(-30.0))
            .splineToSplineHeading(Pose2d(-4.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 4
            .splineToLinearHeading(Pose2d(-4.0, 33.01, Math.toRadians(-90.0)), Math.toRadians(120.0)) //adding this changes rotation direction
            .splineToLinearHeading(Pose2d(-44.0, 57.0, Math.toRadians(93.0)), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            //.waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-44.0, 63.0), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

            //DELIVER SPECIMEN 4 ON HIGH
            .setTangent(Math.toRadians(-30.0))
            .splineToSplineHeading(Pose2d(-2.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(0.1)

            //ROTATE TO PRE-PICKUP POSITION AND PICKUP SPECIMEN 5
            .splineToLinearHeading(Pose2d(-2.0, 33.01, Math.toRadians(-90.0)), Math.toRadians(120.0)) //adding this changes rotation direction
            .splineToLinearHeading(Pose2d(-44.0, 57.0, Math.toRadians(93.0)), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            //.waitSeconds(1.0)
            .splineToConstantHeading(Vector2d(-44.0, 63.0), Math.toRadians(90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(0.2)

//            //DELIVER SPECIMEN 5 ON HIGH
//            .setTangent(Math.toRadians(-30.0))
//            .splineToSplineHeading(Pose2d(0.0, 33.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))//, slowSegmentVelocityConstraint, slowSegmentAccelerationConstraint)
//            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
//            .waitSeconds(0.1)
//
//            //PARK
//            .setTangent(Math.toRadians(90.0))
//            .splineToConstantHeading(Vector2d(-56.0, 58.0), Math.toRadians(180.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(trajectory)
    }
}