package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.transfer.ExtendOut
import org.firstinspires.ftc.teamcode.command.transfer.RotateUp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(group = "CyberDragons")
class NetZoneAuton : OpModeBase() {
    override fun initialize() {
        initHardware()

        // Start Pose
        val startingPose = Pose2d(30.0, 63.25, Math.toRadians(270.0))

        mecanumDrive.poseEstimate = startingPose

        val pickupFirstSamplePose = Pose2d(30.0, 26.75, Math.toRadians(0.0))
        val pickupSecondSamplePose = Pose2d(42.0, 26.75, Math.toRadians(0.0))
        val pickupThirdSamplePose = Pose2d(54.0, 26.75, Math.toRadians(0.0))

        val deliveryPose = Pose2d(52.0, 52.0, Math.toRadians(45.0))
        val parkPose = Pose2d(24.0, 14.0, Math.toRadians(180.0))

        val pickupFirstSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(pickupFirstSamplePose)
            .build()

        val pickupSecondSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(pickupSecondSamplePose)
            .build()

        val pickupThirdSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(pickupThirdSamplePose)
            .build()

        val deliverFirstTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupFirstSamplePose)
            .lineToLinearHeading(deliveryPose)
            .build()

        val deliverSecondTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupSecondSamplePose)
            .lineToLinearHeading(deliveryPose)
            .build()

        val deliverThirdTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupThirdSamplePose)
            .lineToLinearHeading(deliveryPose)
            .build()

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(parkPose)
            .build()

        // TODO: All of the pickup and deliver actions. This is just the driving part.
        schedule(
            SequentialCommandGroup(
                FollowTrajectorySequence(mecanumDrive, pickupFirstSampleTrajectorySequence),
                FollowTrajectorySequence(mecanumDrive, deliverFirstTrajectorySequence),
                FollowTrajectorySequence(mecanumDrive, pickupSecondSampleTrajectorySequence),
                FollowTrajectorySequence(mecanumDrive, deliverSecondTrajectorySequence),
                FollowTrajectorySequence(mecanumDrive, pickupThirdSampleTrajectorySequence),
                FollowTrajectorySequence(mecanumDrive, deliverThirdTrajectorySequence),
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence),
            )
        )
    }
}