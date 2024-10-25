package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.transfer.ExtendOut
import org.firstinspires.ftc.teamcode.command.transfer.RotateUp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

class NetZoneAuton : OpModeBase() {
    override fun initialize() {
        initHardware()

        // Start Pose
        val startingPose = Pose2d(30.0, 63.25, Math.toRadians(270.0))

        val pickupFirstSamplePose = Pose2d(30.0, 26.75, Math.toRadians(0.0))
        val pickupSecondSamplePose = Pose2d(42.0, 26.75, Math.toRadians(0.0))
        val pickupThirdSamplePose = Pose2d(54.0, 26.75, Math.toRadians(0.0))

        val deliveryPose = Pose2d(52.0, 52.0, Math.toRadians(45.0))
        val parkPose = Pose2d(24.0, 14.0, Math.toRadians(180.0))

        val pickupFirstSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(pickupFirstSamplePose)
            .build()

        val deliverTrajectorySequences = listOf(pickupFirstSamplePose, pickupSecondSamplePose, pickupThirdSamplePose).map {
            mecanumDrive.trajectorySequenceBuilder(it)
                .lineToLinearHeading(deliveryPose)
                .build()
        }

        val pickupTrajectorySequences = listOf(pickupFirstSampleTrajectorySequence) + listOf(pickupSecondSamplePose, pickupThirdSamplePose).map {
            mecanumDrive.trajectorySequenceBuilder(deliveryPose)
                .lineToLinearHeading(it)
                .build()
        }

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(parkPose)
            .build()

        // TODO: All of the pickup and deliver actions. This is just the driving part.
        schedule(
            FollowTrajectorySequence(mecanumDrive, pickupTrajectorySequences[0]),
            FollowTrajectorySequence(mecanumDrive, deliverTrajectorySequences[0]),
            FollowTrajectorySequence(mecanumDrive, pickupTrajectorySequences[1]),
            FollowTrajectorySequence(mecanumDrive, deliverTrajectorySequences[1]),
            FollowTrajectorySequence(mecanumDrive, pickupTrajectorySequences[2]),
            FollowTrajectorySequence(mecanumDrive, deliverTrajectorySequences[2]),
            FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence),
        )
    }
}