package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.transfer.EjectSample
import org.firstinspires.ftc.teamcode.command.transfer.IntakeSample
import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToUpperBasket
import org.firstinspires.ftc.teamcode.command.transfer.PositionHome
import org.firstinspires.ftc.teamcode.command.transfer.PositionPickup
import org.firstinspires.ftc.teamcode.opmode.OpModeBase

@Autonomous(group = "CyberDragons")
class NarwhalAuton : OpModeBase() {
    private val startingX = 34.5
    private val spikeY = 25.75

    override fun initialize() {
        initHardware()

        // Start Pose
        val startingPose = Pose2d(startingX, 63.25, Math.toRadians(270.0))

        mecanumDrive.poseEstimate = startingPose

        val pickupFirstSamplePose = Pose2d(startingX, spikeY, Math.toRadians(0.0))
        val pickupSecondSamplePose = Pose2d(44.5, spikeY, Math.toRadians(0.0))
        val pickupThirdSamplePose = Pose2d(54.5, spikeY, Math.toRadians(0.0))

        val deliveryPose = Pose2d(60.0, 60.0, Math.toRadians(45.0))
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
            .splineToLinearHeading(parkPose, Math.toRadians(180.0))
            .build()

        schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pickupFirstSampleTrajectorySequence),
                    PositionPickup(viperArmSubsystem, activeIntakeSubsystem)
                ),
                IntakeSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverFirstTrajectorySequence),
                    PositionDeliveryToUpperBasket(viperArmSubsystem, activeIntakeSubsystem)
                ),
                EjectSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pickupSecondSampleTrajectorySequence),
                    PositionPickup(viperArmSubsystem, activeIntakeSubsystem)
                ),
                IntakeSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverSecondTrajectorySequence),
                    PositionDeliveryToUpperBasket(viperArmSubsystem, activeIntakeSubsystem)
                ),
                EjectSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pickupThirdSampleTrajectorySequence),
                    PositionPickup(viperArmSubsystem, activeIntakeSubsystem)
                ),
                IntakeSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverThirdTrajectorySequence),
                    PositionDeliveryToUpperBasket(viperArmSubsystem, activeIntakeSubsystem)
                ),
                EjectSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence),
                    PositionHome(viperArmSubsystem, activeIntakeSubsystem)
                )
            )
        )
    }
}