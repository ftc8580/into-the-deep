package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.transfer.EjectSample
import org.firstinspires.ftc.teamcode.command.transfer.IntakeSample
import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToUpperBasket
import org.firstinspires.ftc.teamcode.command.transfer.PositionDrive
import org.firstinspires.ftc.teamcode.command.transfer.PositionPickup
import org.firstinspires.ftc.teamcode.command.transfer.PositionPickupGround
import org.firstinspires.ftc.teamcode.command.transfer.PositionAutoRung
import org.firstinspires.ftc.teamcode.opmode.OpModeBase

@Autonomous(group = "CyberDragons")
class NarwhalAuton : OpModeBase() {
    private val startingX = 8.0
    private val startingY = 63.25
    private val spikeY = 25.75

    override fun initialize() {
        initHardware()

        // Start Pose
        val startingPose = Pose2d(startingX, startingY, Math.toRadians(90.0))

        mecanumDrive.poseEstimate = startingPose

        val deliverPreloadSpecimenPose = Pose2d (startingX, 34.0, Math.toRadians(90.0))
        val clearSubmersiblePose = Pose2d (startingX, 44.0, Math.toRadians(90.0) )

        val pickupFirstSamplePose = Pose2d(48.0, 34.0, Math.toRadians(270.0))
        val pickupFirstSamplePoseIntake = Pose2d(48.0, 40.0, Math.toRadians(270.0))
        val pickupSecondSamplePose = Pose2d(58.0, 34.0, Math.toRadians(270.0))
        val pickupSecondSamplePoseIntake = Pose2d(58.0, 40.0, Math.toRadians(270.0))
        val pickupThirdSamplePose = Pose2d(53.0, spikeY, Math.toRadians(0.0))
        val pickupThirdSamplePoseIntake = Pose2d(47.0, 24.0, Math.toRadians(0.0))

        val deliveryPose = Pose2d(60.0, 60.0, Math.toRadians(225.0))
        val preParkPose = Pose2d(36.0, 14.0, Math.toRadians(180.0))
        val parkPose = Pose2d(24.0, 14.0, Math.toRadians(180.0))

        val deliverPreLoadSpecimenTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(deliverPreloadSpecimenPose)
            .build()

        val clearSubmersibleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPose)
            .lineToLinearHeading(clearSubmersiblePose)
            .build()

        val pickupFirstSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(clearSubmersiblePose)
            .lineToLinearHeading(pickupFirstSamplePose)
            .build()

        val pickupSecondSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(pickupSecondSamplePose)
            .build()

        val pickupThirdSampleTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(pickupThirdSamplePose)
            .build()

        val intakeFirstTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupFirstSamplePose)
            .lineToLinearHeading(pickupFirstSamplePoseIntake)
            .build()

        val deliverFirstTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupFirstSamplePoseIntake)
            .lineToLinearHeading(deliveryPose)
            .build()

        val intakeSecondTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupSecondSamplePose)
            .lineToLinearHeading(pickupSecondSamplePoseIntake)
            .build()

        val deliverSecondTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupSecondSamplePoseIntake)
            .lineToLinearHeading(deliveryPose)
            .build()

        val intakeThirdTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupThirdSamplePose)
            .lineToLinearHeading(pickupThirdSamplePoseIntake)
            .build()

        val deliverThirdTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(pickupThirdSamplePoseIntake)
            .lineToLinearHeading(deliveryPose)
            .build()

        val preParkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .splineToLinearHeading(preParkPose, Math.toRadians(180.0))
            .build()

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPose)
            .lineToLinearHeading(parkPose)
            .build()

        schedule(
            SequentialCommandGroup(

                ParallelCommandGroup(
                    //TODO: Move Gripper to deliver high position
                    FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequence)
                ),
                //TODO: Move gripper down to pickup position. This should put it on the chamber.
                FollowTrajectorySequence(mecanumDrive, clearSubmersibleTrajectorySequence),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pickupFirstSampleTrajectorySequence),
                    PositionPickup(viperArmSubsystem, activeIntakeSubsystem)
                ),
                PositionPickupGround(viperArmSubsystem, activeIntakeSubsystem),
                ParallelCommandGroup(
                    IntakeSample(activeIntakeSubsystem),
                    FollowTrajectorySequence(mecanumDrive, intakeFirstTrajectorySequence)
                ),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverFirstTrajectorySequence),
                    PositionDeliveryToUpperBasket(viperArmSubsystem, activeIntakeSubsystem)
                ),
                EjectSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pickupSecondSampleTrajectorySequence),
                    PositionPickup(viperArmSubsystem, activeIntakeSubsystem)
                ),
                PositionPickupGround(viperArmSubsystem, activeIntakeSubsystem),
                ParallelCommandGroup(
                    IntakeSample(activeIntakeSubsystem),
                    FollowTrajectorySequence(mecanumDrive, intakeSecondTrajectorySequence)
                ),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverSecondTrajectorySequence),
                    PositionDeliveryToUpperBasket(viperArmSubsystem, activeIntakeSubsystem)
                ),
                EjectSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pickupThirdSampleTrajectorySequence),
                    PositionPickup(viperArmSubsystem, activeIntakeSubsystem)
                ),
                PositionPickupGround(viperArmSubsystem, activeIntakeSubsystem),
                ParallelCommandGroup(
                    IntakeSample(activeIntakeSubsystem),
                    FollowTrajectorySequence(mecanumDrive, intakeThirdTrajectorySequence)
                ),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverThirdTrajectorySequence),
                    PositionDeliveryToUpperBasket(viperArmSubsystem, activeIntakeSubsystem)
                ),
                EjectSample(activeIntakeSubsystem),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, preParkTrajectorySequence),
                    PositionDrive(viperArmSubsystem, activeIntakeSubsystem)
                ),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence),
                    PositionAutoRung(viperArmSubsystem, activeIntakeSubsystem)
                )
            )
        )
    }
}