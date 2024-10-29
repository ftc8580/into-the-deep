package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.specimen.HighChamberPosition
import org.firstinspires.ftc.teamcode.command.specimen.PickupPosition
import org.firstinspires.ftc.teamcode.command.transfer.EjectSample
import org.firstinspires.ftc.teamcode.command.transfer.IntakeSample
import org.firstinspires.ftc.teamcode.command.transfer.PositionDeliveryToUpperBasket
import org.firstinspires.ftc.teamcode.command.transfer.PositionDrive
import org.firstinspires.ftc.teamcode.command.transfer.PositionPickup
import org.firstinspires.ftc.teamcode.command.transfer.PositionPickupGround
import org.firstinspires.ftc.teamcode.command.transfer.PositionAutoRung
import org.firstinspires.ftc.teamcode.opmode.OpModeBase

@Suppress("UNUSED")
@Autonomous(group = "CyberDragons")
class NarwhalSpecimenAuton : OpModeBase() {
    private val startingHeading = Math.toRadians(90.0)
    private val startingX = 8.0
    private val startingY = 63.25

    private val parkHeading = Math.toRadians(180.0)

    override fun initialize() {
        initHardware(true)

        // Poses
        val startingPose = Pose2d(startingX, startingY, startingHeading)

        mecanumDrive.poseEstimate = startingPose

        val deliverPreloadSpecimenPose = Pose2d (4.0, 32.0, startingHeading)
        val deliverPreloadSpecimenPoseClose = Pose2d (4.0, 30.0, startingHeading)

        //Trajectories
        val deliverPreLoadSpecimenTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(deliverPreloadSpecimenPose)
            .build()

        val deliverPreLoadSpecimenTrajectorySequenceClose = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPose)
            .lineToLinearHeading(deliverPreloadSpecimenPoseClose)
            .build()

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPose)
            .lineToLinearHeading(Pose2d(startingX, 44.0, parkHeading))
            .lineToLinearHeading(Pose2d(36.0, 44.0, parkHeading))
            .lineToLinearHeading(Pose2d(36.0, 14.0, parkHeading))
            .lineToLinearHeading(Pose2d(24.0, 14.0, parkHeading))
            .build()

        schedule(
            SequentialCommandGroup(
                HighChamberPosition(gripperSubsystem),
                WaitCommand(1000),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequence)
                ),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequenceClose),
                PickupPosition(gripperSubsystem),
                WaitCommand(1000),
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence),
                PositionAutoRung(viperArmSubsystem, activeIntakeSubsystem)
            )
        )
    }
}