package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.specimen.HighChamberPosition
import org.firstinspires.ftc.teamcode.command.specimen.PickupPosition
import org.firstinspires.ftc.teamcode.command.transfer.PositionAutoRung
import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_VEL
import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.config.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
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

        val deliverPreloadSpecimenPose = Pose2d (4.0, 33.0, startingHeading)
        val deliverPreloadSpecimenPoseClose = Pose2d (4.0, 30.0, startingHeading)

        //Trajectories
        val deliverPreLoadSpecimenTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(deliverPreloadSpecimenPose)
            .build()

        val deliverPreLoadSpecimenTrajectorySequenceClose = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPose)
            .lineToLinearHeading(
                deliverPreloadSpecimenPoseClose,
                CDMecanumDrive.getVelocityConstraint(MAX_VEL * 0.6, MAX_ANG_VEL * 0.6, TRACK_WIDTH),
                CDMecanumDrive.getAccelerationConstraint(MAX_ACCEL * 0.6)
            )
            .build()

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPose)
            .lineToLinearHeading(Pose2d(startingX, 44.0, parkHeading))
            .lineToLinearHeading(Pose2d(30.0, 44.0, parkHeading))
            .lineToLinearHeading(Pose2d(30.0, 14.0, parkHeading))
            .lineToLinearHeading(Pose2d(14.0, 12.0, parkHeading))
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