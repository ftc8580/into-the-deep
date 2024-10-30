package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.specimen.HighChamberPosition
import org.firstinspires.ftc.teamcode.command.specimen.PickupPosition
import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_ANG_VEL
import org.firstinspires.ftc.teamcode.config.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.config.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(group = "CyberDragons")
class TurtleSpecimenAuton : OpModeBase() {
    private val startingHeading = Math.toRadians(90.0)
    private val pickupY = 63.5
    private val deliveryY = 30.0
    private val startingX = -8.0

    private val parkX = -60.0
    private val parkY = 75.0


    override fun initialize() {
        initHardware(true)

        val startingPose = Pose2d(startingX, pickupY, startingHeading)
        val deliverPreloadSpecimenPose = Pose2d(startingX, 34.0, startingHeading)
        val deliverPreloadSpecimenPoseClose = Pose2d (startingX, deliveryY, startingHeading)
        val parkPose = Pose2d(parkX, parkY, startingHeading)

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

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPoseClose)
            .lineToLinearHeading(parkPose)
            .build()

        mecanumDrive.poseEstimate = startingPose

        schedule(
            SequentialCommandGroup(
                HighChamberPosition(gripperSubsystem),
                WaitCommand(1000),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequence),
                ),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequenceClose),
                PickupPosition(gripperSubsystem),
                WaitCommand(1000),
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence)
            )
        )
    }
}