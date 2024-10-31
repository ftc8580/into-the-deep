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

@Autonomous(group = "CyberDragons")
class TurtleMULTIPLESpecimenAuton : OpModeBase() {
    private val startingHeading = Math.toRadians(90.0)
    private val pickupHeading = Math.toRadians (270.0)
    private val startingY = 63.5
    private val deliveryY = 30.0
    private val startingX = -8.0

    private val parkX = -60.0
    private val parkY = 75.0

    private val alignPickupSpecimenX = -50.0
    private val alignPickupSpecimenY = 40.0
    private val pickupSpecimenX = -50.0
    private val pickupSpecimenY = 55.0
    private val pickupSpecimenYClose = 57.0



    override fun initialize() {
        initHardware(true)

        val startingPose = Pose2d(startingX, startingY, startingHeading)
        val deliverPreloadSpecimenPose = Pose2d(startingX, 34.0, startingHeading)
        val deliverPreloadSpecimenPoseClose = Pose2d (startingX, deliveryY, Math.toRadians(80.0))
        val parkPose = Pose2d(parkX, parkY, startingHeading)
        val alignForPickup = Pose2d(alignPickupSpecimenX, alignPickupSpecimenY, pickupHeading)
        val pickupPose = Pose2d(pickupSpecimenX, pickupSpecimenY, pickupHeading)
        val pickupPoseClose = Pose2d(pickupSpecimenX, pickupSpecimenYClose, pickupHeading)

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

        val deliverToAlign = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPoseClose)
            .lineToLinearHeading(deliverPreloadSpecimenPose)
            .lineToLinearHeading(alignForPickup)
            .build()

        val alignToPickup = mecanumDrive.trajectorySequenceBuilder(alignForPickup)
            .lineToLinearHeading(pickupPose)
            .build()

        val pickupClose = mecanumDrive.trajectorySequenceBuilder(pickupPose)
            .lineToLinearHeading(
                pickupPoseClose,
               CDMecanumDrive.getVelocityConstraint(MAX_VEL * 0.6, MAX_ANG_VEL * 0.6, TRACK_WIDTH),
               CDMecanumDrive.getAccelerationConstraint(MAX_ACCEL * 0.6)
            )
            .build()

        val pickupToDeliver = mecanumDrive.trajectorySequenceBuilder(pickupPoseClose)
            .lineToLinearHeading(pickupPose)
            .lineToLinearHeading(alignForPickup)
            .lineToLinearHeading(deliverPreloadSpecimenPose)
            .build()

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliverPreloadSpecimenPoseClose)
            .lineToLinearHeading(parkPose)
            .build()

        mecanumDrive.poseEstimate = startingPose

        schedule(
            SequentialCommandGroup(
                //Deliver Preloaded
                HighChamberPosition(gripperSubsystem),
                WaitCommand(1000),
                //ParallelCommandGroup()
                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequence),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequenceClose),
                PickupPosition(gripperSubsystem),
                WaitCommand(500),

                //Pickup Specimen 1 From Observation Zone
                FollowTrajectorySequence(mecanumDrive, deliverToAlign),
                WaitCommand(1000),
                FollowTrajectorySequence(mecanumDrive, alignToPickup),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, pickupClose),
                WaitCommand(500),
                HighChamberPosition(gripperSubsystem),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, pickupToDeliver),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequenceClose),
                PickupPosition(gripperSubsystem),
                WaitCommand(1000),

                //Pickup Specimen 2 from Observation Zone
                FollowTrajectorySequence(mecanumDrive, deliverToAlign),
                WaitCommand(1000),
                FollowTrajectorySequence(mecanumDrive, alignToPickup),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, pickupClose),
                WaitCommand(500),
                HighChamberPosition(gripperSubsystem),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, pickupToDeliver),
                WaitCommand(500),
                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequenceClose),
                PickupPosition(gripperSubsystem),
                WaitCommand(1000),

//                //Pickup Specimen 2 from Observation Zone
//                FollowTrajectorySequence(mecanumDrive, deliverToAlign),
//                WaitCommand(1000),
//                FollowTrajectorySequence(mecanumDrive, alignToPickup),
//                WaitCommand(500),
//                FollowTrajectorySequence(mecanumDrive, pickupClose),
//                WaitCommand(500),
//                HighChamberPosition(gripperSubsystem),
//                WaitCommand(500),
//                FollowTrajectorySequence(mecanumDrive, pickupToDeliver),
//                WaitCommand(500),
//                FollowTrajectorySequence(mecanumDrive, deliverPreLoadSpecimenTrajectorySequenceClose),
//                PickupPosition(gripperSubsystem),
//                WaitCommand(1000),

                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence)
            )
        )
    }
}