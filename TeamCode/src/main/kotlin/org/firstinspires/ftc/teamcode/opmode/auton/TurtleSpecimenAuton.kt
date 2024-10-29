package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.specimen.HighChamberPosition
import org.firstinspires.ftc.teamcode.command.specimen.PickupPosition
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(group = "CyberDragons")
class TurtleSpecimenAuton : OpModeBase() {
    private val heading = Math.toRadians(90.0)
    private val pickupY = 63.5
    private val deliveryY = 37.5
    private val deliveryX = -16.0

    private val parkX = -60.0
    private val parkY = 62.0

    override fun initialize() {
        initHardware(true)

        val startingPose = Pose2d(deliveryX, pickupY, heading)
        val initialDeliveryPose = Pose2d(deliveryX, deliveryY, heading)
        val parkPose = Pose2d(parkX, parkY, heading)

        val initialDeliveryTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(initialDeliveryPose)
            .build()
        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(initialDeliveryPose)
            .lineToLinearHeading(parkPose)
            .build()

        mecanumDrive.poseEstimate = startingPose

        schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, initialDeliveryTrajectorySequence),
//                    HighChamberPosition(gripperSubsystem)
                ),
//                PickupPosition(gripperSubsystem),
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence)
            )
        )
    }
}