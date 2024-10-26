package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.specimen.HighChamberPosition
import org.firstinspires.ftc.teamcode.command.specimen.PickupPosition
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

class TurtleAuton : OpModeBase() {
    private val rotationPickup = Math.toRadians(270.0)
    private val rotationDelivery = Math.toRadians(90.0)
    private val pickupY = 63.5
    private val prePickupY = 58.0
    private val deliveryY = 33.5
    private val pickupX = -34.0
    private val pushStartY = 15.0
    private val prePickupPose = Pose2d(pickupX, prePickupY, rotationPickup)
    private val pickupPose = Pose2d(pickupX, pickupY, rotationPickup)
    private val deliveryStartingX = -16.0

    override fun initialize() {
        initHardware()

        var deliveryX = deliveryStartingX

        val startingPose = Pose2d(deliveryX, pickupY, rotationDelivery)
        val initialDeliveryPose = Pose2d(deliveryX, deliveryY, rotationDelivery)
        val deliveryPoses = (1..3).map {
            deliveryX += 2.0
            Pose2d(deliveryX, deliveryY, rotationDelivery)
        }
        val parkPose = Pose2d(-48.0, 60.0, rotationPickup)

        val initialDeliveryTrajectorySequence = startingPose.lineToLinearHeading(initialDeliveryPose)
        val pushSampleTrajectorySequence = getPushSampleTrajectory(initialDeliveryPose)
        val firstPickup = pushSampleTrajectorySequence.end().toPickup()
        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(deliveryPoses[2])
            .lineToLinearHeading(parkPose)
            .build()

        mecanumDrive.poseEstimate = startingPose

        schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, initialDeliveryTrajectorySequence),
                    HighChamberPosition(gripperSubsystem)
                ),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, pushSampleTrajectorySequence),
                    PickupPosition(gripperSubsystem)
                ),
                FollowTrajectorySequence(mecanumDrive, firstPickup),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliveryPoses[0].fromPickup()),
                    HighChamberPosition(gripperSubsystem)
                ),
                PickupPosition(gripperSubsystem),
                FollowTrajectorySequence(mecanumDrive, deliveryPoses[0].toPickup()),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliveryPoses[1].fromPickup()),
                    HighChamberPosition(gripperSubsystem)
                ),
                PickupPosition(gripperSubsystem),
                FollowTrajectorySequence(mecanumDrive, deliveryPoses[1].toPickup()),
                ParallelCommandGroup(
                    FollowTrajectorySequence(mecanumDrive, deliveryPoses[2].fromPickup()),
                    HighChamberPosition(gripperSubsystem)
                ),
                PickupPosition(gripperSubsystem),
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence)
            )
        )
    }

    private fun getPushSampleTrajectory(startPose: Pose2d): TrajectorySequence {
        val clearSubmersibleY = deliveryY + 4.0
        val clearSubmersibleX = -36.0
        val pushSampleX = -48.0
        val pushEndY = 54.0

        return mecanumDrive.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(Pose2d(deliveryStartingX, clearSubmersibleY, rotationDelivery))
            .lineToLinearHeading(Pose2d(clearSubmersibleX, clearSubmersibleY, rotationPickup))
            .lineToLinearHeading(Pose2d(clearSubmersibleX, pushStartY, rotationPickup))
            .lineToLinearHeading(Pose2d(pushSampleX, pushStartY, rotationPickup))
            .lineToLinearHeading(Pose2d(pushSampleX, pushEndY, rotationPickup))
            .build()
    }

    private fun Pose2d.lineToLinearHeading(endPose: Pose2d): TrajectorySequence {
        return mecanumDrive.trajectorySequenceBuilder(this)
            .lineToLinearHeading(endPose)
            .build()
    }

    private fun Pose2d.toPickup(): TrajectorySequence {
        return mecanumDrive.trajectorySequenceBuilder(this)
            .lineToLinearHeading(prePickupPose)
            .lineToLinearHeading(pickupPose)
            .build()
    }

    private fun Pose2d.fromPickup(): TrajectorySequence {
        return mecanumDrive.trajectorySequenceBuilder(pickupPose)
            .lineToLinearHeading(this)
            .build()
    }
}