package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.command.FollowTrajectorySequence
import org.firstinspires.ftc.teamcode.command.specimen.HighChamberPosition
import org.firstinspires.ftc.teamcode.command.specimen.PickupPosition
import org.firstinspires.ftc.teamcode.command.transfer.PositionDrive
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(group = "CyberDragons")
class TurtleParkOnlyAuton : OpModeBase() {
    private val heading = Math.toRadians(270.0)
    private val startingX = -16.0
    private val startingY = 63.5

    private val parkX = -60.0
    private val parkY = 60.0

    override fun initialize() {
        initHardware()

        val startingPose = Pose2d(startingX, startingY, heading)
        val parkPose = Pose2d(parkX, parkY, heading)
        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(parkPose)
            .build()

        mecanumDrive.poseEstimate = startingPose

        schedule(
            SequentialCommandGroup(
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence)
            )
        )
    }
}