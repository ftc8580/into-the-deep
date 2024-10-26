package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
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

@Autonomous(group = "CyberDragons")
class NarwhalParkOnlyAuton : OpModeBase() {
    private val startHeading = Math.toRadians(90.0)
    private val startingX = 36.0
    private val startingY = 63.25

    private val parkHeading = Math.toRadians(180.0)
    private val parkX = 24.0
    private val parkY = 14.0

    override fun initialize() {
        initHardware()

        val startingPose = Pose2d(startingX, startingY, startHeading)
        val preParkPose = Pose2d(startingX, parkY, parkHeading)
        val parkPose = Pose2d(parkX, parkY, parkHeading)

        mecanumDrive.poseEstimate = startingPose

        val parkTrajectorySequence = mecanumDrive.trajectorySequenceBuilder(startingPose)
            .lineToLinearHeading(preParkPose)
            .lineToLinearHeading(parkPose)
            .build()

        schedule(
            SequentialCommandGroup(
                PositionDrive(viperArmSubsystem, activeIntakeSubsystem),
                FollowTrajectorySequence(mecanumDrive, parkTrajectorySequence),
                PositionAutoRung(viperArmSubsystem, activeIntakeSubsystem)
            )
        )
    }
}