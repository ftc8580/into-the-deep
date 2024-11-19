package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.actions.RotationPosition
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.ArmRotationPosition

@Suppress("Unused")
@Autonomous(name = "Narwhal Max", group = "Narwhal")
class NarwhalMaxAuton : AutonBase() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

        initialize(initialPose)

        val action = drive.actionBuilder(initialPose)
            .lineToY(62.0)
            .afterTime(0.0, RotationPosition(armRotationSubsystem, ArmRotationPosition.DRIVE))
            .waitSeconds(2.0)
            .lineToY(60.0)
            .afterTime(0.0, RotationPosition(armRotationSubsystem, ArmRotationPosition.HOME))
            .waitSeconds(2.0)
            .lineToY(58.0)
            .afterTime(0.0, RotationPosition(armRotationSubsystem, ArmRotationPosition.PARK))
            .waitSeconds(2.0)
            .lineToY(56.0)
            .afterTime(0.0, RotationPosition(armRotationSubsystem, ArmRotationPosition.HOME))
            .waitSeconds(2.0)
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}