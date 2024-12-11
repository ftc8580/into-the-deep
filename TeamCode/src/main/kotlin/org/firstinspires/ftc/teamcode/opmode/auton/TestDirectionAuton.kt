package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.opmode.AutonBase

@Disabled
@Suppress("Unused")
@Autonomous(name = "Test Direction Auton")
class TestDirectionAuton : AutonBase() {
    private val initialPose = Pose2d(-8.0, 60.0, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        waitForStart()

        if (isStopRequested) return

        runBlocking(
            drive.actionBuilder(initialPose)
                .lineToY(30.0)
                .turnTo(Math.toRadians(90.0))
                .lineToY(60.0)
//                .turnTo(Math.toRadians(270.0))
                .build()
        )
    }
}