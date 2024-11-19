package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmode.AutonBase

@Suppress("Unused")
@Autonomous(name = "Narwhal Park Only", group = "Narwhal")
class NarwhalParkOnlyAuton : AutonBase() {
    private val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        waitForStart()

        if (isStopRequested) return

        runBlocking(
            drive.actionBuilder(initialPose)
                .splineTo(Vector2d(44.0, 7.5), Math.toRadians(180.0))
                .splineTo(Vector2d(23.5, 7.5), Math.toRadians(180.0))
                .build()
        )
    }
}