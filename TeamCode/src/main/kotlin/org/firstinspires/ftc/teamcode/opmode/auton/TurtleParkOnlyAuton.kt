package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.opmode.AutonBase

@Disabled
@Suppress("Unused")
@Autonomous(name = "Turtle Park Only", group = "Turtle")
class TurtleParkOnlyAuton : AutonBase() {
    private val initialPose = Pose2d(-8.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        waitForStart()

        if (isStopRequested) return

        runBlocking(
            drive.actionBuilder(initialPose)
                .strafeToConstantHeading(Vector2d(-60.0, 60.0))
                .build()
        )
    }
}