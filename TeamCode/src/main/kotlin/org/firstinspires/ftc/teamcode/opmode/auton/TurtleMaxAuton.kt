package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmode.AutonBase

@Suppress("Unused")
@Autonomous(name = "Turtle Park Only", group = "Turtle")
class TurtleMaxAuton : AutonBase() {
    private val initialPose = Pose2d(-8.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val trajectory = drive.actionBuilder(
            Pose2d(-8.0, 63.0, Math.toRadians(270.0))
        )
            .lineToY(33.0)
            .waitSeconds(1.0)
            .lineToY(38.0)
            .splineToConstantHeading(Vector2d(-38.0, 30.0), Math.toRadians(270.0))
            .lineToY(12.0)
            .strafeToConstantHeading(Vector2d(-47.5, 12.0))
            .strafeToConstantHeading(Vector2d(-47.5, 53.0))
            .strafeToConstantHeading(Vector2d(-47.5, 12.0))
            .strafeToConstantHeading(Vector2d(-57.5, 12.0))
            .strafeToConstantHeading(Vector2d(-57.5, 53.0))
            .strafeToConstantHeading(Vector2d(-57.5, 12.0))
            .strafeToConstantHeading(Vector2d(-63.0, 12.0))
            .strafeToConstantHeading(Vector2d(-63.0, 53.0))
            .strafeToConstantHeading(Vector2d(-63.0, 48.0))
            .splineToSplineHeading(Pose2d(-40.0, 58.0, Math.toRadians(90.0)), Math.toRadians(0.0))
            .strafeToConstantHeading(Vector2d(-40.0, 63.0))
            .splineToSplineHeading(Pose2d(-8.0, 33.0, Math.toRadians(270.0)), Math.toRadians(270.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(trajectory)
    }
}