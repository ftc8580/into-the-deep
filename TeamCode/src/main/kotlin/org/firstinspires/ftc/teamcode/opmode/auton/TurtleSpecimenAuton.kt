package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight

@Disabled
@Suppress("Unused")
@Autonomous(name = "Turtle Single Specimen", group = "Turtle")
class TurtleSpecimenAuton : AutonBase() {
    private val initialPose = Pose2d(-8.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val action = drive.actionBuilder(initialPose)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH))
            .waitSeconds(1.0)
            .lineToY(33.0)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME))
            .waitSeconds(1.0)
            .lineToY(38.0)
            .strafeTo(Vector2d(-60.0, 60.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}