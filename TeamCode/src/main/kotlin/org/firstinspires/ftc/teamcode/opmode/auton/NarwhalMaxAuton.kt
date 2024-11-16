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

        val action = drive.actionBuilder(initialPose)
            .afterTime(0.0, RotationPosition(armRotationSubsystem, ArmRotationPosition.DRIVE))
            .splineTo(Vector2d(44.0, 7.5), Math.toRadians(0.0))
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}