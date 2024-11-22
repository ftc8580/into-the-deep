package org.firstinspires.ftc.teamcode.opmode.auton

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.actions.GripperPosition
import org.firstinspires.ftc.teamcode.actions.buildParkArmPositionAction
import org.firstinspires.ftc.teamcode.actions.buildPreParkArmPositionAction
import org.firstinspires.ftc.teamcode.opmode.AutonBase
import org.firstinspires.ftc.teamcode.subsystem.GripperHeight

@Disabled
@Suppress("Unused")
@Autonomous(name = "Narwhal Single Specimen", group = "Narwhal")
class NarwhalSpecimenAuton : AutonBase() {
    private val initialPose = Pose2d(40.0, 63.5, Math.toRadians(270.0))

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize(initialPose)

        val action = drive.actionBuilder(initialPose)
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HIGH)) // Raise gripper to delivery position
            .waitSeconds(0.5)
            .splineToConstantHeading(Vector2d(5.0, 33.0), Math.toRadians(270.0)) // Spline to specimen delivery position
            .afterTime(0.0, GripperPosition(gripperSubsystem, GripperHeight.HOME)) // Hang specimen
            .waitSeconds(0.3) // Wait for specimen to hook
            .setTangent(Math.toRadians(90.0)) // Set exit heading
            .splineToLinearHeading(Pose2d(28.0, 40.0, Math.toRadians(200.0)), Math.toRadians(0.0)) // Intermediate pose around submersible
            .afterTime(0.2, armSubsystems.buildPreParkArmPositionAction()) // Move arm to pre-park position while driving
            .splineToLinearHeading(Pose2d(23.5, 12.0, Math.toRadians(180.0)), Math.toRadians(180.0)) // Finish drive into park position
            .afterTime(0.0, armSubsystems.buildParkArmPositionAction()) // Move arm to contact rung
            .waitSeconds(2.0) // Ensure wait until arm is moved
            .build()

        waitForStart()

        if (isStopRequested) return

        runBlocking(action)
    }
}